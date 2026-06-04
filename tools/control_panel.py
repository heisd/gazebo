#!/usr/bin/env python3
"""Web control panel for the two-AGV warehouse fleet.

The panel is a single self-contained page (no build step, no third-party
Python deps) that talks to the *real* ``AGVScheduler`` over its operator
command channel and status feed:

    * POST /api/command  -> publishes ``/agv/agv_command``
                            ({cmd: task|goto|charge|return|stop, ...})
    * GET  /api/status   -> latest ``/agv/scheduler_status`` snapshot
    * GET  /api/layout   -> static warehouse geometry for the map

Two backends share that interface:

    ROS mode (``--ros``):  creates an rclpy node and bridges the two topics.
                           Use this on the robot host where ROS 2 is installed.

    SIM mode (default):    runs the unmodified scheduler in-process behind the
                           ROS stubs from ``tools/closed_loop_sim.py`` and a
                           kinematic world, so the panel is fully driveable
                           (and automatically testable) without ROS/Gazebo.

Usage::

    python3 tools/control_panel.py                 # sim mode, http://0.0.0.0:8080
    python3 tools/control_panel.py --ros           # bridge to a live scheduler
    python3 tools/control_panel.py --port 9000 --rate 4
"""

import argparse
import json
import os
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS = os.path.join(ROOT, "tools")
PKG = os.path.join(ROOT, "src", "agv_scheduler")
LAYOUT_FILE = os.path.join(PKG, "config", "warehouse_layout.yaml")
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)


# --------------------------------------------------------------------------
# Warehouse geometry (served to the browser so it can draw the map)
# --------------------------------------------------------------------------
def load_layout():
    import yaml
    with open(LAYOUT_FILE, "r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh) or {}
    layout = raw.get("warehouse_layout", raw)
    shelves = {
        sid: {
            "center": list(s.get("center", [0, 0])),
            "pickup": list(s.get("pickup", s.get("center", [0, 0]))),
        }
        for sid, s in (layout.get("shelves", {}) or {}).items()
    }
    station = layout.get("station", {})
    charging = layout.get("charging", {})
    zones = {
        zid: {
            "type": z.get("type", "exclusive"),
            "polygon": [list(p) for p in z.get("polygon", [])],
        }
        for zid, z in (layout.get("traffic_zones", {}) or {}).items()
    }
    return {
        "shelves": shelves,
        "station": list(station.get("dock", station.get("center", [8, 0]))),
        "charging": list(charging.get("center", [9, -8])),
        "zones": zones,
        "wait_points": {
            k: list(v) for k, v in (layout.get("wait_points", {}) or {}).items()
        },
    }


# --------------------------------------------------------------------------
# Conflict scenarios — deliberately put the two AGVs on crossing paths so the
# reservation / right-of-way machinery has to resolve the contention.  Each is
# just an ordered list of operator commands, so it behaves identically whether
# the panel is driving the in-process sim or a live ROS scheduler.
# --------------------------------------------------------------------------
SCENARIOS = {
    "cross": {
        "label": "Cross-corridor tasks",
        "desc": "Pins opposite-side shelf jobs to each AGV (agv_01→D1 south, "
                "agv_02→A1 north). Both must traverse the central EXCLUSIVE "
                "corridor on crossing paths and then share the delivery dock. "
                "Resolution: the corridor reservation serialises them and the "
                "loser retreats to a wait point — both jobs still complete.",
        "commands": [
            {"cmd": "task", "shelf": "D1", "priority": 6, "agv_id": "agv_01"},
            {"cmd": "task", "shelf": "A1", "priority": 6, "agv_id": "agv_02"},
        ],
    },
    "headon": {
        "label": "Head-on goto",
        "desc": "Mirror-image manual moves (agv_01→(-9,-5), agv_02→(-9,5)) "
                "that cross inside the corridor. Resolution: one AGV reserves "
                "the corridor and crosses while the other holds in place, then "
                "resumes once it clears — no collision, no deadlock.",
        "commands": [
            {"cmd": "goto", "agv_id": "agv_01", "x": -9.0, "y": -5.0},
            {"cmd": "goto", "agv_id": "agv_02", "x": -9.0, "y": 5.0},
        ],
    },
    "reset": {
        "label": "Reset → home",
        "desc": "Stop both AGVs and send them back to their home parking "
                "spots, ready to re-run a scenario.",
        "commands": [
            {"cmd": "return", "agv_id": "agv_01"},
            {"cmd": "return", "agv_id": "agv_02"},
        ],
    },
}


def run_scenario(backend, name):
    sc = SCENARIOS.get(name)
    if not sc:
        return {"ok": False, "error": f"unknown scenario '{name}'"}
    for cmd in sc["commands"]:
        backend.command(dict(cmd))
    return {"ok": True, "ran": name, "commands": len(sc["commands"])}


# --------------------------------------------------------------------------
# SIM backend: real scheduler logic behind the closed-loop ROS stubs
# --------------------------------------------------------------------------
class SimBackend:
    mode = "sim"

    def __init__(self, speed=1.5, rate=1.0, dt=0.1, starts=None):
        import closed_loop_sim as sim
        self.sim = sim
        sim.install_stubs()
        sim.WORLD.reset()
        sim.WORLD.speed = speed
        sim.SimClock.t = 0.0
        sim.FakeLogger.SINK = []
        sim.FakeNode.PARAM_OVERRIDES = sim.load_params(LAYOUT_FILE)

        self.mod = sim.import_scheduler()
        self.sched = self.mod.AGVScheduler()
        self.agv_ids = list(self.sched.agvs.keys())

        default_starts = {"agv_01": (10.8, 4.0), "agv_02": (10.8, -4.0)}
        starts = starts or default_starts
        for i, (aid, agv) in enumerate(self.sched.agvs.items()):
            sim.WORLD.frame_to_aid[agv.base_frame] = aid
            sim.WORLD.set_pose(aid, *starts.get(aid, (10.8, 4.0 - 2.0 * i)))
        for aid, client in self.sched.nav_clients.items():
            client.aid = aid

        self.odom_cbs = {t: cb for t, cb in self.sched._subs}
        self.lock = threading.Lock()
        self.dt = dt
        self.rate = max(0.1, float(rate))
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    # -- simulation loop ----------------------------------------------------
    def _tick(self):
        sim = self.sim
        sim.SimClock.t = round(sim.SimClock.t + self.dt, 4)
        arrivals = sim.WORLD.step(self.dt)
        for aid, agv in self.sched.agvs.items():
            x, y = sim.WORLD.pos[aid]
            cb = self.odom_cbs.get(agv.odom_topic)
            if cb:
                cb(sim.make_odom(x, y, sim.WORLD.vel.get(aid, 0.0)))
        for timer in self.sched._timers:
            period, cb, next_due = timer
            while sim.SimClock.t + 1e-9 >= next_due:
                cb()
                next_due += period
            timer[2] = next_due
        for aid, gh in arrivals:
            gh.get_result_async().set_result(
                sim.FakeResult(sim.World.SUCCEEDED))

    def _run(self):
        while not self._stop:
            with self.lock:
                self._tick()
            time.sleep(self.dt / self.rate)

    def start(self):
        self._thread.start()

    def step_for(self, sim_seconds, on_tick=None):
        """Advance the sim deterministically (used by the verifier).

        ``on_tick`` (if given) is called with this backend after every tick so
        a caller can audit per-tick invariants (e.g. conflict resolution).
        """
        ticks = int(round(sim_seconds / self.dt))
        for _ in range(ticks):
            with self.lock:
                self._tick()
            if on_tick is not None:
                on_tick(self)

    # -- operator interface -------------------------------------------------
    def command(self, payload):
        s = self.sim.FakeString()
        s.data = json.dumps(payload, ensure_ascii=False)
        with self.lock:
            self.sched._on_command(s)
        return {"ok": True}

    def status(self):
        with self.lock:
            raw = self.sched._pubs.get("/agv/scheduler_status")
            st = json.loads(raw) if raw else {}
            st["mode"] = self.mode
            st["sim_time"] = round(self.sim.SimClock.t, 1)
            st["log"] = [
                {"t": round(t, 1), "level": lv, "text": txt}
                for (t, lv, txt) in self.sim.FakeLogger.SINK[-60:]
            ]
        return st


# --------------------------------------------------------------------------
# ROS backend: bridge to a live scheduler on the robot host
# --------------------------------------------------------------------------
class RosBackend:
    mode = "ros"

    def __init__(self):
        import rclpy
        from std_msgs.msg import String
        self._String = String
        rclpy.init()
        self.node = rclpy.create_node("agv_web_panel")
        self.pub_cmd = self.node.create_publisher(
            String, "/agv/agv_command", 10)
        self._status = {}
        self._log = deque(maxlen=60)
        self.node.create_subscription(
            String, "/agv/scheduler_status", self._on_status, 10)
        self._rclpy = rclpy
        self._thread = threading.Thread(
            target=lambda: rclpy.spin(self.node), daemon=True)

    def _on_status(self, msg):
        try:
            self._status = json.loads(msg.data)
        except Exception:
            pass

    def start(self):
        self._thread.start()

    def command(self, payload):
        m = self._String()
        m.data = json.dumps(payload, ensure_ascii=False)
        self.pub_cmd.publish(m)
        return {"ok": True}

    def status(self):
        st = dict(self._status)
        st["mode"] = self.mode
        st.setdefault("log", list(self._log))
        return st


# --------------------------------------------------------------------------
# HTTP server
# --------------------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    backend = None
    layout = None

    def log_message(self, *args):
        pass  # quiet

    def _send(self, code, body, ctype="application/json"):
        data = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(200, INDEX_HTML, "text/html; charset=utf-8")
        elif self.path == "/api/layout":
            self._send(200, json.dumps(self.layout))
        elif self.path == "/api/scenarios":
            self._send(200, json.dumps({
                k: {"label": v["label"], "desc": v["desc"]}
                for k, v in SCENARIOS.items()}))
        elif self.path == "/api/status":
            self._send(200, json.dumps(self.backend.status()))
        else:
            self._send(404, json.dumps({"error": "not found"}))

    def do_POST(self):
        if self.path not in ("/api/command", "/api/scenario"):
            self._send(404, json.dumps({"error": "not found"}))
            return
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            payload = json.loads(raw or b"{}")
        except Exception as exc:
            self._send(400, json.dumps({"error": f"bad json: {exc}"}))
            return
        try:
            if self.path == "/api/scenario":
                result = run_scenario(self.backend, payload.get("name", ""))
            else:
                result = self.backend.command(payload)
            self._send(200, json.dumps(result))
        except Exception as exc:
            self._send(500, json.dumps({"error": str(exc)}))


INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>AGV Fleet Control Panel</title>
<style>
  :root { --bg:#0f1419; --panel:#1a212b; --line:#2b3645; --txt:#d7e0ea;
          --muted:#8a97a8; --a1:#4fc3f7; --a2:#ffb74d; --ok:#66bb6a;
          --warn:#ef5350; --acc:#7e57c2; }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--txt);
         font:14px/1.45 system-ui,Segoe UI,Roboto,sans-serif; }
  header { padding:10px 16px; background:var(--panel);
           border-bottom:1px solid var(--line); display:flex; gap:14px;
           align-items:center; }
  header h1 { font-size:16px; margin:0; font-weight:600; }
  #mode { font-size:12px; color:var(--muted); }
  .wrap { display:flex; gap:14px; padding:14px; align-items:flex-start;
          flex-wrap:wrap; }
  .col { display:flex; flex-direction:column; gap:14px; }
  .card { background:var(--panel); border:1px solid var(--line);
          border-radius:8px; padding:12px; }
  .card h2 { font-size:12px; text-transform:uppercase; letter-spacing:.06em;
             color:var(--muted); margin:0 0 8px; }
  canvas { background:#0b0f14; border-radius:6px; display:block;
           touch-action:none; cursor:crosshair; }
  table { width:100%; border-collapse:collapse; font-size:13px; }
  th,td { text-align:left; padding:4px 6px; border-bottom:1px solid var(--line); }
  th { color:var(--muted); font-weight:500; }
  .pill { display:inline-block; padding:1px 8px; border-radius:10px;
          font-size:11px; background:#26303d; }
  .bat { font-variant-numeric:tabular-nums; }
  button { background:#26303d; color:var(--txt); border:1px solid var(--line);
           border-radius:6px; padding:6px 10px; cursor:pointer; font-size:13px; }
  button:hover { border-color:#3d4d61; }
  button.sel { background:var(--acc); border-color:var(--acc); color:#fff; }
  button.danger { color:#ffd9d7; }
  label { font-size:12px; color:var(--muted); }
  input,select { background:#0b0f14; color:var(--txt); border:1px solid var(--line);
                 border-radius:6px; padding:5px 7px; font-size:13px; }
  .row { display:flex; gap:8px; align-items:center; flex-wrap:wrap; }
  .seg { display:flex; gap:6px; }
  #log { height:150px; overflow:auto; font:12px/1.4 ui-monospace,monospace;
         background:#0b0f14; border-radius:6px; padding:8px; }
  #log .WARN { color:#ffcc80; } #log .ERROR { color:#ef9a9a; }
  #log .t { color:var(--muted); }
  .k { color:var(--muted); } .small { font-size:12px; color:var(--muted); }
  .dotA{color:var(--a1)} .dotB{color:var(--a2)}
</style>
</head>
<body>
<header>
  <h1>AGV Fleet Control Panel</h1>
  <span id="mode">connecting…</span>
  <span id="clock" class="small"></span>
  <span id="stats" class="small"></span>
</header>
<div class="wrap">
  <div class="col">
    <div class="card">
      <h2>Warehouse map <span class="small">(click to drive selected AGV)</span></h2>
      <canvas id="map" width="560" height="460"></canvas>
    </div>
    <div class="card">
      <h2>Activity log</h2>
      <div id="log"></div>
    </div>
  </div>
  <div class="col">
    <div class="card">
      <h2>Fleet</h2>
      <table id="fleet"><thead><tr>
        <th>AGV</th><th>State</th><th>Battery</th><th>Task</th><th>Pos</th>
      </tr></thead><tbody></tbody></table>
    </div>
    <div class="card">
      <h2>Manual control</h2>
      <div class="row" style="margin-bottom:8px">
        <label>Selected AGV</label>
        <div class="seg" id="agvSel"></div>
      </div>
      <div class="row" style="margin-bottom:8px">
        <label>Goto</label>
        x <input id="gx" type="number" step="0.5" style="width:70px" value="0">
        y <input id="gy" type="number" step="0.5" style="width:70px" value="0">
        <button onclick="goto()">Drive there</button>
      </div>
      <div class="seg">
        <button onclick="cmd('charge')">⚡ Charge</button>
        <button onclick="cmd('return')">⌂ Return home</button>
        <button class="danger" onclick="cmd('stop')">■ Stop</button>
      </div>
    </div>
    <div class="card">
      <h2>Dispatch task</h2>
      <div class="row">
        <label>Shelf</label><select id="shelf"></select>
        <label>Prio</label><input id="prio" type="number" min="1" max="9"
          value="5" style="width:54px">
        <label>AGV</label><select id="taskAgv"><option value="">auto</option></select>
        <button onclick="dispatch()">Queue task</button>
      </div>
    </div>
    <div class="card">
      <h2>Conflict tests <span class="small">(force a path conflict)</span></h2>
      <div class="seg" id="scenarios" style="flex-wrap:wrap"></div>
      <div id="scDesc" class="small" style="margin-top:8px;min-height:32px"></div>
    </div>
    <div class="card">
      <h2>Task queue (<span id="pending">0</span>)</h2>
      <table id="queue"><tbody></tbody></table>
    </div>
  </div>
</div>
<script>
let LAYOUT=null, SEL="agv_01", BOUNDS=null;
const COL={agv_01:"#4fc3f7", agv_02:"#ffb74d"};
const $=s=>document.querySelector(s);

async function init(){
  LAYOUT=await (await fetch('/api/layout')).json();
  // shelf + agv dropdowns
  const sh=$('#shelf');
  Object.keys(LAYOUT.shelves).sort().forEach(s=>{
    const o=document.createElement('option');o.value=s;o.textContent=s;sh.appendChild(o);});
  // map bounds
  const xs=[],ys=[];
  for(const s of Object.values(LAYOUT.shelves)){xs.push(s.center[0]);ys.push(s.center[1]);}
  xs.push(LAYOUT.station[0],LAYOUT.charging[0],11,-11);
  ys.push(LAYOUT.station[1],LAYOUT.charging[1],9,-9);
  BOUNDS={minx:Math.min(...xs)-1.5,maxx:Math.max(...xs)+1.5,
          miny:Math.min(...ys)-1.5,maxy:Math.max(...ys)+1.5};
  $('#map').addEventListener('click',onMapClick);
  await initScenarios();
  poll();
  setInterval(poll, 500);
}
async function initScenarios(){
  let sc; try{sc=await (await fetch('/api/scenarios')).json();}catch(e){return;}
  const box=$('#scenarios');box.innerHTML='';
  for(const [name,info] of Object.entries(sc)){
    const b=document.createElement('button');
    b.textContent=info.label;
    b.onmouseenter=()=>{$('#scDesc').textContent=info.desc;};
    b.onclick=()=>runScenario(name,info);
    box.appendChild(b);
  }
}
async function runScenario(name,info){
  $('#scDesc').textContent='▶ '+info.label+': '+info.desc;
  await fetch('/api/scenario',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({name})});
}
function W2C(x,y){
  const c=$('#map'),pad=8;
  const w=c.width-2*pad,h=c.height-2*pad;
  const px=pad+(x-BOUNDS.minx)/(BOUNDS.maxx-BOUNDS.minx)*w;
  const py=pad+(BOUNDS.maxy-y)/(BOUNDS.maxy-BOUNDS.miny)*h;
  return [px,py];
}
function C2W(px,py){
  const c=$('#map'),pad=8;
  const w=c.width-2*pad,h=c.height-2*pad;
  const x=BOUNDS.minx+(px-pad)/w*(BOUNDS.maxx-BOUNDS.minx);
  const y=BOUNDS.maxy-(py-pad)/h*(BOUNDS.maxy-BOUNDS.miny);
  return [x,y];
}
function onMapClick(e){
  const r=e.target.getBoundingClientRect();
  const [x,y]=C2W(e.clientX-r.left,e.clientY-r.top);
  $('#gx').value=x.toFixed(1); $('#gy').value=y.toFixed(1);
  cmd('goto',{x:+x.toFixed(2),y:+y.toFixed(2)});
}
function selAgv(a){SEL=a;renderAgvButtons();}
function renderAgvButtons(){
  const ids=(window._fleet?Object.keys(window._fleet):['agv_01','agv_02']);
  const box=$('#agvSel');box.innerHTML='';
  const ta=$('#taskAgv');const cur=ta.value;
  ta.innerHTML='<option value="">auto</option>';
  ids.forEach(a=>{
    const b=document.createElement('button');
    b.textContent=a;b.className=(a===SEL?'sel':'');b.onclick=()=>selAgv(a);
    box.appendChild(b);
    const o=document.createElement('option');o.value=a;o.textContent=a;
    ta.appendChild(o);
  });
  ta.value=cur;
}
async function post(payload){
  await fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify(payload)});
}
function cmd(c,extra){post(Object.assign({cmd:c,agv_id:SEL},extra||{}));}
function goto(){cmd('goto',{x:+$('#gx').value,y:+$('#gy').value});}
function dispatch(){
  const p={cmd:'task',shelf:$('#shelf').value,priority:+$('#prio').value};
  if($('#taskAgv').value) p.agv_id=$('#taskAgv').value;
  post(p);
}
async function poll(){
  let st; try{st=await (await fetch('/api/status')).json();}catch(e){return;}
  window._fleet=st.fleet||{};
  if(!$('#agvSel').children.length) renderAgvButtons();
  $('#mode').textContent='backend: '+(st.mode||'?');
  $('#clock').textContent= st.sim_time!=null?('t='+st.sim_time+'s'):'';
  $('#stats').textContent='completed '+(st.completed||0)+' · pending '+(st.pending||0);
  $('#pending').textContent=st.pending||0;
  renderFleet(st); renderQueue(st); renderLog(st); draw(st);
}
function renderFleet(st){
  const tb=$('#fleet tbody');tb.innerHTML='';
  for(const [a,f] of Object.entries(st.fleet||{})){
    const tr=document.createElement('tr');
    const bcol=f.battery<10?'var(--warn)':f.battery<20?'#ffb74d':'var(--txt)';
    let note='';
    if(f.yielding_to)
      note=`<span class="pill" style="background:#5a3a00">⤳ ${f.yielding_to}</span>`;
    else if(f.wait_reason)
      note=`<span class="pill" style="background:#3a2a4a">⏸ ${f.wait_reason}</span>`;
    tr.innerHTML=`<td><span style="color:${COL[a]||'#fff'}">●</span> ${a}</td>
      <td><span class="pill">${f.state}</span> ${note}</td>
      <td class="bat" style="color:${bcol}">${f.battery}%</td>
      <td>${f.task||'<span class=k>—</span>'}</td>
      <td class="small">${f.pos?f.pos[0]+', '+f.pos[1]:''}</td>`;
    tb.appendChild(tr);
  }
}
function renderQueue(st){
  const tb=$('#queue tbody');tb.innerHTML='';
  (st.queue||[]).forEach(q=>{
    const tr=document.createElement('tr');
    tr.innerHTML=`<td>${q.tid}</td><td>${q.shelf}</td>
      <td class="small">${q.status}</td>`;
    tb.appendChild(tr);
  });
}
function renderLog(st){
  const el=$('#log');const at=el.scrollTop+el.clientHeight>=el.scrollHeight-4;
  el.innerHTML=(st.log||[]).map(l=>
    `<div class="${l.level}"><span class="t">${l.t}</span> ${esc(l.text)}</div>`).join('');
  if(at) el.scrollTop=el.scrollHeight;
}
function esc(s){return (s+'').replace(/[<>&]/g,c=>({'<':'&lt;','>':'&gt;','&':'&amp;'}[c]));}
function draw(st){
  const c=$('#map'),g=c.getContext('2d');
  g.clearRect(0,0,c.width,c.height);
  // zones
  for(const z of Object.values(LAYOUT.zones||{})){
    if(!z.polygon.length) continue;
    g.beginPath();
    z.polygon.forEach((p,i)=>{const[x,y]=W2C(p[0],p[1]);i?g.lineTo(x,y):g.moveTo(x,y);});
    g.closePath();
    g.fillStyle=z.type==='exclusive'?'rgba(239,83,80,.08)':'rgba(79,195,247,.06)';
    g.strokeStyle='rgba(120,140,160,.25)';g.fill();g.stroke();
  }
  // shelves
  g.font='10px sans-serif';
  for(const [id,s] of Object.entries(LAYOUT.shelves)){
    const [x,y]=W2C(s.center[0],s.center[1]);
    g.fillStyle='#33414f';g.fillRect(x-12,y-9,24,18);
    g.fillStyle='#9fb0c2';g.textAlign='center';g.fillText(id,x,y+3);
    const [px,py]=W2C(s.pickup[0],s.pickup[1]);
    g.fillStyle='#55657a';g.fillRect(px-2,py-2,4,4);
  }
  // station + charger
  marker(g,LAYOUT.station,'#66bb6a','DOCK');
  marker(g,LAYOUT.charging,'#ab47bc','⚡');
  // agvs
  for(const [a,f] of Object.entries(st.fleet||{})){
    if(!f.pos) continue;
    const [x,y]=W2C(f.pos[0],f.pos[1]);
    if(f.goal){const[gx,gy]=W2C(f.goal[0],f.goal[1]);
      g.strokeStyle=COL[a]||'#fff';g.globalAlpha=.5;g.setLineDash([4,4]);
      g.beginPath();g.moveTo(x,y);g.lineTo(gx,gy);g.stroke();
      g.setLineDash([]);g.globalAlpha=1;
      g.fillStyle=COL[a];g.beginPath();g.arc(gx,gy,3,0,7);g.fill();}
    if(f.wait_point){const[wx,wy]=W2C(f.wait_point[0],f.wait_point[1]);
      g.strokeStyle='#888';g.beginPath();g.arc(wx,wy,5,0,7);g.stroke();}
    g.beginPath();g.arc(x,y,8,0,7);
    g.fillStyle=COL[a]||'#fff';g.globalAlpha=(a===SEL?1:.85);g.fill();
    g.globalAlpha=1;
    if(a===SEL){g.strokeStyle='#fff';g.lineWidth=2;g.stroke();g.lineWidth=1;}
    g.fillStyle='#0b0f14';g.textAlign='center';g.font='9px sans-serif';
    g.fillText(a.slice(-1),x,y+3);
    g.fillStyle=COL[a];g.font='10px sans-serif';
    g.fillText(f.state+' '+f.battery+'%',x,y-12);
  }
}
function marker(g,p,col,txt){
  const [x,y]=W2C(p[0],p[1]);
  g.fillStyle=col;g.beginPath();g.arc(x,y,6,0,7);g.fill();
  g.fillStyle='#cfe;';g.font='10px sans-serif';g.textAlign='center';
  g.fillStyle='#c7d2dd';g.fillText(txt,x,y-10);
}
init();
</script>
</body>
</html>
"""


# --------------------------------------------------------------------------
def build_backend(use_ros):
    if use_ros:
        return RosBackend()
    return SimBackend()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ros", action="store_true",
                    help="bridge to a live ROS scheduler instead of the sim")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--rate", type=float, default=2.0,
                    help="sim speed multiplier (sim mode only)")
    args = ap.parse_args()

    backend = RosBackend() if args.ros else SimBackend(rate=args.rate)
    backend.start()

    Handler.backend = backend
    Handler.layout = load_layout()
    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"[panel] backend={backend.mode}  serving http://{args.host}:"
          f"{args.port}  (Ctrl-C to stop)")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[panel] shutting down")


if __name__ == "__main__":
    main()
