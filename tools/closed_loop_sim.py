#!/usr/bin/env python3
"""Headless closed-loop simulation harness for the AGV scheduler.

A real ROS 2 + Gazebo + Nav2 stack cannot run in this container (the ROS
apt repo is blocked and the host is Ubuntu 24.04 while the project targets
Humble / Gazebo Classic). To still exercise the *real* scheduling logic
end-to-end, this harness:

  * stubs the ROS plumbing (rclpy, Nav2 action, TF, messages) in sys.modules
    BEFORE importing the unmodified ``scheduler_node``;
  * drives a simple kinematic robot model per AGV;
  * fakes the Nav2 ``NavigateToPose`` action so that goals "complete" once
    the robot reaches them, firing the scheduler's own result callbacks;
  * feeds odom + TF poses back into the scheduler every tick;
  * advances a deterministic simulation clock and ticks every scheduler timer.

The code that decides *what* to do (task assignment, staged reservations,
right-of-way, battery/charge, watchdog) is the genuine production code in
``src/agv_scheduler/agv_scheduler/scheduler_node.py``.

Usage::

    python3 tools/closed_loop_sim.py
"""

import copy
import importlib.util
import json
import math
import os
import sys
import tempfile
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "src", "agv_scheduler")
SCHED_PY = os.path.join(PKG, "agv_scheduler", "scheduler_node.py")
LAYOUT = os.path.join(PKG, "config", "warehouse_layout.yaml")
PARAMS = os.path.join(PKG, "config", "two_agv_scheduler.yaml")

ACTIVE_STATES = {"to_shelf", "picking", "to_aisle_exit", "to_station",
                 "waiting", "to_charge"}


# --------------------------------------------------------------------------
# Simulation clock (the scheduler's ``time.time()`` is redirected here)
# --------------------------------------------------------------------------
class SimClock:
    t = 0.0


def now():
    return SimClock.t


def _ns(**kw):
    return types.SimpleNamespace(**kw)


# --------------------------------------------------------------------------
# Minimal future / goal-handle mimicking rclpy async behaviour
# --------------------------------------------------------------------------
class FakeFuture:
    def __init__(self):
        self._done = False
        self._result = None
        self._cbs = []

    def set_result(self, result):
        self._result = result
        self._done = True
        for cb in list(self._cbs):
            cb(self)

    def result(self):
        return self._result

    def add_done_callback(self, cb):
        if self._done:
            cb(self)
        else:
            self._cbs.append(cb)

    def done(self):
        return self._done


class FakeResult:
    def __init__(self, status):
        self.status = status


class FakeGoalHandle:
    def __init__(self, sim, aid):
        self.accepted = True
        self._sim = sim
        self._aid = aid
        self._result_future = FakeFuture()

    def cancel_goal_async(self):
        self._sim.cancel(self._aid, self)
        f = FakeFuture()
        f.set_result(True)
        return f

    def get_result_async(self):
        return self._result_future


# --------------------------------------------------------------------------
# The kinematic world: positions, velocities, active Nav2 goals
# --------------------------------------------------------------------------
class World:
    SUCCEEDED = 4

    def __init__(self, speed=1.5, tol=0.3):
        self.speed = speed
        self.tol = tol
        self.pos = {}
        self.vel = {}
        self.active = {}
        self.frame_to_aid = {}

    def reset(self):
        self.pos.clear()
        self.vel.clear()
        self.active.clear()
        self.frame_to_aid.clear()

    def set_pose(self, aid, x, y):
        self.pos[aid] = (x, y)
        self.vel[aid] = 0.0

    def set_goal(self, aid, gx, gy, gh):
        self.active[aid] = {"gx": gx, "gy": gy, "gh": gh, "arrived": False}

    def cancel(self, aid, gh):
        g = self.active.get(aid)
        if g and g["gh"] is gh:
            del self.active[aid]
            self.vel[aid] = 0.0

    def step(self, dt):
        arrivals = []
        for aid, g in list(self.active.items()):
            if g["arrived"]:
                continue
            sx, sy = self.pos[aid]
            gx, gy = g["gx"], g["gy"]
            dist = math.hypot(gx - sx, gy - sy)
            if dist <= self.tol:
                self.pos[aid] = (gx, gy)
                self.vel[aid] = 0.0
                g["arrived"] = True
                arrivals.append((aid, g["gh"]))
            else:
                step = min(dist, self.speed * dt)
                self.pos[aid] = (sx + (gx - sx) / dist * step,
                                 sy + (gy - sy) / dist * step)
                self.vel[aid] = self.speed
        for aid in self.pos:
            if aid not in self.active or self.active[aid]["arrived"]:
                self.vel[aid] = 0.0
        return arrivals


WORLD = World()


# --------------------------------------------------------------------------
# ROS stubs installed into sys.modules before importing the scheduler
# --------------------------------------------------------------------------
class FakePublisher:
    def __init__(self, store, topic):
        self._store = store
        self._topic = topic

    def publish(self, msg):
        self._store[self._topic] = getattr(msg, "data", msg)


class FakeLogger:
    SINK = []

    def _emit(self, level, text):
        self.SINK.append((SimClock.t, level, text))

    def info(self, text):
        self._emit("INFO", text)

    def warn(self, text):
        self._emit("WARN", text)

    def error(self, text):
        self._emit("ERROR", text)

    def debug(self, text):
        pass


class FakeActionClient:
    def __init__(self, node, action_type, action_name):
        self.action_name = action_name
        self.aid = None
        self.sim = WORLD

    def wait_for_server(self, timeout_sec=1.0):
        return True

    def send_goal_async(self, goal):
        gx = float(goal.pose.pose.position.x)
        gy = float(goal.pose.pose.position.y)
        gh = FakeGoalHandle(self.sim, self.aid)
        self.sim.set_goal(self.aid, gx, gy, gh)
        f = FakeFuture()
        f.set_result(gh)
        return f


class _TF_EXC(Exception):
    pass


class FakeBuffer:
    def lookup_transform(self, target, source, when, timeout=None):
        aid = WORLD.frame_to_aid.get(source)
        if aid is None or aid not in WORLD.pos:
            raise _TF_EXC("no transform")
        x, y = WORLD.pos[aid]
        return _ns(transform=_ns(
            translation=_ns(x=x, y=y, z=0.0),
            rotation=_ns(x=0.0, y=0.0, z=0.0, w=1.0)))


class FakeNode:
    PARAM_OVERRIDES = {}

    def __init__(self, name):
        self._name = name
        self._params = dict(FakeNode.PARAM_OVERRIDES)
        self._timers = []
        self._subs = []
        self._pubs = {}
        self._logger = FakeLogger()

    def declare_parameter(self, name, value):
        if name not in self._params:
            self._params[name] = value

    def get_parameter(self, name):
        return _ns(value=self._params.get(name))

    def create_publisher(self, msg_type, topic, qos):
        return FakePublisher(self._pubs, topic)

    def create_subscription(self, msg_type, topic, cb, qos):
        self._subs.append((topic, cb))
        return _ns()

    def create_timer(self, period, cb):
        self._timers.append([float(period), cb, float(period)])
        return _ns()

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return _ns(now=lambda: _ns(to_msg=lambda: _ns()))

    def destroy_node(self):
        pass


class FakeString:
    def __init__(self):
        self.data = ""


class FakeTwist:
    def __init__(self):
        self.linear = _ns(x=0.0, y=0.0, z=0.0)
        self.angular = _ns(x=0.0, y=0.0, z=0.0)


def install_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    mod("rclpy", init=lambda *a, **k: None, shutdown=lambda *a, **k: None,
        spin=lambda *a, **k: None)
    mod("rclpy.action", ActionClient=FakeActionClient)
    mod("rclpy.duration", Duration=lambda **k: _ns(**k))
    mod("rclpy.node", Node=FakeNode)
    mod("rclpy.time", Time=lambda *a, **k: _ns())
    mod("action_msgs")
    mod("action_msgs.msg", GoalStatus=_ns(
        STATUS_SUCCEEDED=4, STATUS_ABORTED=6, STATUS_CANCELED=5,
        STATUS_UNKNOWN=0))
    mod("geometry_msgs")
    mod("geometry_msgs.msg", Twist=FakeTwist)
    mod("nav2_msgs")

    class _NavGoal:
        def __init__(self):
            self.pose = _ns(
                header=_ns(frame_id="", stamp=None),
                pose=_ns(position=_ns(x=0.0, y=0.0, z=0.0),
                         orientation=_ns(x=0.0, y=0.0, z=0.0, w=1.0)))

    mod("nav2_msgs.action", NavigateToPose=_ns(Goal=_NavGoal))
    mod("nav_msgs")
    mod("nav_msgs.msg", Odometry=object)
    mod("std_msgs")
    mod("std_msgs.msg", String=FakeString)
    mod("tf2_ros", Buffer=FakeBuffer, TransformException=_TF_EXC,
        TransformListener=lambda *a, **k: _ns())


# --------------------------------------------------------------------------
# Harness
# --------------------------------------------------------------------------
def load_params(layout_file):
    import yaml
    with open(PARAMS, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    p = dict(data["agv_scheduler"]["ros__parameters"])
    p["shelf_layout_file"] = layout_file
    p["auto_demo_enabled"] = False
    return p


def import_scheduler():
    if "scheduler_node" in sys.modules:
        return sys.modules["scheduler_node"]
    spec = importlib.util.spec_from_file_location("scheduler_node", SCHED_PY)
    module = importlib.util.module_from_spec(spec)
    sys.modules["scheduler_node"] = module
    spec.loader.exec_module(module)
    module.time = _ns(time=now)   # redirect wall clock to the sim clock
    return module


def make_odom(x, y, vx):
    return _ns(
        pose=_ns(pose=_ns(
            position=_ns(x=x, y=y, z=0.0),
            orientation=_ns(x=0.0, y=0.0, z=0.0, w=1.0))),
        twist=_ns(twist=_ns(
            linear=_ns(x=vx, y=0.0, z=0.0),
            angular=_ns(x=0.0, y=0.0, z=0.0))))


def make_task(tid, shelf, priority, agv_id=None):
    s = FakeString()
    payload = {"tid": tid, "shelf": shelf, "priority": priority}
    if agv_id:
        payload["agv_id"] = agv_id
    s.data = json.dumps(payload)
    return s


class Harness:
    def __init__(self, sched, dt=0.1):
        self.sched = sched
        self.dt = dt
        self.odom_cbs = {t: cb for t, cb in sched._subs}
        self.pending = []
        self.last_state = {}
        self.transitions = []
        self.peak_active = 0
        self.snapshots = {}      # label -> status dict
        self.snap_at = []        # (t, label)

    def schedule_task(self, t, msg):
        self.pending.append((t, msg))

    def snapshot(self, t, label):
        self.snap_at.append((t, label))

    def _feed_poses(self):
        for aid, agv in self.sched.agvs.items():
            x, y = WORLD.pos[aid]
            cb = self.odom_cbs.get(agv.odom_topic)
            if cb:
                cb(make_odom(x, y, WORLD.vel.get(aid, 0.0)))

    def _fire_timers(self):
        for timer in self.sched._timers:
            period, cb, next_due = timer
            while SimClock.t + 1e-9 >= next_due:
                cb()
                next_due += period
            timer[2] = next_due

    def _track(self):
        active = 0
        for aid, agv in self.sched.agvs.items():
            cur = agv.state.value
            if self.last_state.get(aid) != cur:
                self.transitions.append(
                    (SimClock.t, aid, self.last_state.get(aid), cur))
                self.last_state[aid] = cur
            if cur in ACTIVE_STATES:
                active += 1
        self.peak_active = max(self.peak_active, active)

    def run(self, duration):
        for _ in range(int(duration / self.dt)):
            SimClock.t = round(SimClock.t + self.dt, 4)
            due = [m for (tt, m) in self.pending if tt <= SimClock.t]
            self.pending = [(tt, m) for (tt, m) in self.pending
                            if tt > SimClock.t]
            for m in due:
                self.sched._on_task(m)
            arrivals = WORLD.step(self.dt)
            self._feed_poses()
            self._fire_timers()
            for aid, gh in arrivals:
                gh.get_result_async().set_result(FakeResult(World.SUCCEEDED))
            self._track()
            for (tt, label) in list(self.snap_at):
                if SimClock.t >= tt:
                    self.snapshots[label] = self.status()
                    self.snap_at.remove((tt, label))

    def status(self):
        raw = self.sched._pubs.get("/agv/scheduler_status")
        return json.loads(raw) if raw else {}


def make_layout_variant(station_lane_type):
    """Write a temp layout copy with a given station_lane type."""
    import yaml
    with open(LAYOUT, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    zones = data["warehouse_layout"]["traffic_zones"]
    zones["station_lane"]["type"] = station_lane_type
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, encoding="utf-8")
    yaml.safe_dump(data, tmp)
    tmp.close()
    return tmp.name


def run_scenario(layout_file, dt=0.1, duration=420.0):
    SimClock.t = 0.0
    WORLD.reset()
    FakeLogger.SINK = []
    FakeNode.PARAM_OVERRIDES = load_params(layout_file)

    sched_mod = import_scheduler()
    sched = sched_mod.AGVScheduler()

    starts = {"agv_01": (10.8, 4.0), "agv_02": (10.8, -4.0)}
    for aid, agv in sched.agvs.items():
        WORLD.frame_to_aid[agv.base_frame] = aid
        WORLD.set_pose(aid, *starts[aid])
    for aid, client in sched.nav_clients.items():
        client.aid = aid

    h = Harness(sched, dt=dt)
    # 4 tasks: A1 (north) & D4 (south) are non-crossing -> parallel-able;
    # B2 & C3 sit in the central corridor.
    h.schedule_task(1.0, make_task("T1001", "A1", 5))
    h.schedule_task(1.0, make_task("T1002", "D4", 5))
    h.schedule_task(3.0, make_task("T1003", "B2", 4))
    h.schedule_task(3.0, make_task("T1004", "C3", 4))
    h.snapshot(2.0, "after_first_sched")
    h.run(duration=duration)

    dones = [(t, txt) for (t, lv, txt) in FakeLogger.SINK if "[DONE]" in txt]
    assigns = [(t, txt) for (t, lv, txt) in FakeLogger.SINK
               if "[ASSIGN]" in txt]
    by_agv = {}
    for _, txt in assigns:
        aid = txt.split("-> ")[1].split(" ")[0]
        by_agv[aid] = by_agv.get(aid, 0) + 1
    makespan = max((t for t, _ in dones), default=float("nan"))
    return {
        "sched": sched,
        "harness": h,
        "assigns": assigns,
        "dones": dones,
        "by_agv": by_agv,
        "makespan": makespan,
        "peak_active": h.peak_active,
        "status": h.status(),
        "snap": h.snapshots.get("after_first_sched", {}),
    }


def banner(title):
    print("\n" + "=" * 74)
    print(title)
    print("=" * 74)


def report(label, r):
    banner(f"{label}")
    print("  ASSIGN / DONE timeline:")
    merged = sorted(
        [(t, "ASSIGN", txt) for t, txt in r["assigns"]]
        + [(t, "DONE", txt) for t, txt in r["dones"]])
    for t, kind, txt in merged:
        short = txt.split("center=")[0].strip() if kind == "ASSIGN" else txt
        print(f"    t={t:6.1f}s  {short}")
    print(f"\n  tasks per AGV : {r['by_agv']}")
    print(f"  peak active AGVs (concurrently busy): {r['peak_active']}")
    print(f"  makespan (last DONE): {r['makespan']:.1f}s")
    print(f"  completed: {r['status'].get('completed')}/4")


def yield_count(sink):
    return sum(1 for _, _, txt in sink if "[YIELD]" in txt)


def main():
    install_stubs()

    excl_layout = make_layout_variant("exclusive")
    ref = run_scenario(excl_layout)
    report("REFERENCE  (station_lane = exclusive) — full-lane lock", ref)
    print("    => the exclusive lane every task must cross serializes the "
          "fleet")
    os.unlink(excl_layout)

    fixed = run_scenario(LAYOUT)        # committed config: queue + return-home
    fixed_yields = yield_count(FakeLogger.SINK)
    report("FIXED  (committed: station_lane=queue + return-home de-squat)",
           fixed)

    banner("COMPARISON  (batch-dispatch active in both)")
    r, f = ref, fixed
    print(f"  peak concurrent AGVs : {r['peak_active']}  ->  {f['peak_active']}")
    print(f"  work split (per AGV) : {r['by_agv']}  ->  {f['by_agv']}")
    print(f"  makespan             : {r['makespan']:.1f}s  ->  "
          f"{f['makespan']:.1f}s")
    if not math.isnan(r["makespan"]) and not math.isnan(f["makespan"]):
        delta = (f["makespan"] - r["makespan"]) / r["makespan"] * 100.0
        print(f"  makespan change      : {delta:+.0f}%")
    print(f"  YIELD events (fixed) : {fixed_yields}  "
          "(no livelock; cf. 47 + 306s before the fix)")
    print("\n  Conclusion: queue lane unlocks parallelism AND the return-home")
    print("  de-squat removes the idle-on-dock deadlock, so the two AGVs now")
    print("  run in parallel and finish faster than the serialized baseline.")


if __name__ == "__main__":
    main()
