#!/usr/bin/env python3
"""
AGV warehouse scheduler — Nav2 Dynamic Avoidance Model

完全依赖 Nav2 local_costmap 动态避障：
- 移除 yield/停车/requeue 机制
- scheduler 主动发布对方车体位置点云注入 local_costmap
- 两车相遇时 Nav2 MPPI 感知点云障碍物自动绕行
- scheduler 只负责任务分配、导航状态监控、卡死检测
"""

import json
import math
import struct
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Set, Tuple

import yaml
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# ---------------------------------------------------------------------------
# Enumerations
# ---------------------------------------------------------------------------

class State(Enum):
    IDLE          = "idle"
    TO_SHELF      = "to_shelf"
    PICKING       = "picking"
    TO_AISLE_EXIT = "to_aisle_exit"
    TO_STATION    = "to_station"
    DELIVERING    = "delivering"
    TO_CHARGE     = "to_charge"
    CHARGING      = "charging"
    WAITING       = "waiting"
    ERROR         = "error"


class VehicleClass(Enum):
    PATROL    = 1
    EMPTY     = 2
    LOADED    = 3
    EMERGENCY = 4


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class Task:
    tid: str
    shelf: str
    shelf_center_xy: Tuple[float, float]
    pick_xy: Tuple[float, float]
    pick_yaw: float
    aisle_exit_xy: Tuple[float, float]
    drop_xy: Tuple[float, float]
    priority: int = 1
    cargo_value: float = 1.0
    deadline: float = 0.0
    ts: float = field(default_factory=time.time)
    agv: str = ""
    requested_agv: str = ""
    status: str = "pending"
    retry_after: float = 0.0
    last_error: str = ""
    enqueue_time: float = field(default_factory=time.time)

    def __lt__(self, other: "Task") -> bool:
        return self.priority > other.priority


@dataclass
class AGVState:
    aid: str
    nav_action: str
    odom_topic: str
    cmd_vel_topic: str
    status_topic: str
    vehicle_class: VehicleClass = VehicleClass.EMPTY
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    state: State = State.IDLE
    battery: float = 100.0
    task: Optional[Task] = None
    vx: float = 0.0
    wz: float = 0.0
    last_odom_ts: float = 0.0
    current_goal_handle: object = None
    current_goal_xy: Optional[Tuple[float, float]] = None
    current_goal_yaw: float = 0.0
    nav_goal_sent_ts: float = 0.0
    nav_goal_accepted_ts: float = 0.0
    last_progress_xy: Tuple[float, float] = field(default_factory=lambda: (0.0, 0.0))
    last_progress_ts: float = 0.0
    spacetime_path: List[Tuple[str, float]] = field(default_factory=list)
    # nav_seq：每次 _send_nav 自增，用于让被取消/被替换的 goal 回调自我作废
    nav_seq: int = 0
    # pick_until_ts：进入 PICKING 后的取货截止时刻；0 表示未在取货
    pick_until_ts: float = 0.0


@dataclass(frozen=True)
class ShelfLocation:
    center_xy: Tuple[float, float]
    pick_xy: Tuple[float, float]
    pick_yaw: float = 0.0


# ---------------------------------------------------------------------------
# Layer-3: Spacetime reservation table
# ---------------------------------------------------------------------------

@dataclass
class SpacetimeSlot:
    agv_id: str
    task_id: str
    t_start: float
    t_end: float
    expires_at: float


class SpacetimeTable:
    def __init__(self, cell_size: float = 2.0, time_buffer: float = 4.0):
        self.cell_size   = cell_size
        self.time_buffer = time_buffer
        self._table: Dict[str, List[SpacetimeSlot]] = {}

    def _cell(self, x: float, y: float) -> str:
        gx = math.floor(x / self.cell_size)
        gy = math.floor(y / self.cell_size)
        return f"c:{gx}:{gy}"

    def _path_cells_eta(
            self,
            waypoints: List[Tuple[float, float]],
            speed: float = 0.5,
            t0: float = 0.0,
    ) -> List[Tuple[str, float]]:
        result: List[Tuple[str, float]] = []
        t = t0
        prev = waypoints[0]
        for wp in waypoints[1:]:
            dist = math.hypot(wp[0] - prev[0], wp[1] - prev[1])
            steps = max(1, int(math.ceil(dist / self.cell_size)))
            for s in range(steps + 1):
                ratio = s / steps
                x = prev[0] + (wp[0] - prev[0]) * ratio
                y = prev[1] + (wp[1] - prev[1]) * ratio
                eta = t + (dist * ratio / speed)
                result.append((self._cell(x, y), eta))
            t += dist / speed
            prev = wp
        seen: Set[str] = set()
        unique: List[Tuple[str, float]] = []
        for ck, eta in result:
            if ck not in seen:
                seen.add(ck)
                unique.append((ck, eta))
        return unique

    def conflicts(
            self,
            agv_id: str,
            waypoints: List[Tuple[float, float]],
            speed: float = 0.5,
    ) -> List[str]:
        t0 = time.time()
        path = self._path_cells_eta(waypoints, speed, t0)
        conflicting: List[str] = []
        for ck, eta in path:
            for slot in self._table.get(ck, []):
                if slot.agv_id == agv_id:
                    continue
                if slot.t_start - self.time_buffer <= eta <= slot.t_end + self.time_buffer:
                    if slot.agv_id not in conflicting:
                        conflicting.append(slot.agv_id)
        return conflicting

    def reserve(
            self,
            agv_id: str,
            task_id: str,
            waypoints: List[Tuple[float, float]],
            speed: float = 0.5,
            hold: float = 300.0,
    ) -> List[Tuple[str, float]]:
        self.release(agv_id)
        t0 = time.time()
        path = self._path_cells_eta(waypoints, speed, t0)
        exp = t0 + hold
        for ck, eta in path:
            slot = SpacetimeSlot(
                agv_id=agv_id,
                task_id=task_id,
                t_start=eta - self.time_buffer,
                t_end=eta + self.time_buffer,
                expires_at=exp,
            )
            self._table.setdefault(ck, []).append(slot)
        return path

    def release(self, agv_id: str):
        for ck in list(self._table):
            self._table[ck] = [s for s in self._table[ck] if s.agv_id != agv_id]
            if not self._table[ck]:
                del self._table[ck]

    def cleanup_stale(self):
        now = time.time()
        for ck in list(self._table):
            self._table[ck] = [s for s in self._table[ck] if s.expires_at > now]
            if not self._table[ck]:
                del self._table[ck]

    def snapshot(self) -> Dict:
        return {
            ck: [
                {"agv": s.agv_id, "t_start": round(s.t_start, 1),
                 "t_end": round(s.t_end, 1)}
                for s in slots
            ]
            for ck, slots in self._table.items()
        }


# ---------------------------------------------------------------------------
# Layer-2: Task urgency scorer
# ---------------------------------------------------------------------------

class UrgencyScorer:
    W_DEADLINE      = 0.35
    W_VALUE         = 0.25
    W_BATTERY       = 0.20
    W_WAIT          = 0.20
    MAX_WAIT_S      = 300.0
    DEADLINE_WINDOW = 600.0

    @classmethod
    def score(cls, task: Task, agv_battery: float) -> float:
        now = time.time()
        if task.deadline > 0:
            remaining = max(0.0, task.deadline - now)
            dl_score = 1.0 - min(remaining / cls.DEADLINE_WINDOW, 1.0)
        else:
            dl_score = 0.0
        val_score  = min(task.cargo_value / 100.0, 1.0)
        bat_score  = max(0.0, (50.0 - agv_battery) / 50.0)
        wait_score = min((now - task.enqueue_time) / cls.MAX_WAIT_S, 1.0)
        return (cls.W_DEADLINE * dl_score + cls.W_VALUE * val_score
                + cls.W_BATTERY * bat_score + cls.W_WAIT * wait_score)


# ---------------------------------------------------------------------------
# Main scheduler node
# ---------------------------------------------------------------------------

class AGVScheduler(Node):

    DEFAULT_SHELVES = {
        "A1": ShelfLocation((-9.0,  7.0), (-9.0,  5.0),  math.pi / 2),
        "A2": ShelfLocation((-5.0,  7.0), (-5.0,  5.0),  math.pi / 2),
        "A3": ShelfLocation((-1.0,  7.0), (-1.0,  5.0),  math.pi / 2),
        "A4": ShelfLocation(( 3.0,  7.0), ( 3.0,  5.0),  math.pi / 2),
        "B1": ShelfLocation((-9.0,  3.0), (-9.0,  1.0),  math.pi / 2),
        "B2": ShelfLocation((-5.0,  3.0), (-5.0,  1.0),  math.pi / 2),
        "B3": ShelfLocation((-1.0,  3.0), (-1.0,  1.0),  math.pi / 2),
        "B4": ShelfLocation(( 3.0,  3.0), ( 3.0,  1.0),  math.pi / 2),
        "C1": ShelfLocation((-9.0, -3.0), (-9.0, -1.0), -math.pi / 2),
        "C2": ShelfLocation((-5.0, -3.0), (-5.0, -1.0), -math.pi / 2),
        "C3": ShelfLocation((-1.0, -3.0), (-1.0, -1.0), -math.pi / 2),
        "C4": ShelfLocation(( 3.0, -3.0), ( 3.0, -1.0), -math.pi / 2),
        "D1": ShelfLocation((-9.0, -7.0), (-9.0, -5.0), -math.pi / 2),
        "D2": ShelfLocation((-5.0, -7.0), (-5.0, -5.0), -math.pi / 2),
        "D3": ShelfLocation((-1.0, -7.0), (-1.0, -5.0), -math.pi / 2),
        "D4": ShelfLocation(( 3.0, -7.0), ( 3.0, -5.0), -math.pi / 2),
    }
    DEFAULT_STATION      = (6.4,  0.0)
    DEFAULT_CHARGING     = (9.0, -8.0)
    DEFAULT_AISLE_EXIT_X = 5.5
    NAV_SPEED            = 0.5

    STALL_TIMEOUT  = 45.0
    STALL_MIN_DIST = 0.3

    # 到达货架后停车并保持该状态的时长（秒），覆盖 Nav2 速度残留并给 controller 收尾时间
    PICK_DURATION  = 1.5

    # 对方车体轮廓点云参数
    PEER_HALF_LEN  = 0.45   # 车长一半（米）
    PEER_HALF_WID  = 0.45   # 车宽一半（米）
    PEER_GRID_STEP = 3      # 每侧采样数（3→5×5=25点）

    def __init__(self):
        super().__init__("agv_scheduler")
        self._declare_params()

        self.shelves, self.station_xy, self.charging_xy = (
            self._load_warehouse_layout())

        self.route_cell_size       = float(self.get_parameter("route_cell_size").value)
        self.route_hold_timeout    = float(self.get_parameter("route_hold_timeout").value)
        self.auto_demo_enabled     = bool(self.get_parameter("auto_demo_enabled").value)
        self.deadlock_check_period = float(self.get_parameter("deadlock_check_period").value)

        agv_ids        = self._string_list_param("agv_ids", ["agv_01"])
        nav_actions    = self._expanded_param("nav_action_names", ["navigate_to_pose"], len(agv_ids))
        odom_topics    = self._expanded_param("odom_topics",      ["/agv/odom"],        len(agv_ids))
        cmd_vel_topics = self._expanded_param("cmd_vel_topics",   ["/agv/cmd_vel"],     len(agv_ids))
        status_topics  = self._expanded_param("status_topics",    ["/agv/agv_status"],  len(agv_ids))
        vclass_strs    = self._expanded_param("vehicle_classes",  ["EMPTY"],            len(agv_ids))

        self.stt = SpacetimeTable(
            cell_size=self.route_cell_size,
            time_buffer=self.get_parameter("spacetime_time_buffer").value,
        )

        self.agvs: Dict[str, AGVState] = {}
        self.nav_clients: Dict[str, ActionClient] = {}
        self.cmd_publishers: Dict[str, object] = {}

        for idx, aid in enumerate(agv_ids):
            try:
                vc = VehicleClass[vclass_strs[idx].upper()]
            except KeyError:
                vc = VehicleClass.EMPTY
            agv = AGVState(
                aid=aid,
                nav_action=nav_actions[idx],
                odom_topic=odom_topics[idx],
                cmd_vel_topic=cmd_vel_topics[idx],
                status_topic=status_topics[idx],
                vehicle_class=vc,
            )
            self.agvs[aid] = agv
            self.nav_clients[aid] = ActionClient(self, NavigateToPose, agv.nav_action)
            self.cmd_publishers[aid] = self.create_publisher(
                Twist, agv.cmd_vel_topic, 10)
            self.create_subscription(
                Odometry,
                agv.odom_topic,
                lambda msg, robot_id=aid: self._on_odom(robot_id, msg),
                10,
            )

        for topic in sorted({agv.status_topic for agv in self.agvs.values()}):
            self.create_subscription(String, topic, self._on_status, 10)

        self.queue:   List[Task] = []
        self.history: List[Task] = []
        self.lock = threading.Lock()
        self._task_cnt = 0
        self._demo_n   = 0

        self.create_subscription(String, "/agv/task_request", self._on_task, 10)
        self.pub_assign = self.create_publisher(String, "/agv/task_assigned",    10)
        self.pub_sched  = self.create_publisher(String, "/agv/scheduler_status", 10)

        # 为每台车创建"对方车体点云"发布器
        # 话题 /agv_01/peer_obstacles 供 agv_01 的 local_costmap 订阅
        self.peer_cloud_pubs: Dict[str, object] = {}
        qos_peer = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        for aid in self.agvs:
            self.peer_cloud_pubs[aid] = self.create_publisher(
                PointCloud2, f"/{aid}/peer_obstacles", qos_profile=qos_peer)
    

        self.create_timer(1.0,  self._sched_loop)
        self.create_timer(0.5,  self._pub_status)
        self.create_timer(0.2,  self._publish_peer_obstacles)  # 5Hz 点云注入
        self.create_timer(0.2,  self._pickup_check)            # 5Hz PICKING→TO_AISLE_EXIT
        self.create_timer(0.5,  self._proximity_monitor)
        self.create_timer(1.0,  self._nav_watchdog)
        self.create_timer(5.0,  self._stall_check)
        self.create_timer(15.0, self._auto_demo)

        self.get_logger().info("=" * 60)
        self.get_logger().info("  AGV Scheduler — Nav2 Dynamic Avoidance Mode")
        self.get_logger().info("=" * 60)
        self.get_logger().info(
            f"Shelves: {len(self.shelves)} | "
            f"station=({self.station_xy[0]:.1f},{self.station_xy[1]:.1f}) | "
            f"AGVs: {', '.join(self.agvs)}"
        )

    # -----------------------------------------------------------------------
    # Parameter helpers
    # -----------------------------------------------------------------------

    def _declare_params(self):
        self.declare_parameter("agv_ids",               ["agv_01"])
        self.declare_parameter("nav_action_names",      ["navigate_to_pose"])
        self.declare_parameter("odom_topics",           ["/agv/odom"])
        self.declare_parameter("cmd_vel_topics",        ["/agv/cmd_vel"])
        self.declare_parameter("status_topics",         ["/agv/agv_status"])
        self.declare_parameter("vehicle_classes",       ["EMPTY"])
        self.declare_parameter("route_cell_size",       2.0)
        self.declare_parameter("route_hold_timeout",    300.0)
        self.declare_parameter("spacetime_time_buffer", 4.0)
        self.declare_parameter("deadlock_check_period", 5.0)
        self.declare_parameter("auto_demo_enabled",     False)
        self.declare_parameter("shelf_layout_file",     "")

    def _load_warehouse_layout(
            self,
    ) -> Tuple[Dict[str, ShelfLocation], Tuple[float, float], Tuple[float, float]]:
        layout_file = str(self.get_parameter("shelf_layout_file").value or "")
        if not layout_file:
            self.get_logger().warn("No shelf_layout_file; using built-in layout")
            return dict(self.DEFAULT_SHELVES), self.DEFAULT_STATION, self.DEFAULT_CHARGING

        with open(layout_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        layout      = data.get("warehouse_layout", data)
        raw_shelves = layout.get("shelves", {})
        if not raw_shelves:
            raise ValueError(f"No shelves defined in {layout_file}")

        shelves: Dict[str, ShelfLocation] = {}
        for shelf_id, raw in raw_shelves.items():
            if not isinstance(raw, dict):
                raise ValueError(f"Shelf {shelf_id} must be a mapping")
            center = self._xy_from_config(raw.get("center"), f"{shelf_id}.center")
            pickup = self._xy_from_config(raw.get("pickup"), f"{shelf_id}.pickup")
            yaw    = self._yaw_from_config(raw, f"{shelf_id}.pickup_yaw")
            shelves[str(shelf_id)] = ShelfLocation(center, pickup, yaw)

        raw_station = layout.get("station", {})
        station  = self._xy_from_config(
            raw_station.get("dock", raw_station.get("center")), "station.dock")
        charging = self._xy_from_config(
            layout.get("charging", {}).get("center"), "charging.center")
        self.get_logger().info(f"Loaded shelf layout: {layout_file}")
        return shelves, station, charging

    def _xy_from_config(self, value, name: str) -> Tuple[float, float]:
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            raise ValueError(f"{name} must be a two-item [x, y] list")
        return (float(value[0]), float(value[1]))

    def _yaw_from_config(self, raw: dict, name: str) -> float:
        if "pickup_yaw"     in raw: return float(raw["pickup_yaw"])
        if "pickup_yaw_deg" in raw: return math.radians(float(raw["pickup_yaw_deg"]))
        return 0.0

    def _string_list_param(self, name: str, default: List[str]) -> List[str]:
        value = self.get_parameter(name).value
        if value is None:
            return default
        items = [i.strip() for i in (value.split(",") if isinstance(value, str) else value)]
        return [i for i in items if i] or default

    def _expanded_param(self, name: str, default: List[str], count: int) -> List[str]:
        values = self._string_list_param(name, default)
        if len(values) == 1 and count > 1:
            return [values[0]] * count
        if len(values) != count:
            raise ValueError(
                f"Parameter {name} must have 1 or {count} entries, got {len(values)}")
        return values

    # -----------------------------------------------------------------------
    # Odometry / status callbacks
    # -----------------------------------------------------------------------

    def _on_odom(self, aid: str, msg: Odometry):
        p = msg.pose.pose
        q = p.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        with self.lock:
            agv = self.agvs.get(aid)
            if not agv:
                return
            agv.x            = round(p.position.x, 3)
            agv.y            = round(p.position.y, 3)
            agv.yaw          = round(math.atan2(siny, cosy), 3)
            agv.vx           = round(msg.twist.twist.linear.x,  3)
            agv.wz           = round(msg.twist.twist.angular.z, 3)
            agv.last_odom_ts = time.time()
            moved = math.hypot(agv.x - agv.last_progress_xy[0],
                               agv.y - agv.last_progress_xy[1])
            if moved > self.STALL_MIN_DIST:
                agv.last_progress_xy = (agv.x, agv.y)
                agv.last_progress_ts = time.time()

    def _on_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            aid  = data.get("agv_id") or data.get("agv")
            if aid not in self.agvs:
                return
            with self.lock:
                agv = self.agvs[aid]
                agv.state   = State(data.get("state", agv.state.value))
                agv.battery = float(data.get("battery", agv.battery))
        except Exception as exc:
            self.get_logger().warn(f"AGV status parse error: {exc}")

    # -----------------------------------------------------------------------
    # Task ingestion
    # -----------------------------------------------------------------------

    def _on_task(self, msg: String):
        try:
            data  = json.loads(msg.data)
            shelf = data.get("shelf", "A1")
            if shelf not in self.shelves:
                self.get_logger().warn(f"Unknown shelf: {shelf}")
                return
            sl = self.shelves[shelf]
            self._task_cnt += 1
            task = Task(
                tid=data.get("tid", f"T{self._task_cnt:04d}"),
                shelf=shelf,
                shelf_center_xy=sl.center_xy,
                pick_xy=sl.pick_xy,
                pick_yaw=sl.pick_yaw,
                aisle_exit_xy=(self.DEFAULT_AISLE_EXIT_X, sl.pick_xy[1]),
                drop_xy=self.station_xy,
                priority=int(data.get("priority", 1)),
                cargo_value=float(data.get("cargo_value", 1.0)),
                deadline=float(data.get("deadline", 0.0)),
                requested_agv=data.get("agv_id", data.get("agv", "")),
            )
            with self.lock:
                self.queue.append(task)
                self._sort_queue_locked()
            self.get_logger().info(
                f"[QUEUE] {task.tid} shelf={shelf} priority={task.priority} "
                f"pending={len(self.queue)}"
            )
        except Exception as exc:
            self.get_logger().error(f"Task parse failed: {exc}")

    def _sort_queue_locked(self):
        def key(task: Task) -> float:
            best_bat = min((a.battery for a in self.agvs.values()), default=100.0)
            urgency  = UrgencyScorer.score(task, best_bat)
            return -(task.priority + urgency * 3)
        self.queue.sort(key=key)

    # -----------------------------------------------------------------------
    # Scheduling loop
    # -----------------------------------------------------------------------

    def _sched_loop(self):
        self.stt.cleanup_stale()
        assignment = None

        with self.lock:
            self._sort_queue_locked()
            idle = [
                agv for agv in self.agvs.values()
                if agv.state == State.IDLE and agv.battery > 15
            ]
            if not self.queue or not idle:
                return

            for task_idx, task in enumerate(list(self.queue)):
                if task.retry_after > time.time():
                    continue
                candidates = [
                    agv for agv in idle
                    if not task.requested_agv or agv.aid == task.requested_agv
                ]
                candidates.sort(
                    key=lambda a: math.hypot(
                        a.x - task.pick_xy[0], a.y - task.pick_xy[1])
                )
                for agv in candidates:
                    waypoints = [
                        (agv.x, agv.y),
                        task.pick_xy,
                        task.aisle_exit_xy,
                        task.drop_xy,
                    ]
                    blockers = self.stt.conflicts(agv.aid, waypoints, self.NAV_SPEED)
                    if blockers:
                        task.status = f"waiting:{','.join(blockers)}"
                        continue
                    path = self.stt.reserve(
                        agv.aid, task.tid, waypoints,
                        self.NAV_SPEED, self.route_hold_timeout,
                    )
                    agv.spacetime_path   = path
                    agv.last_progress_xy = (agv.x, agv.y)
                    agv.last_progress_ts = time.time()
                    self.queue.pop(task_idx)
                    task.agv    = agv.aid
                    task.status = "running"
                    agv.state   = State.TO_SHELF
                    agv.task    = task
                    assignment  = (agv.aid, task)
                    break
                if assignment:
                    break

        if not assignment:
            return

        aid, task = assignment
        agv = self.agvs[aid]
        self.get_logger().info(
            f"[ASSIGN] {task.tid} → {aid} shelf={task.shelf} "
            f"pick=({task.pick_xy[0]:.1f},{task.pick_xy[1]:.1f})"
        )
        self._pub_assign(agv, task)
        if not self._send_nav(task.pick_xy, task.pick_yaw, task, agv):
            self._return_task_to_queue(agv, task, "Nav2 server not ready")
            return
        with self.lock:
            self.history.append(task)

    # -----------------------------------------------------------------------
    # Navigation
    # -----------------------------------------------------------------------

    def _send_nav(
            self,
            xy: Tuple[float, float],
            yaw: float,
            task: Task,
            agv: AGVState,
    ) -> bool:
        client = self.nav_clients[agv.aid]
        if not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                f"[Nav2] {agv.aid} action not ready: {agv.nav_action}")
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id    = "map"
        goal.pose.header.stamp       = self.get_clock().now().to_msg()
        goal.pose.pose.position.x    = float(xy[0])
        goal.pose.pose.position.y    = float(xy[1])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        with self.lock:
            cur = self.agvs.get(agv.aid)
            if not cur:
                return False
            cur.nav_seq             += 1
            seq                      = cur.nav_seq
            cur.current_goal_handle  = None
            cur.current_goal_xy      = xy
            cur.current_goal_yaw     = yaw
            cur.nav_goal_sent_ts     = time.time()
            cur.nav_goal_accepted_ts = 0.0

        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._nav_accepted(f, task, agv, seq))
        self.get_logger().info(
            f"[Nav2] {agv.aid} goal=({xy[0]:.1f},{xy[1]:.1f},yaw={yaw:.2f}) "
            f"seq={seq} via {agv.nav_action}"
        )
        return True

    def _nav_accepted(self, future, task: Task, agv: AGVState, seq: int):
        def stale() -> bool:
            cur = self.agvs.get(agv.aid)
            return (
                cur is None
                or cur.nav_seq != seq
                or not cur.task
                or cur.task.tid != task.tid
            )

        try:
            goal_handle = future.result()
        except Exception as exc:
            with self.lock:
                if stale():
                    return
            self._return_task_to_queue(agv, task, f"goal response failed: {exc}")
            return
        if goal_handle is None:
            with self.lock:
                if stale():
                    return
            self._return_task_to_queue(agv, task, "empty goal response")
            return
        if not goal_handle.accepted:
            with self.lock:
                if stale():
                    return
            self._return_task_to_queue(agv, task, "goal rejected")
            return

        with self.lock:
            if stale():
                goal_handle.cancel_goal_async()
                return
            cur = self.agvs[agv.aid]
            cur.current_goal_handle  = goal_handle
            cur.nav_goal_accepted_ts = time.time()

        goal_handle.get_result_async().add_done_callback(
            lambda f: self._nav_done(f, task, agv, seq)
        )

    def _nav_done(self, future, task: Task, agv: AGVState, seq: int):
        result = future.result()
        status = getattr(result, "status", None)

        with self.lock:
            cur = self.agvs.get(agv.aid)
            if not cur or cur.nav_seq != seq:
                # 已被新一轮 _send_nav 或 stall 重发顶替，丢弃此回调
                return
            if not cur.task or cur.task.tid != task.tid:
                return
            cur.current_goal_handle = None

        if status != GoalStatus.STATUS_SUCCEEDED:
            self._return_task_to_queue(agv, task, f"navigation status {status}")
            return

        next_xy: Optional[Tuple[float, float]] = None
        next_yaw   = 0.0
        next_label = ""
        with self.lock:
            cur = self.agvs[agv.aid]
            if cur.state == State.TO_SHELF:
                # 到达货架：进入 PICKING，先停车并保持，避免立即跨腿产生速度残留+抖动
                cur.state                = State.PICKING
                cur.pick_until_ts        = time.time() + self.PICK_DURATION
                cur.nav_goal_sent_ts     = 0.0   # 阻止 _nav_watchdog 误判
                cur.nav_goal_accepted_ts = 0.0
                cur.last_progress_ts     = time.time()
            elif cur.state == State.TO_AISLE_EXIT:
                cur.state  = State.TO_STATION
                next_xy    = self.station_xy
                next_yaw   = 0.0
                next_label = "station"
            elif cur.state == State.TO_STATION:
                cur.state                = State.IDLE
                cur.task                 = None
                cur.current_goal_xy      = None
                cur.nav_goal_sent_ts     = 0.0
                cur.nav_goal_accepted_ts = 0.0
                cur.pick_until_ts        = 0.0
                task.status = "done"
                self.stt.release(agv.aid)
                self.get_logger().info(f"[DONE] {task.tid} completed by {agv.aid}")
                return
            else:
                return

        if next_xy is None:
            # PICKING 分支：先停一脚速度，剩下的等 _pickup_check 接力
            self._publish_stop(agv.aid)
            self.get_logger().info(
                f"[PICKING] {agv.aid} 到达货架 {task.shelf}，"
                f"取货停留 {self.PICK_DURATION:.1f}s"
            )
            return

        self.get_logger().info(
            f"[ARRIVE] {agv.aid} reached step → heading to {next_label}"
        )
        if not self._send_nav(next_xy, next_yaw, task, agv):
            self._return_task_to_queue(
                agv, task, f"{next_label} navigation unavailable")

    def _pickup_check(self):
        """轮询：PICKING 倒计时到期后发起前往巷道出口的下一段导航。"""
        ready: List[Tuple[AGVState, Task]] = []
        now = time.time()
        with self.lock:
            for agv in self.agvs.values():
                if (agv.state == State.PICKING
                        and agv.task
                        and 0.0 < agv.pick_until_ts <= now):
                    agv.pick_until_ts    = 0.0
                    agv.state            = State.TO_AISLE_EXIT
                    agv.last_progress_ts = now
                    ready.append((agv, agv.task))

        for agv, task in ready:
            self.get_logger().info(
                f"[PICK_DONE] {agv.aid} 取货完成，前往巷道出口 "
                f"({task.aisle_exit_xy[0]:.1f},{task.aisle_exit_xy[1]:.1f})"
            )
            if not self._send_nav(task.aisle_exit_xy, 0.0, task, agv):
                self._return_task_to_queue(
                    agv, task, "aisle exit navigation unavailable")

    def _return_task_to_queue(self, agv: AGVState, task: Task, reason: str):
        with self.lock:
            cur = self.agvs.get(agv.aid)
            if cur:
                cur.nav_seq             += 1   # 让仍在飞的回调自我作废
                cur.state                = State.IDLE
                cur.task                 = None
                cur.current_goal_handle  = None
                cur.current_goal_xy      = None
                cur.nav_goal_sent_ts     = 0.0
                cur.nav_goal_accepted_ts = 0.0
                cur.pick_until_ts        = 0.0
                self.stt.release(cur.aid)
            task.status      = "pending"
            task.agv         = ""
            task.last_error  = reason
            task.retry_after = time.time() + 5.0
            if not any(t.tid == task.tid for t in self.queue):
                self.queue.append(task)
                self._sort_queue_locked()
        self._publish_stop(agv.aid)
        self.get_logger().warn(f"[REQUEUE] {task.tid}: {reason}")

    # -----------------------------------------------------------------------
    # 核心：把对方车体位置注入为点云障碍物
    # -----------------------------------------------------------------------

    def _publish_peer_obstacles(self):
        """
        以 5Hz 频率把每台车的车体轮廓点云发布到对方的
        /agv_XX/peer_obstacles 话题，供对方 local_costmap
        的 obstacle_layer 标记为动态障碍物。
        MPPI 控制器看到代价后自动规划绕行路径。
        """
        now_stamp = self.get_clock().now().to_msg()
        agv_list  = list(self.agvs.values())

        hl   = self.PEER_HALF_LEN
        hw   = self.PEER_HALF_WID
        n    = self.PEER_GRID_STEP
        # 生成本地坐标格网（车体轮廓采样点）
        local_offsets = [
            (dx, dy)
            for dx in [hl * (2 * i / (n - 1) - 1) for i in range(n)]
            for dy in [hw * (2 * j / (n - 1) - 1) for j in range(n)]
        ]

        for target in agv_list:
            if target.last_odom_ts == 0.0:
                continue

            points: List[Tuple[float, float, float]] = []
            for other in agv_list:
                if other.aid == target.aid:
                    continue
                if other.last_odom_ts == 0.0:
                    continue
                cos_y = math.cos(other.yaw)
                sin_y = math.sin(other.yaw)
                for dx, dy in local_offsets:
                    wx = other.x + dx * cos_y - dy * sin_y
                    wy = other.y + dx * sin_y + dy * cos_y
                    points.append((wx, wy, 0.1))

            if not points:
                continue

            cloud = self._make_pointcloud2(now_stamp, "map", points)
            self.peer_cloud_pubs[target.aid].publish(cloud)

    @staticmethod
    def _make_pointcloud2(stamp, frame_id: str,
                          points: List[Tuple[float, float, float]]) -> PointCloud2:
        fields = [
            PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        data = bytearray()
        for px, py, pz in points:
            data += struct.pack("fff", float(px), float(py), float(pz))
        header          = Header()
        header.stamp    = stamp
        header.frame_id = frame_id
        cloud               = PointCloud2()
        cloud.header        = header
        cloud.height        = 1
        cloud.width         = len(points)
        cloud.fields        = fields
        cloud.is_bigendian  = False
        cloud.point_step    = 12
        cloud.row_step      = 12 * len(points)
        cloud.data          = bytes(data)
        cloud.is_dense      = True
        return cloud

    # -----------------------------------------------------------------------
    # 接近监控（仅日志）
    # -----------------------------------------------------------------------

    def _proximity_monitor(self):
        with self.lock:
            agvs = list(self.agvs.values())
        for i, left in enumerate(agvs):
            for right in agvs[i + 1:]:
                if left.last_odom_ts == 0.0 or right.last_odom_ts == 0.0:
                    continue
                dist = math.hypot(left.x - right.x, left.y - right.y)
                if dist < 2.0:
                    self.get_logger().info(
                        f"[PROXIMITY] {left.aid} ↔ {right.aid} "
                        f"dist={dist:.2f}m — 点云障碍已注入代价地图"
                    )

    # -----------------------------------------------------------------------
    # 卡死检测
    # -----------------------------------------------------------------------

    def _stall_check(self):
        now     = time.time()
        stalled = []
        with self.lock:
            for agv in self.agvs.values():
                if agv.state == State.IDLE or not agv.task:
                    continue
                if agv.state == State.PICKING:   # 取货停留是预期行为，不算卡死
                    continue
                if agv.last_progress_ts == 0.0:
                    continue
                if now - agv.last_progress_ts > self.STALL_TIMEOUT:
                    stalled.append(agv)

        for agv in stalled:
            with self.lock:
                cur = self.agvs.get(agv.aid)
                if not cur or not cur.task or not cur.current_goal_xy:
                    continue
                # 让旧 goal 的回调失效，避免 cancel+_send_nav 与 _nav_done 竞态把任务踢回队列
                cur.nav_seq             += 1
                old_handle               = cur.current_goal_handle
                cur.current_goal_handle  = None
                cur.last_progress_ts     = time.time()
                goal_xy                  = cur.current_goal_xy
                goal_yaw                 = cur.current_goal_yaw
                task                     = cur.task
                aid                      = cur.aid

            self.get_logger().warn(
                f"[STALL] {aid} 卡死超过 {self.STALL_TIMEOUT}s，"
                f"重新规划至 ({goal_xy[0]:.1f},{goal_xy[1]:.1f})"
            )

            def resend(_unused, agv=agv, task=task, goal_xy=goal_xy, goal_yaw=goal_yaw):
                if not self._send_nav(goal_xy, goal_yaw, task, agv):
                    self._return_task_to_queue(
                        agv, task, "stall recovery navigation unavailable")

            if old_handle is not None:
                try:
                    old_handle.cancel_goal_async().add_done_callback(resend)
                except Exception as exc:
                    self.get_logger().warn(
                        f"[STALL] {aid} cancel 失败，直接重发: {exc}")
                    resend(None)
            else:
                resend(None)

    # -----------------------------------------------------------------------
    # Nav watchdog
    # -----------------------------------------------------------------------

    def _nav_watchdog(self):
        victims: List[Tuple[AGVState, Task, str]] = []
        now = time.time()
        with self.lock:
            for agv in self.agvs.values():
                if agv.state == State.IDLE or not agv.task:
                    continue
                if agv.state == State.PICKING:   # 取货停留期间无在飞 goal，跳过
                    continue
                if not agv.nav_goal_sent_ts:
                    continue
                pending = (
                    agv.current_goal_handle is None
                    and agv.nav_goal_accepted_ts == 0.0
                    and now - agv.nav_goal_sent_ts > 5.0
                )
                if pending:
                    victims.append((agv, agv.task,
                                    "Nav2 goal not accepted within 5 s"))
        for agv, task, reason in victims:
            self._return_task_to_queue(agv, task, reason)

    # -----------------------------------------------------------------------
    # Publishing helpers
    # -----------------------------------------------------------------------

    def _publish_stop(self, aid: str):
        pub = self.cmd_publishers.get(aid)
        if pub:
            pub.publish(Twist())

    def _pub_assign(self, agv: AGVState, task: Task):
        msg = String()
        msg.data = json.dumps({
            "agv":           agv.aid,
            "tid":           task.tid,
            "shelf":         task.shelf,
            "shelf_center":  list(task.shelf_center_xy),
            "pick":          list(task.pick_xy),
            "pick_yaw":      task.pick_yaw,
            "aisle_exit":    list(task.aisle_exit_xy),
            "drop":          list(task.drop_xy),
            "priority":      task.priority,
            "cargo_value":   task.cargo_value,
            "deadline":      task.deadline,
            "vehicle_class": agv.vehicle_class.name,
            "nav_action":    agv.nav_action,
        }, ensure_ascii=False)
        self.pub_assign.publish(msg)

    def _pub_status(self):
        with self.lock:
            done = sum(1 for t in self.history if t.status == "done")
            payload = {
                "pending":   len(self.queue),
                "completed": done,
                "queue": [
                    {
                        "tid":           t.tid,
                        "shelf":         t.shelf,
                        "status":        t.status,
                        "requested_agv": t.requested_agv,
                        "retry_in":      max(0.0, round(
                            t.retry_after - time.time(), 1)),
                        "last_error":    t.last_error,
                        "cargo_value":   t.cargo_value,
                        "deadline":      t.deadline,
                    }
                    for t in self.queue
                ],
                "spacetime_reservations": len(self.stt._table),
                "fleet": {
                    aid: {
                        "state":         agv.state.value,
                        "vehicle_class": agv.vehicle_class.name,
                        "pos":           [round(agv.x, 2), round(agv.y, 2)],
                        "battery":       round(agv.battery, 1),
                        "vx":            agv.vx,
                        "wz":            agv.wz,
                        "task":          agv.task.tid if agv.task else None,
                        "nav_action":    agv.nav_action,
                        "goal": (
                            [round(agv.current_goal_xy[0], 2),
                             round(agv.current_goal_xy[1], 2)]
                            if agv.current_goal_xy else None
                        ),
                        "goal_active": agv.current_goal_handle is not None,
                    }
                    for aid, agv in self.agvs.items()
                },
            }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_sched.publish(msg)

    # -----------------------------------------------------------------------
    # Auto demo
    # -----------------------------------------------------------------------

    def _auto_demo(self):
        if not self.auto_demo_enabled or self._demo_n >= 8:
            return
        import random
        shelf    = random.choice(list(self.shelves.keys()))
        priority = random.randint(1, 5)
        value    = round(random.uniform(1.0, 100.0), 1)
        msg = String()
        msg.data = json.dumps({
            "tid":         f"AUTO_{self._demo_n + 1:03d}",
            "shelf":       shelf,
            "priority":    priority,
            "cargo_value": value,
        }, ensure_ascii=False)
        self._on_task(msg)
        self._demo_n += 1


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = AGVScheduler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Scheduler shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
