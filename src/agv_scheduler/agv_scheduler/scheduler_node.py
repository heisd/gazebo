#!/usr/bin/env python3
"""
AGV warehouse scheduler with staged traffic reservations.

The scheduler keeps the original single-AGV compatibility, but upgrades the
fleet logic in three ways:
- prefer map-frame poses from TF for shared traffic decisions;
- reserve traffic resources per execution stage instead of locking the whole
  task up front;
- send yielding AGVs to configured safe wait points instead of stopping in the
  middle of a corridor.
"""

import json
import math
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Set, Tuple

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class State(Enum):
    IDLE = "idle"
    TO_SHELF = "to_shelf"
    PICKING = "picking"
    TO_AISLE_EXIT = "to_aisle_exit"
    TO_STATION = "to_station"
    DELIVERING = "delivering"
    TO_CHARGE = "to_charge"
    CHARGING = "charging"
    WAITING = "waiting"
    ERROR = "error"


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
    ts: float = field(default_factory=time.time)
    agv: str = ""
    requested_agv: str = ""
    status: str = "pending"
    retry_after: float = 0.0
    last_error: str = ""

    def __lt__(self, other):
        return self.priority > other.priority


@dataclass
class WaitPoint:
    name: str
    xy: Tuple[float, float]
    yaw: float = 0.0
    zone_id: str = ""


@dataclass
class TrafficZone:
    zid: str
    zone_type: str
    polygon: Tuple[Tuple[float, float], ...]
    wait_points: Dict[str, WaitPoint] = field(default_factory=dict)


@dataclass
class AGVState:
    aid: str
    nav_action: str
    base_frame: str
    odom_topic: str
    cmd_vel_topic: str
    status_topic: str
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    odom_x: float = 0.0
    odom_y: float = 0.0
    odom_yaw: float = 0.0
    map_x: float = 0.0
    map_y: float = 0.0
    map_yaw: float = 0.0
    pose_source: str = "unknown"
    state: State = State.IDLE
    battery: float = 100.0
    task: Optional[Task] = None
    vx: float = 0.0
    wz: float = 0.0
    last_odom_ts: float = 0.0
    last_map_pose_ts: float = 0.0
    current_goal_handle: object = None
    current_goal_xy: Optional[Tuple[float, float]] = None
    current_goal_yaw: float = 0.0
    nav_goal_sent_ts: float = 0.0
    nav_goal_accepted_ts: float = 0.0
    nav_goal_seq: int = 0
    pause_until: float = 0.0
    resume_state: State = State.IDLE
    resume_goal_xy: Optional[Tuple[float, float]] = None
    resume_goal_yaw: float = 0.0
    wait_until: float = 0.0
    wait_point_xy: Optional[Tuple[float, float]] = None
    wait_point_yaw: float = 0.0
    wait_reason: str = ""
    wait_zone: str = ""
    yielding_to: str = ""
    yield_cooldown_until: float = 0.0
    reserved_stage: str = ""
    reserved_zones: Set[str] = field(default_factory=set)
    reservation_deadline: float = 0.0


@dataclass
class RouteReservation:
    agv_id: str
    task_id: str
    stage: str
    zones: Set[str]
    expires_at: float
    granted_at: float = field(default_factory=time.time)


@dataclass
class ConflictLock:
    key: str
    winner_agv: str
    loser_agv: str
    zones: Set[str]
    created_at: float = field(default_factory=time.time)


@dataclass(frozen=True)
class ShelfLocation:
    center_xy: Tuple[float, float]
    pick_xy: Tuple[float, float]
    pick_yaw: float = 0.0


class AGVScheduler(Node):

    DEFAULT_SHELVES = {
        "A1": ShelfLocation((-9.0, 7.0), (-9.0, 5.0), math.pi / 2),
        "A2": ShelfLocation((-5.0, 7.0), (-5.0, 5.0), math.pi / 2),
        "A3": ShelfLocation((-1.0, 7.0), (-1.0, 5.0), math.pi / 2),
        "A4": ShelfLocation((3.0, 7.0), (3.0, 5.0), math.pi / 2),
        "B1": ShelfLocation((-9.0, 3.0), (-9.0, 1.0), math.pi / 2),
        "B2": ShelfLocation((-5.0, 3.0), (-5.0, 1.0), math.pi / 2),
        "B3": ShelfLocation((-1.0, 3.0), (-1.0, 1.0), math.pi / 2),
        "B4": ShelfLocation((3.0, 3.0), (3.0, 1.0), math.pi / 2),
        "C1": ShelfLocation((-9.0, -3.0), (-9.0, -1.0), -math.pi / 2),
        "C2": ShelfLocation((-5.0, -3.0), (-5.0, -1.0), -math.pi / 2),
        "C3": ShelfLocation((-1.0, -3.0), (-1.0, -1.0), -math.pi / 2),
        "C4": ShelfLocation((3.0, -3.0), (3.0, -1.0), -math.pi / 2),
        "D1": ShelfLocation((-9.0, -7.0), (-9.0, -5.0), -math.pi / 2),
        "D2": ShelfLocation((-5.0, -7.0), (-5.0, -5.0), -math.pi / 2),
        "D3": ShelfLocation((-1.0, -7.0), (-1.0, -5.0), -math.pi / 2),
        "D4": ShelfLocation((3.0, -7.0), (3.0, -5.0), -math.pi / 2),
    }
    DEFAULT_STATION = (8.0, 0.0)
    DEFAULT_CHARGING = (9.0, -8.0)
    DEFAULT_AISLE_EXIT_X = 5.5

    def __init__(self):
        super().__init__("agv_scheduler")
        self._declare_params()

        (
            self.shelves,
            self.station_xy,
            self.charging_xy,
            self.traffic_zones,
            self.global_wait_points,
        ) = self._load_warehouse_layout()

        self.route_cell_size = float(
            self.get_parameter("route_cell_size").value)
        self.route_hold_timeout = float(
            self.get_parameter("route_hold_timeout").value)
        self.reservation_refresh_interval = float(
            self.get_parameter("reservation_refresh_interval").value)
        self.safety_stop_distance = float(
            self.get_parameter("safety_stop_distance").value)
        self.right_of_way_release_distance = float(
            self.get_parameter("right_of_way_release_distance").value)
        self.yield_hold_duration = float(
            self.get_parameter("yield_hold_duration").value)
        self.yield_cooldown_duration = float(
            self.get_parameter("yield_cooldown_duration").value)
        self.pickup_pause_duration = float(
            self.get_parameter("pickup_pause_duration").value)
        self.pose_stale_timeout = float(
            self.get_parameter("pose_stale_timeout").value)
        self.wait_point_tolerance = float(
            self.get_parameter("wait_point_tolerance").value)
        self.pose_source_mode = str(
            self.get_parameter("pose_source").value or "map_then_odom")
        self.map_frame = str(self.get_parameter("map_frame").value or "map")
        self.auto_demo_enabled = bool(
            self.get_parameter("auto_demo_enabled").value)
        self.battery_drain_moving = float(
            self.get_parameter("battery_drain_moving").value)
        self.battery_drain_idle = float(
            self.get_parameter("battery_drain_idle").value)
        self.battery_charge_rate = float(
            self.get_parameter("battery_charge_rate").value)
        self.battery_low_threshold = float(
            self.get_parameter("battery_low_threshold").value)
        self.battery_critical_threshold = float(
            self.get_parameter("battery_critical_threshold").value)
        self.battery_full_threshold = float(
            self.get_parameter("battery_full_threshold").value)
        self.battery_min_dispatch = float(
            self.get_parameter("battery_min_dispatch").value)
        self.path_sample_step = max(0.25, self.route_cell_size / 2.0)

        agv_ids = self._string_list_param("agv_ids", ["agv_01"])
        nav_actions = self._expanded_param(
            "nav_action_names", ["navigate_to_pose"], len(agv_ids))
        odom_topics = self._expanded_param(
            "odom_topics", ["/agv/odom"], len(agv_ids))
        cmd_vel_topics = self._expanded_param(
            "cmd_vel_topics", ["/agv/cmd_vel"], len(agv_ids))
        status_topics = self._expanded_param(
            "status_topics", ["/agv/agv_status"], len(agv_ids))
        base_frames = self._expanded_param(
            "base_frames",
            [f"{aid}_base_footprint" for aid in agv_ids],
            len(agv_ids),
        )

        self.agvs: Dict[str, AGVState] = {}
        self.nav_clients: Dict[str, ActionClient] = {}
        self.cmd_publishers: Dict[str, object] = {}
        for idx, aid in enumerate(agv_ids):
            agv = AGVState(
                aid=aid,
                nav_action=nav_actions[idx],
                base_frame=base_frames[idx],
                odom_topic=odom_topics[idx],
                cmd_vel_topic=cmd_vel_topics[idx],
                status_topic=status_topics[idx],
            )
            self.agvs[aid] = agv
            self.nav_clients[aid] = ActionClient(
                self, NavigateToPose, agv.nav_action)
            self.cmd_publishers[aid] = self.create_publisher(
                Twist, agv.cmd_vel_topic, 10)
            self.create_subscription(
                Odometry,
                agv.odom_topic,
                lambda msg, robot_id=aid: self._on_odom(robot_id, msg),
                10,
            )

        self.tf_buffer: Optional[Buffer] = None
        self.tf_listener: Optional[TransformListener] = None
        if self.pose_source_mode != "odom":
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(
                self.tf_buffer, self, spin_thread=True)
            self.create_timer(0.2, self._refresh_map_poses)

        for topic in sorted({agv.status_topic for agv in self.agvs.values()}):
            self.create_subscription(String, topic, self._on_status, 10)

        self.queue: List[Task] = []
        self.history: List[Task] = []
        self.route_reservations: Dict[str, RouteReservation] = {}
        self.conflict_locks: Dict[str, ConflictLock] = {}
        self.lock = threading.Lock()
        self._task_cnt = 0
        self._demo_n = 0

        self.create_subscription(
            String, "/agv/task_request", self._on_task, 10)
        self.pub_assign = self.create_publisher(
            String, "/agv/task_assigned", 10)
        self.pub_sched = self.create_publisher(
            String, "/agv/scheduler_status", 10)

        self.create_timer(1.0, self._sched_loop)
        self.create_timer(0.5, self._pub_status)
        self.create_timer(0.2, self._safety_loop)
        self.create_timer(0.2, self._pause_loop)
        self.create_timer(0.2, self._right_of_way_loop)
        self.create_timer(1.0, self._nav_watchdog)
        if self.reservation_refresh_interval > 0.0:
            self.create_timer(
                self.reservation_refresh_interval,
                self._reservation_refresh_loop,
            )
        self.create_timer(15.0, self._auto_demo)
        self.create_timer(1.0, self._battery_loop)

        self.get_logger().info("=" * 50)
        self.get_logger().info(
            " AGV warehouse scheduler with staged traffic control started")
        self.get_logger().info("=" * 50)
        self.get_logger().info(
            f"Shelves: {len(self.shelves)} | "
            f"station=({self.station_xy[0]:.1f},{self.station_xy[1]:.1f}) | "
            f"AGVs: {', '.join(self.agvs)} | pose_source={self.pose_source_mode}")

    def _declare_params(self):
        self.declare_parameter("agv_ids", ["agv_01"])
        self.declare_parameter("nav_action_names", ["navigate_to_pose"])
        self.declare_parameter("base_frames", ["base_footprint"])
        self.declare_parameter("odom_topics", ["/agv/odom"])
        self.declare_parameter("cmd_vel_topics", ["/agv/cmd_vel"])
        self.declare_parameter("status_topics", ["/agv/agv_status"])
        self.declare_parameter("pose_source", "map_then_odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("route_cell_size", 2.0)
        self.declare_parameter("route_hold_timeout", 180.0)
        self.declare_parameter("reservation_refresh_interval", 2.0)
        self.declare_parameter("pose_stale_timeout", 8.0)
        self.declare_parameter("wait_point_tolerance", 0.35)
        self.declare_parameter("safety_stop_distance", 1.0)
        self.declare_parameter("right_of_way_release_distance", 1.6)
        self.declare_parameter("yield_hold_duration", 2.0)
        self.declare_parameter("yield_cooldown_duration", 3.0)
        self.declare_parameter("pickup_pause_duration", 4.0)
        self.declare_parameter("auto_demo_enabled", True)
        self.declare_parameter("battery_drain_moving", 0.1)
        self.declare_parameter("battery_drain_idle", 0.01)
        self.declare_parameter("battery_charge_rate", 0.5)
        self.declare_parameter("battery_low_threshold", 20.0)
        self.declare_parameter("battery_critical_threshold", 10.0)
        self.declare_parameter("battery_full_threshold", 95.0)
        self.declare_parameter("battery_min_dispatch", 15.0)
        self.declare_parameter("shelf_layout_file", "")

    def _load_warehouse_layout(
            self) -> Tuple[
                Dict[str, ShelfLocation],
                Tuple[float, float],
                Tuple[float, float],
                Dict[str, TrafficZone],
                Dict[str, WaitPoint],
            ]:
        layout_file = str(self.get_parameter("shelf_layout_file").value or "")
        if not layout_file:
            self.get_logger().warn(
                "No shelf_layout_file configured; using built-in layout")
            return (
                dict(self.DEFAULT_SHELVES),
                self.DEFAULT_STATION,
                self.DEFAULT_CHARGING,
                {},
                {},
            )

        with open(layout_file, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}

        layout = data.get("warehouse_layout", data)
        raw_shelves = layout.get("shelves", {})
        if not raw_shelves:
            raise ValueError(f"No shelves defined in {layout_file}")

        shelves: Dict[str, ShelfLocation] = {}
        for shelf_id, raw in raw_shelves.items():
            if not isinstance(raw, dict):
                raise ValueError(f"Shelf {shelf_id} must be a mapping")
            center = self._xy_from_config(raw.get("center"),
                                          f"{shelf_id}.center")
            pickup = self._xy_from_config(raw.get("pickup"),
                                          f"{shelf_id}.pickup")
            pickup_yaw = self._yaw_from_config(raw, f"{shelf_id}.pickup_yaw")
            shelves[str(shelf_id)] = ShelfLocation(
                center, pickup, pickup_yaw)

        raw_station = layout.get("station", {})
        station = self._xy_from_config(
            raw_station.get("dock", raw_station.get("center")),
            "station.dock")
        charging = self._xy_from_config(
            layout.get("charging", {}).get("center"), "charging.center")

        traffic_zones: Dict[str, TrafficZone] = {}
        for zone_id, raw_zone in (layout.get("traffic_zones", {}) or {}).items():
            if not isinstance(raw_zone, dict):
                raise ValueError(f"Traffic zone {zone_id} must be a mapping")
            polygon = self._polygon_from_config(
                raw_zone.get("polygon"), f"traffic_zones.{zone_id}.polygon")
            zone_type = str(raw_zone.get("type", "exclusive")).strip() or "exclusive"
            wait_points: Dict[str, WaitPoint] = {}
            for agv_id, raw_wait in (raw_zone.get("wait_points", {}) or {}).items():
                wait_xy, wait_yaw = self._pose_from_config(
                    raw_wait,
                    f"traffic_zones.{zone_id}.wait_points.{agv_id}",
                )
                wait_points[str(agv_id)] = WaitPoint(
                    name=f"{zone_id}:{agv_id}",
                    xy=wait_xy,
                    yaw=wait_yaw,
                    zone_id=str(zone_id),
                )
            traffic_zones[str(zone_id)] = TrafficZone(
                zid=str(zone_id),
                zone_type=zone_type,
                polygon=polygon,
                wait_points=wait_points,
            )

        global_wait_points: Dict[str, WaitPoint] = {}
        for name, raw_wait in (layout.get("wait_points", {}) or {}).items():
            wait_xy, wait_yaw = self._pose_from_config(
                raw_wait, f"wait_points.{name}")
            global_wait_points[str(name)] = WaitPoint(
                name=str(name),
                xy=wait_xy,
                yaw=wait_yaw,
            )

        self.get_logger().info(f"Loaded shelf layout: {layout_file}")
        return shelves, station, charging, traffic_zones, global_wait_points

    def _xy_from_config(self, value, name: str) -> Tuple[float, float]:
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            raise ValueError(f"{name} must be a two-item [x, y] list")
        return (float(value[0]), float(value[1]))

    def _pose_from_config(
            self,
            value,
            name: str) -> Tuple[Tuple[float, float], float]:
        if not isinstance(value, (list, tuple)) or len(value) not in (2, 3):
            raise ValueError(f"{name} must be [x, y] or [x, y, yaw]")
        yaw = float(value[2]) if len(value) == 3 else 0.0
        return (float(value[0]), float(value[1])), yaw

    def _polygon_from_config(
            self,
            value,
            name: str) -> Tuple[Tuple[float, float], ...]:
        if not isinstance(value, list) or len(value) < 3:
            raise ValueError(f"{name} must contain at least three points")
        return tuple(self._xy_from_config(point, name) for point in value)

    def _yaw_from_config(self, raw: dict, name: str) -> float:
        if "pickup_yaw" in raw:
            return float(raw["pickup_yaw"])
        if "pickup_yaw_deg" in raw:
            return math.radians(float(raw["pickup_yaw_deg"]))
        return 0.0

    def _string_list_param(self, name: str, default: List[str]) -> List[str]:
        value = self.get_parameter(name).value
        if value is None:
            return default
        if isinstance(value, str):
            items = [item.strip() for item in value.split(",")]
        else:
            items = [str(item).strip() for item in value]
        return [item for item in items if item] or default

    def _expanded_param(
            self,
            name: str,
            default: List[str],
            count: int) -> List[str]:
        values = self._string_list_param(name, default)
        if len(values) == 1 and count > 1:
            return [values[0] for _ in range(count)]
        if len(values) != count:
            raise ValueError(
                f"Parameter {name} must have 1 or {count} entries, "
                f"got {len(values)}")
        return values

    def _pose_age_locked(self, agv: AGVState, now: Optional[float] = None) -> float:
        now = now or time.time()
        latest = agv.last_map_pose_ts if agv.pose_source == "map" else 0.0
        latest = max(latest, agv.last_odom_ts)
        if latest <= 0.0:
            return float("inf")
        return now - latest

    def _quat_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny, cosy)

    def _refresh_map_poses(self):
        if not self.tf_buffer:
            return

        for aid, agv in self.agvs.items():
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    agv.base_frame,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
            except TransformException:
                continue

            translation = transform.transform.translation
            rotation = transform.transform.rotation
            yaw = self._quat_to_yaw(
                rotation.x, rotation.y, rotation.z, rotation.w)
            now = time.time()
            with self.lock:
                current = self.agvs.get(aid)
                if not current:
                    continue
                current.map_x = round(translation.x, 3)
                current.map_y = round(translation.y, 3)
                current.map_yaw = round(yaw, 3)
                current.last_map_pose_ts = now
                current.x = current.map_x
                current.y = current.map_y
                current.yaw = current.map_yaw
                current.pose_source = "map"

    def _on_odom(self, aid: str, msg: Odometry):
        p = msg.pose.pose
        yaw = self._quat_to_yaw(
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w,
        )
        now = time.time()
        with self.lock:
            agv = self.agvs.get(aid)
            if not agv:
                return
            agv.odom_x = round(p.position.x, 3)
            agv.odom_y = round(p.position.y, 3)
            agv.odom_yaw = round(yaw, 3)
            agv.vx = round(msg.twist.twist.linear.x, 3)
            agv.wz = round(msg.twist.twist.angular.z, 3)
            agv.last_odom_ts = now

            use_odom = self.pose_source_mode == "odom"
            fallback_to_odom = (
                self.pose_source_mode == "map_then_odom"
                and now - agv.last_map_pose_ts > self.pose_stale_timeout
            )
            if use_odom or fallback_to_odom or agv.pose_source == "unknown":
                agv.x = agv.odom_x
                agv.y = agv.odom_y
                agv.yaw = agv.odom_yaw
                agv.pose_source = "odom"

    def _on_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            aid = data.get("agv_id") or data.get("agv")
            if aid not in self.agvs:
                return
            with self.lock:
                agv = self.agvs[aid]
                agv.state = State(data.get("state", agv.state.value))
                agv.battery = float(data.get("battery", agv.battery))
        except Exception as exc:
            self.get_logger().warn(f"AGV status parse error: {exc}")

    def _on_task(self, msg: String):
        try:
            data = json.loads(msg.data)
            shelf = data.get("shelf", "A1")
            if shelf not in self.shelves:
                self.get_logger().warn(f"Unknown shelf: {shelf}")
                return
            shelf_location = self.shelves[shelf]

            self._task_cnt += 1
            task = Task(
                tid=data.get("tid", f"T{self._task_cnt:04d}"),
                shelf=shelf,
                shelf_center_xy=shelf_location.center_xy,
                pick_xy=shelf_location.pick_xy,
                pick_yaw=shelf_location.pick_yaw,
                aisle_exit_xy=(
                    self.DEFAULT_AISLE_EXIT_X,
                    shelf_location.pick_xy[1],
                ),
                drop_xy=self.station_xy,
                priority=int(data.get("priority", 1)),
                requested_agv=data.get("agv_id", data.get("agv", "")),
            )
            with self.lock:
                self.queue.append(task)
                self.queue.sort()
            self.get_logger().info(
                f"[QUEUE] {task.tid} shelf={shelf} priority={task.priority} "
                f"pending={len(self.queue)}")
        except Exception as exc:
            self.get_logger().error(f"Task parse failed: {exc}")

    def _pair_key(self, left: str, right: str) -> str:
        ordered = sorted([left, right])
        return f"{ordered[0]}::{ordered[1]}"

    def _cleanup_conflict_locks_locked(self, now: Optional[float] = None):
        now = now or time.time()
        for key, conflict in list(self.conflict_locks.items()):
            winner = self.agvs.get(conflict.winner_agv)
            loser = self.agvs.get(conflict.loser_agv)
            if not winner or not loser:
                del self.conflict_locks[key]
                continue
            if winner.state == State.IDLE and loser.state != State.WAITING:
                del self.conflict_locks[key]
                continue
            dist = math.hypot(winner.x - loser.x, winner.y - loser.y)
            if dist >= self.right_of_way_release_distance:
                del self.conflict_locks[key]
                continue
            if now - conflict.created_at > max(
                    self.route_hold_timeout,
                    self.yield_hold_duration + self.yield_cooldown_duration + 5.0):
                del self.conflict_locks[key]

    def _clear_conflict_locks_for_agv_locked(self, aid: str):
        for key in list(self.conflict_locks):
            conflict = self.conflict_locks[key]
            if conflict.winner_agv == aid or conflict.loser_agv == aid:
                del self.conflict_locks[key]

    def _safety_loop(self):
        yield_requests = []
        with self.lock:
            now = time.time()
            self._cleanup_conflict_locks_locked(now)
            agvs = list(self.agvs.values())
            for i, left in enumerate(agvs):
                for right in agvs[i + 1:]:
                    if left.state == State.IDLE and right.state == State.IDLE:
                        continue
                    dist = math.hypot(left.x - right.x, left.y - right.y)
                    if dist >= self.safety_stop_distance:
                        continue

                    pair_key = self._pair_key(left.aid, right.aid)
                    conflict = self.conflict_locks.get(pair_key)
                    shared_zones = self._shared_traffic_zone_ids(left, right)
                    if not conflict:
                        victim = self._right_of_way_victim(left, right, now)
                        if not victim or not victim.task:
                            continue
                        winner = right if victim.aid == left.aid else left
                        conflict = ConflictLock(
                            key=pair_key,
                            winner_agv=winner.aid,
                            loser_agv=victim.aid,
                            zones=shared_zones,
                        )
                        self.conflict_locks[pair_key] = conflict

                    loser = self.agvs.get(conflict.loser_agv)
                    if loser and loser.task:
                        yield_requests.append((
                            loser.aid,
                            conflict.winner_agv,
                            dist,
                            conflict.zones,
                        ))

        for aid, other, dist, zones in yield_requests:
            self._yield_for_right_of_way(aid, other, dist, zones)

        for agv in self.agvs.values():
            if agv.state != State.IDLE and agv.task is None:
                self._publish_stop(agv.aid)

    def _right_of_way_victim(
            self,
            left: AGVState,
            right: AGVState,
            now: float) -> Optional[AGVState]:
        if left.state == State.WAITING or right.state == State.WAITING:
            return None
        if left.yield_cooldown_until > now or right.yield_cooldown_until > now:
            return None

        # 最高优先级：低电量或正在充电的车拥有路权，其他车让行
        left_low = (left.state == State.TO_CHARGE
                    or left.battery < self.battery_low_threshold)
        right_low = (right.state == State.TO_CHARGE
                     or right.battery < self.battery_low_threshold)
        if left_low and not right_low:
            return right   # right 让行给低电量的 left
        if right_low and not left_low:
            return left    # left 让行给低电量的 right

        if left.task and not right.task:
            return left
        if right.task and not left.task:
            return right

        left_moving = left.current_goal_handle is not None
        right_moving = right.current_goal_handle is not None
        if left_moving and not right_moving:
            return left
        if right_moving and not left_moving:
            return right

        return self._lower_priority_agv(left, right)

    def _goal_for_state(
            self,
            task: Task,
            state: State) -> Tuple[Tuple[float, float], float, str]:
        if state == State.TO_SHELF:
            return task.pick_xy, task.pick_yaw, "shelf"
        if state == State.TO_AISLE_EXIT:
            return task.aisle_exit_xy, 0.0, "aisle exit"
        if state == State.TO_STATION:
            return self.station_xy, 0.0, "station"
        if state == State.TO_CHARGE:
            return self.charging_xy, 0.0, "charging"
        raise ValueError(f"Unsupported stage goal for state {state.value}")

    def _route_points_for_stage(
            self,
            start: Tuple[float, float],
            goal: Tuple[float, float],
            stage: State) -> List[Tuple[float, float]]:
        if stage == State.TO_STATION and abs(start[1] - goal[1]) > 0.2:
            return [start, (start[0], goal[1]), goal]
        return [start, goal]

    def _sample_path_points(
            self,
            points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        samples: List[Tuple[float, float]] = []
        for idx in range(len(points) - 1):
            start = points[idx]
            end = points[idx + 1]
            dist = math.hypot(end[0] - start[0], end[1] - start[1])
            steps = max(1, int(math.ceil(dist / self.path_sample_step)))
            for step in range(steps + 1):
                ratio = step / steps
                x = start[0] + (end[0] - start[0]) * ratio
                y = start[1] + (end[1] - start[1]) * ratio
                samples.append((x, y))
        if not samples and points:
            samples.append(points[0])
        return samples

    def _path_zones(self, points: List[Tuple[float, float]]) -> Set[str]:
        zones: Set[str] = set()
        samples = self._sample_path_points(points)
        for x, y in samples:
            gx = math.floor(x / self.route_cell_size)
            gy = math.floor(y / self.route_cell_size)
            zones.add(f"cell:{gx}:{gy}")
            for zone_id in self._traffic_zones_for_point((x, y)):
                zones.add(f"traffic:{zone_id}")
        return zones

    def _zones_for_stage(
            self,
            agv: AGVState,
            task: Task,
            stage: State,
            start_xy: Optional[Tuple[float, float]] = None) -> Set[str]:
        start_xy = start_xy or (agv.x, agv.y)
        goal_xy, _, _ = self._goal_for_state(task, stage)
        route_points = self._route_points_for_stage(start_xy, goal_xy, stage)
        zones = self._path_zones(route_points)
        if stage in (State.TO_SHELF, State.TO_AISLE_EXIT):
            zones.add(f"shelf:{task.shelf}")
        if stage == State.TO_STATION:
            zones.add("dock:station")
        return zones

    def _zone_is_exclusive(self, zone: str) -> bool:
        if zone.startswith(("cell:", "shelf:", "dock:", "wait:")):
            return True
        if zone.startswith("traffic:"):
            zone_id = zone.split(":", 1)[1]
            raw = self.traffic_zones.get(zone_id)
            if not raw:
                return True
            return raw.zone_type == "exclusive"
        return True

    def _traffic_zones_for_point(self, point: Tuple[float, float]) -> Set[str]:
        return {
            zone_id for zone_id, zone in self.traffic_zones.items()
            if self._point_in_polygon(point, zone.polygon)
        }

    def _point_in_polygon(
            self,
            point: Tuple[float, float],
            polygon: Tuple[Tuple[float, float], ...]) -> bool:
        x, y = point
        inside = False
        for idx in range(len(polygon)):
            x1, y1 = polygon[idx]
            x2, y2 = polygon[(idx + 1) % len(polygon)]
            intersects = ((y1 > y) != (y2 > y))
            if not intersects:
                continue
            cross_x = (x2 - x1) * (y - y1) / ((y2 - y1) or 1e-9) + x1
            if x < cross_x:
                inside = not inside
        return inside

    def _shared_traffic_zone_ids(self, left: AGVState, right: AGVState) -> Set[str]:
        left_zones = set(
            zone.split(":", 1)[1]
            for zone in left.reserved_zones
            if zone.startswith("traffic:"))
        right_zones = set(
            zone.split(":", 1)[1]
            for zone in right.reserved_zones
            if zone.startswith("traffic:"))
        left_zones.update(self._traffic_zones_for_point((left.x, left.y)))
        right_zones.update(self._traffic_zones_for_point((right.x, right.y)))
        return left_zones & right_zones

    def _select_wait_point(
            self,
            agv: AGVState,
            preferred_zone_ids: Optional[Set[str]] = None) -> Optional[WaitPoint]:
        candidates: List[WaitPoint] = []
        preferred_zone_ids = preferred_zone_ids or set()
        for zone_id in preferred_zone_ids:
            zone = self.traffic_zones.get(zone_id)
            if zone and agv.aid in zone.wait_points:
                candidates.append(zone.wait_points[agv.aid])
        if not candidates:
            for zone in self.traffic_zones.values():
                if agv.aid in zone.wait_points:
                    candidates.append(zone.wait_points[agv.aid])
        if not candidates:
            candidates.extend(self.global_wait_points.values())
        if not candidates:
            return None
        return min(
            candidates,
            key=lambda item: math.hypot(agv.x - item.xy[0], agv.y - item.xy[1]),
        )

    def _reserve_stage_locked(
            self,
            agv: AGVState,
            task: Task,
            stage: State,
            start_xy: Optional[Tuple[float, float]] = None) -> Tuple[str, Set[str]]:
        zones = self._zones_for_stage(agv, task, stage, start_xy)
        blockers: Dict[str, Set[str]] = {}
        for zone in zones:
            if not self._zone_is_exclusive(zone):
                continue
            reservation = self.route_reservations.get(zone)
            if reservation and reservation.agv_id != agv.aid:
                blockers.setdefault(reservation.agv_id, set()).add(zone)

        if blockers:
            blocker_id, blocked_zones = max(
                blockers.items(),
                key=lambda item: (len(item[1]), item[0]),
            )
            return blocker_id, blocked_zones

        self._release_route_locked(agv.aid)
        expires_at = time.time() + self.route_hold_timeout
        reservation = RouteReservation(
            agv_id=agv.aid,
            task_id=task.tid,
            stage=stage.value,
            zones=zones,
            expires_at=expires_at,
        )
        for zone in zones:
            if self._zone_is_exclusive(zone):
                self.route_reservations[zone] = reservation
        agv.reserved_stage = stage.value
        agv.reserved_zones = set(zones)
        agv.reservation_deadline = expires_at
        return "", set()

    def _reservation_refresh_loop(self):
        with self.lock:
            self._cleanup_route_reservations_locked()
            now = time.time()
            for agv in self.agvs.values():
                if not agv.task or not agv.reserved_zones:
                    continue
                if agv.state in (State.IDLE, State.ERROR):
                    continue
                expires_at = now + self.route_hold_timeout
                agv.reservation_deadline = expires_at
                for zone in agv.reserved_zones:
                    reservation = self.route_reservations.get(zone)
                    if reservation and reservation.agv_id == agv.aid:
                        reservation.expires_at = expires_at

    def _cleanup_route_reservations_locked(self):
        now = time.time()
        for zone in list(self.route_reservations):
            reservation = self.route_reservations[zone]
            if reservation.expires_at >= now:
                continue
            agv = self.agvs.get(reservation.agv_id)
            if agv and agv.task and agv.state not in (State.IDLE, State.ERROR):
                if self._pose_age_locked(agv, now) <= self.pose_stale_timeout * 2.0:
                    reservation.expires_at = now + self.route_hold_timeout
                    continue
            if agv:
                agv.reserved_zones.discard(zone)
                if not agv.reserved_zones:
                    agv.reserved_stage = ""
                    agv.reservation_deadline = 0.0
            del self.route_reservations[zone]

    def _battery_loop(self):
        charge_agvs = []
        emergency_agvs = []
        fully_charged = []
        with self.lock:
            for agv in self.agvs.values():
                if agv.state == State.CHARGING:
                    agv.battery = min(
                        100.0, round(agv.battery + self.battery_charge_rate, 3))
                    if agv.battery >= self.battery_full_threshold:
                        agv.state = State.IDLE
                        self._release_route_locked(agv.aid)
                        fully_charged.append((agv.aid, agv.battery))
                    continue

                moving = abs(agv.vx) > 0.01 or abs(agv.wz) > 0.01
                drain = self.battery_drain_moving if moving else self.battery_drain_idle
                agv.battery = max(0.0, round(agv.battery - drain, 3))

                if agv.state == State.TO_CHARGE:
                    continue

                if agv.battery < self.battery_critical_threshold:
                    emergency_agvs.append(agv.aid)
                elif (agv.battery < self.battery_low_threshold
                        and agv.state == State.IDLE
                        and agv.task is None):
                    charge_agvs.append(agv.aid)

        for aid, pct in fully_charged:
            self.get_logger().info(
                f"[CHARGE] {aid} fully charged ({pct:.1f}%), returning to idle")

        for aid in emergency_agvs:
            agv = self.agvs.get(aid)
            if agv:
                self._emergency_charge(agv)

        for aid in charge_agvs:
            agv = self.agvs.get(aid)
            if agv:
                self._send_to_charge(agv)

    def _send_to_charge(self, agv: AGVState):
        charge_task = Task(
            tid=f"CHARGE_{agv.aid}",
            shelf="",
            shelf_center_xy=self.charging_xy,
            pick_xy=self.charging_xy,
            pick_yaw=0.0,
            aisle_exit_xy=self.charging_xy,
            drop_xy=self.charging_xy,
            priority=0,
            agv=agv.aid,
            status="charging",
        )
        with self.lock:
            if agv.state != State.IDLE or agv.task is not None:
                return
            agv.state = State.TO_CHARGE
            agv.task = charge_task

        if not self._send_nav(self.charging_xy, 0.0, charge_task, agv):
            with self.lock:
                if agv.state == State.TO_CHARGE:
                    agv.state = State.IDLE
                    agv.task = None
            self.get_logger().warn(
                f"[CHARGE] {agv.aid} Nav2 not ready, will retry on next battery check")
            return
        self.get_logger().warn(
            f"[CHARGE] {agv.aid} battery={agv.battery:.1f}% < "
            f"{self.battery_low_threshold:.0f}%, heading to charger at "
            f"({self.charging_xy[0]:.1f},{self.charging_xy[1]:.1f})")

    def _emergency_charge(self, agv: AGVState):
        """Highest-priority charge: interrupt any ongoing task and go charge now."""
        charge_task = Task(
            tid=f"CHARGE_{agv.aid}",
            shelf="",
            shelf_center_xy=self.charging_xy,
            pick_xy=self.charging_xy,
            pick_yaw=0.0,
            aisle_exit_xy=self.charging_xy,
            drop_xy=self.charging_xy,
            priority=0,
            agv=agv.aid,
            status="charging",
        )
        interrupted_tid = None
        goal_handle = None
        with self.lock:
            if agv.state in (State.TO_CHARGE, State.CHARGING):
                return
            if agv.task and not agv.task.tid.startswith("CHARGE_"):
                interrupted_tid = agv.task.tid
                goal_handle = agv.current_goal_handle
                agv.task.status = "pending"
                agv.task.agv = ""
                agv.task.last_error = "battery critical"
                agv.task.retry_after = time.time() + 10.0
                if not any(existing.tid == agv.task.tid for existing in self.queue):
                    self.queue.append(agv.task)
                    self.queue.sort()
            agv.state = State.TO_CHARGE
            agv.task = charge_task
            agv.current_goal_handle = None
            agv.current_goal_xy = None
            agv.current_goal_yaw = 0.0
            agv.nav_goal_sent_ts = 0.0
            agv.nav_goal_accepted_ts = 0.0
            agv.pause_until = 0.0
            agv.resume_state = State.IDLE
            agv.resume_goal_xy = None
            agv.resume_goal_yaw = 0.0
            agv.wait_until = 0.0
            agv.wait_point_xy = None
            agv.wait_point_yaw = 0.0
            agv.wait_reason = ""
            agv.wait_zone = ""
            agv.yielding_to = ""
            agv.yield_cooldown_until = 0.0
            self._release_route_locked(agv.aid)
            self._clear_conflict_locks_for_agv_locked(agv.aid)

        if goal_handle:
            goal_handle.cancel_goal_async()

        if not self._send_nav(self.charging_xy, 0.0, charge_task, agv):
            with self.lock:
                if agv.state == State.TO_CHARGE:
                    agv.state = State.IDLE
                    agv.task = None
            self.get_logger().warn(
                f"[CHARGE] {agv.aid} CRITICAL: Nav2 not ready for emergency charge")
            return

        self.get_logger().warn(
            f"[CHARGE] {agv.aid} CRITICAL battery={agv.battery:.1f}% < "
            f"{self.battery_critical_threshold:.0f}% — interrupted "
            f"{interrupted_tid or 'idle'}, going to charger NOW")

    def _release_route_locked(self, aid: str):
        for zone in list(self.route_reservations):
            if self.route_reservations[zone].agv_id == aid:
                del self.route_reservations[zone]
        agv = self.agvs.get(aid)
        if agv:
            agv.reserved_stage = ""
            agv.reserved_zones.clear()
            agv.reservation_deadline = 0.0

    def _prepare_stage_dispatch_locked(
            self,
            agv: AGVState,
            task: Task,
            stage: State,
            wait_on_block: bool) -> dict:
        goal_xy, goal_yaw, label = self._goal_for_state(task, stage)
        blocker, blocked_zones = self._reserve_stage_locked(
            agv, task, stage, start_xy=(agv.x, agv.y))
        if blocker:
            if wait_on_block:
                wait_point = self._set_wait_state_locked(
                    agv,
                    task,
                    resume_state=stage,
                    resume_goal_xy=goal_xy,
                    resume_goal_yaw=goal_yaw,
                    reason="reservation",
                    blocker=blocker,
                    blocked_zone_ids=blocked_zones,
                    hold_until=0.0,
                )
                return {
                    "action": "wait",
                    "goal_xy": goal_xy,
                    "goal_yaw": goal_yaw,
                    "label": label,
                    "blocker": blocker,
                    "wait_point": wait_point,
                }
            return {
                "action": "blocked",
                "blocker": blocker,
                "blocked_zones": blocked_zones,
            }

        agv.state = stage
        agv.wait_reason = ""
        agv.wait_zone = ""
        agv.wait_until = 0.0
        agv.wait_point_xy = None
        agv.wait_point_yaw = 0.0
        agv.yielding_to = ""
        task.status = "running"
        return {
            "action": "dispatch",
            "goal_xy": goal_xy,
            "goal_yaw": goal_yaw,
            "label": label,
        }

    def _set_wait_state_locked(
            self,
            agv: AGVState,
            task: Task,
            resume_state: State,
            resume_goal_xy: Tuple[float, float],
            resume_goal_yaw: float,
            reason: str,
            blocker: str,
            blocked_zone_ids: Set[str],
            hold_until: float) -> Optional[WaitPoint]:
        wait_point = self._select_wait_point(
            agv,
            {
                zone.split(":", 1)[1]
                for zone in blocked_zone_ids
                if zone.startswith("traffic:")
            },
        )
        agv.state = State.WAITING
        agv.resume_state = resume_state
        agv.resume_goal_xy = resume_goal_xy
        agv.resume_goal_yaw = resume_goal_yaw
        agv.wait_reason = reason
        agv.wait_zone = ",".join(sorted(blocked_zone_ids))
        agv.wait_until = hold_until
        agv.wait_point_xy = wait_point.xy if wait_point else None
        agv.wait_point_yaw = wait_point.yaw if wait_point else 0.0
        agv.yielding_to = blocker
        task.status = f"waiting:{blocker or reason}"
        return wait_point

    def _send_wait_nav_or_stop(
            self,
            agv: AGVState,
            task: Task,
            wait_point: Optional[WaitPoint],
            reason: str,
            blocker: str):
        if not wait_point:
            self._publish_stop(agv.aid)
            self.get_logger().warn(
                f"[WAIT] {agv.aid} waits in place; no configured wait point "
                f"for {reason} ({blocker})")
            return

        dist = math.hypot(agv.x - wait_point.xy[0], agv.y - wait_point.xy[1])
        if dist <= self.wait_point_tolerance:
            self._publish_stop(agv.aid)
            self.get_logger().info(
                f"[WAIT] {agv.aid} holds at wait point {wait_point.name} "
                f"for {reason} ({blocker})")
            return

        if not self._send_nav(wait_point.xy, wait_point.yaw, task, agv):
            self._publish_stop(agv.aid)
            self.get_logger().warn(
                f"[WAIT] {agv.aid} could not navigate to wait point "
                f"{wait_point.name}; holding position")
            return

        self.get_logger().warn(
            f"[WAIT] {agv.aid} -> {wait_point.name} for {reason} "
            f"({blocker})")

    def _assignment_cost(self, agv: AGVState, task: Task) -> float:
        """Rank candidate AGVs for a task (lower is better).

        Nearest AGV to the pickup wins; a battery below the low threshold
        adds a penalty so a nearly empty AGV is not sent on a long run while
        a fuller one is idle.
        """
        dist = math.hypot(
            agv.x - task.pick_xy[0], agv.y - task.pick_xy[1])
        battery_penalty = max(
            0.0, self.battery_low_threshold - agv.battery) * 0.1
        return dist + battery_penalty

    def _sched_loop(self):
        assignments = []
        with self.lock:
            self._cleanup_route_reservations_locked()
            now = time.time()
            idle = [
                agv for agv in self.agvs.values()
                if agv.state == State.IDLE
                and agv.battery > self.battery_min_dispatch
            ]
            if not self.queue or not idle:
                return

            assigned_agvs: Set[str] = set()
            assigned_tids: Set[str] = set()
            for task in list(self.queue):
                if len(assigned_agvs) >= len(idle):
                    break
                if task.retry_after > now:
                    continue
                candidates = [
                    agv for agv in idle
                    if agv.aid not in assigned_agvs
                    and (not task.requested_agv
                         or agv.aid == task.requested_agv)
                ]
                if not candidates:
                    continue
                candidates.sort(
                    key=lambda agv: self._assignment_cost(agv, task))
                for agv in candidates:
                    prep = self._prepare_stage_dispatch_locked(
                        agv,
                        task,
                        State.TO_SHELF,
                        wait_on_block=False,
                    )
                    if prep["action"] != "dispatch":
                        task.status = f"waiting:{prep['blocker']}"
                        continue

                    task.agv = agv.aid
                    task.status = "running"
                    agv.task = task
                    assigned_agvs.add(agv.aid)
                    assigned_tids.add(task.tid)
                    assignments.append((agv.aid, task, prep))
                    break

            if assigned_tids:
                self.queue = [
                    task for task in self.queue
                    if task.tid not in assigned_tids
                ]

        for aid, task, prep in assignments:
            agv = self.agvs[aid]
            self.get_logger().info(
                f"[ASSIGN] {task.tid} -> {aid} shelf={task.shelf} "
                f"center=({task.shelf_center_xy[0]:.1f},"
                f"{task.shelf_center_xy[1]:.1f}) "
                f"pickup=({task.pick_xy[0]:.1f},{task.pick_xy[1]:.1f},"
                f"yaw={task.pick_yaw:.2f})")
            self._pub_assign(agv, task)
            if not self._send_nav(
                    prep["goal_xy"], prep["goal_yaw"], task, agv):
                self._return_task_to_queue(agv, task, "Nav2 server not ready")
                continue
            with self.lock:
                if not any(
                        existing.tid == task.tid
                        for existing in self.history):
                    self.history.append(task)

    def _send_nav(
            self,
            xy: Tuple[float, float],
            yaw: float,
            task: Task,
            agv: AGVState) -> bool:
        client = self.nav_clients[agv.aid]
        if not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                f"[Nav2] {agv.aid} action not ready: {agv.nav_action}")
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(xy[0])
        goal.pose.pose.position.y = float(xy[1])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        with self.lock:
            current = self.agvs.get(agv.aid)
            if not current:
                return False
            current.nav_goal_seq += 1
            goal_seq = current.nav_goal_seq
            current.current_goal_handle = None
            current.current_goal_xy = xy
            current.current_goal_yaw = yaw
            current.nav_goal_sent_ts = time.time()
            current.nav_goal_accepted_ts = 0.0

        future = client.send_goal_async(goal)
        future.add_done_callback(
            lambda f, aid=agv.aid, tid=task.tid, seq=goal_seq:
            self._nav_accepted(f, aid, tid, seq))
        self.get_logger().info(
            f"[Nav2] {agv.aid} goal=({xy[0]:.1f},{xy[1]:.1f},"
            f"yaw={yaw:.2f}) via {agv.nav_action}")
        return True

    def _nav_accepted(self, future, aid: str, task_id: str, goal_seq: int):
        try:
            goal_handle = future.result()
        except Exception as exc:
            agv = self.agvs.get(aid)
            task = agv.task if agv else None
            if agv and task and task.tid == task_id:
                self._return_task_to_queue(
                    agv, task, f"goal response failed: {exc}")
            return

        if goal_handle is None:
            agv = self.agvs.get(aid)
            task = agv.task if agv else None
            if agv and task and task.tid == task_id:
                self._return_task_to_queue(agv, task, "empty goal response")
            return

        if not goal_handle.accepted:
            agv = self.agvs.get(aid)
            task = agv.task if agv else None
            if agv and task and task.tid == task_id:
                self.get_logger().warn(
                    f"[Nav2] goal rejected: {task.tid} on {agv.aid}")
                self._return_task_to_queue(agv, task, "goal rejected")
            return

        with self.lock:
            current = self.agvs.get(aid)
            if not current or current.nav_goal_seq != goal_seq:
                goal_handle.cancel_goal_async()
                return
            if not current.task or current.task.tid != task_id:
                goal_handle.cancel_goal_async()
                return
            current.current_goal_handle = goal_handle
            current.nav_goal_accepted_ts = time.time()

        goal_handle.get_result_async().add_done_callback(
            lambda f, aid=aid, tid=task_id, seq=goal_seq:
            self._nav_done(f, aid, tid, seq))

    def _nav_done(self, future, aid: str, task_id: str, goal_seq: int):
        try:
            result = future.result()
        except Exception as exc:
            agv = self.agvs.get(aid)
            task = agv.task if agv else None
            if agv and task and task.tid == task_id:
                self._return_task_to_queue(
                    agv, task, f"navigation result failed: {exc}")
            return

        status = getattr(result, "status", None)
        next_action = None
        publish_stop = False
        wait_release = False

        with self.lock:
            agv = self.agvs.get(aid)
            if not agv or agv.nav_goal_seq != goal_seq:
                return
            if not agv.task or agv.task.tid != task_id:
                return
            task = agv.task
            agv.current_goal_handle = None
            agv.nav_goal_sent_ts = 0.0
            agv.nav_goal_accepted_ts = 0.0

            if agv.state == State.WAITING:
                if status == GoalStatus.STATUS_SUCCEEDED:
                    if agv.wait_point_xy and math.hypot(
                            agv.x - agv.wait_point_xy[0],
                            agv.y - agv.wait_point_xy[1]) <= 1.0:
                        self._release_route_locked(agv.aid)
                        wait_release = True
                publish_stop = True
            elif agv.state == State.TO_CHARGE:
                if status == GoalStatus.STATUS_SUCCEEDED:
                    agv.state = State.CHARGING
                    self.get_logger().info(
                        f"[CHARGE] {agv.aid} docked at charger, charging ...")
                else:
                    agv.state = State.IDLE
                    self.get_logger().warn(
                        f"[CHARGE] {agv.aid} failed to reach charger "
                        f"(status={status}), will retry")
                agv.task = None
                agv.current_goal_xy = None
                agv.current_goal_yaw = 0.0
                self._release_route_locked(agv.aid)
                self._clear_conflict_locks_for_agv_locked(agv.aid)
                publish_stop = True
            elif status != GoalStatus.STATUS_SUCCEEDED:
                next_action = ("requeue", task, f"navigation status {status}")
            elif agv.state == State.TO_SHELF:
                if self.pickup_pause_duration > 0.0:
                    agv.state = State.PICKING
                    agv.pause_until = time.time() + self.pickup_pause_duration
                    task.status = "picking"
                    publish_stop = True
                else:
                    next_action = (
                        "transition",
                        task,
                        self._prepare_stage_dispatch_locked(
                            agv, task, State.TO_AISLE_EXIT, wait_on_block=True),
                    )
            elif agv.state == State.TO_AISLE_EXIT:
                next_action = (
                    "transition",
                    task,
                    self._prepare_stage_dispatch_locked(
                        agv, task, State.TO_STATION, wait_on_block=True),
                )
            elif agv.state == State.TO_STATION:
                agv.state = State.IDLE
                agv.task = None
                agv.current_goal_xy = None
                agv.current_goal_yaw = 0.0
                agv.pause_until = 0.0
                agv.resume_state = State.IDLE
                agv.resume_goal_xy = None
                agv.resume_goal_yaw = 0.0
                agv.wait_until = 0.0
                agv.wait_point_xy = None
                agv.wait_point_yaw = 0.0
                agv.wait_reason = ""
                agv.wait_zone = ""
                agv.yielding_to = ""
                agv.yield_cooldown_until = 0.0
                self._release_route_locked(agv.aid)
                self._clear_conflict_locks_for_agv_locked(agv.aid)
                task.status = "done"
                self.get_logger().info(
                    f"[DONE] {task.tid} completed by {agv.aid}")
                return
            else:
                return

        if wait_release:
            self.get_logger().info(
                f"[WAIT] {aid} parked at safe wait point and released its "
                "old reservation")
        if publish_stop:
            self._publish_stop(aid)
            return

        if not next_action:
            return
        if next_action[0] == "requeue":
            _, task, reason = next_action
            self._return_task_to_queue(self.agvs[aid], task, reason)
            return

        _, task, prep = next_action
        self._execute_prepared_stage(self.agvs[aid], task, prep)

    def _execute_prepared_stage(self, agv: AGVState, task: Task, prep: dict):
        action = prep.get("action")
        if action == "dispatch":
            self.get_logger().info(
                f"[ARRIVE] {agv.aid} heading to {prep['label']}")
            if not self._send_nav(prep["goal_xy"], prep["goal_yaw"], task, agv):
                self._return_task_to_queue(
                    agv, task, f"{prep['label']} navigation unavailable")
            return

        if action == "wait":
            blocker = prep.get("blocker", "traffic")
            wait_point = prep.get("wait_point")
            self._send_wait_nav_or_stop(
                agv,
                task,
                wait_point,
                reason=f"stage:{prep.get('label', 'next')}",
                blocker=blocker,
            )
            return

    def _return_task_to_queue(self, agv: AGVState, task: Task, reason: str):
        is_charge_task = task.tid.startswith("CHARGE_")
        with self.lock:
            current = self.agvs.get(agv.aid)
            if current:
                current.state = State.IDLE
                current.task = None
                current.current_goal_handle = None
                current.current_goal_xy = None
                current.current_goal_yaw = 0.0
                current.nav_goal_sent_ts = 0.0
                current.nav_goal_accepted_ts = 0.0
                current.pause_until = 0.0
                current.resume_state = State.IDLE
                current.resume_goal_xy = None
                current.resume_goal_yaw = 0.0
                current.wait_until = 0.0
                current.wait_point_xy = None
                current.wait_point_yaw = 0.0
                current.wait_reason = ""
                current.wait_zone = ""
                current.yielding_to = ""
                current.yield_cooldown_until = 0.0
                self._release_route_locked(current.aid)
                self._clear_conflict_locks_for_agv_locked(current.aid)
            if not is_charge_task:
                task.status = "pending"
                task.agv = ""
                task.last_error = reason
                task.retry_after = time.time() + 5.0
                if not any(existing.tid == task.tid for existing in self.queue):
                    self.queue.append(task)
                    self.queue.sort()
        self._publish_stop(agv.aid)
        if is_charge_task:
            self.get_logger().warn(f"[CHARGE] {agv.aid} charge trip aborted: {reason}")
        else:
            self.get_logger().warn(f"[REQUEUE] {task.tid}: {reason}")

    def _goal_for_active_state(
            self,
            agv: AGVState) -> Tuple[Optional[Tuple[float, float]], float]:
        task = agv.task
        if not task:
            return None, 0.0
        if agv.state in (State.TO_SHELF, State.TO_AISLE_EXIT, State.TO_STATION):
            goal_xy, goal_yaw, _ = self._goal_for_state(task, agv.state)
            return goal_xy, goal_yaw
        if agv.state == State.WAITING and agv.resume_goal_xy:
            return agv.resume_goal_xy, agv.resume_goal_yaw
        return agv.current_goal_xy, agv.current_goal_yaw

    def _yield_for_right_of_way(
            self,
            aid: str,
            other: str,
            dist: float,
            conflict_zones: Set[str]):
        goal_handle = None
        wait_point = None
        task = None
        with self.lock:
            agv = self.agvs[aid]
            if not agv.task or agv.state == State.WAITING:
                return
            now = time.time()
            if agv.yield_cooldown_until > now:
                return

            resume_goal_xy, resume_goal_yaw = self._goal_for_active_state(agv)
            if not resume_goal_xy:
                self._publish_stop(agv.aid)
                return

            goal_handle = agv.current_goal_handle
            task = agv.task
            wait_point = self._set_wait_state_locked(
                agv,
                task,
                resume_state=agv.state,
                resume_goal_xy=resume_goal_xy,
                resume_goal_yaw=resume_goal_yaw,
                reason="yield",
                blocker=other,
                blocked_zone_ids={f"traffic:{zone}" for zone in conflict_zones},
                hold_until=now + self.yield_hold_duration,
            )
            agv.current_goal_handle = None
            agv.current_goal_xy = None
            agv.current_goal_yaw = 0.0
            agv.nav_goal_sent_ts = 0.0
            agv.nav_goal_accepted_ts = 0.0
            agv.yield_cooldown_until = (
                now + self.yield_hold_duration + self.yield_cooldown_duration)

        if goal_handle:
            goal_handle.cancel_goal_async()
        if task:
            self._send_wait_nav_or_stop(
                self.agvs[aid],
                task,
                wait_point,
                reason="yield",
                blocker=other,
            )
        self.get_logger().warn(
            f"[YIELD] {aid} yields to {other}: "
            f"{dist:.2f}m < {self.safety_stop_distance:.2f}m")

    def _right_of_way_loop(self):
        resumes = []
        now = time.time()
        with self.lock:
            self._cleanup_conflict_locks_locked(now)
            for agv in self.agvs.values():
                if agv.state != State.WAITING or not agv.task:
                    continue
                if agv.current_goal_handle is not None:
                    continue
                if not agv.resume_goal_xy:
                    continue

                if agv.wait_reason == "yield":
                    if now < agv.wait_until:
                        continue
                    blocker = self.agvs.get(agv.yielding_to)
                    if blocker:
                        dist = math.hypot(agv.x - blocker.x, agv.y - blocker.y)
                        if dist < self.right_of_way_release_distance:
                            self._publish_stop(agv.aid)
                            continue

                prep = self._prepare_stage_dispatch_locked(
                    agv,
                    agv.task,
                    agv.resume_state,
                    wait_on_block=False,
                )
                if prep["action"] != "dispatch":
                    agv.task.status = f"waiting:{prep['blocker']}"
                    continue

                agv.wait_reason = ""
                agv.wait_zone = ""
                agv.wait_until = 0.0
                agv.wait_point_xy = None
                agv.wait_point_yaw = 0.0
                agv.yielding_to = ""
                resumes.append((agv, agv.task, prep))
                self._clear_conflict_locks_for_agv_locked(agv.aid)

        for agv, task, prep in resumes:
            self.get_logger().info(
                f"[RESUME] {agv.aid} resumes {agv.state.value} toward "
                f"({prep['goal_xy'][0]:.1f},{prep['goal_xy'][1]:.1f})")
            if not self._send_nav(prep["goal_xy"], prep["goal_yaw"], task, agv):
                self._return_task_to_queue(
                    agv, task, "right-of-way resume unavailable")

    def _pause_loop(self):
        dispatches = []
        now = time.time()
        with self.lock:
            for agv in self.agvs.values():
                if agv.state != State.PICKING or not agv.task:
                    continue
                if agv.pause_until <= 0.0 or now < agv.pause_until:
                    continue
                prep = self._prepare_stage_dispatch_locked(
                    agv,
                    agv.task,
                    State.TO_AISLE_EXIT,
                    wait_on_block=True,
                )
                agv.pause_until = 0.0
                dispatches.append((agv, agv.task, prep))

        for agv, task, prep in dispatches:
            self._execute_prepared_stage(agv, task, prep)

    def _nav_watchdog(self):
        victims = []
        now = time.time()
        with self.lock:
            for agv in self.agvs.values():
                if agv.state == State.IDLE or not agv.task:
                    continue
                if self._pose_age_locked(agv, now) > self.pose_stale_timeout:
                    victims.append((
                        agv,
                        agv.task,
                        f"pose source stale for {self.pose_stale_timeout:.1f}s",
                    ))
                    continue
                if not agv.nav_goal_sent_ts:
                    continue
                goal_pending = (
                    agv.current_goal_handle is None
                    and agv.nav_goal_accepted_ts == 0.0
                    and now - agv.nav_goal_sent_ts > 5.0
                )
                if goal_pending:
                    victims.append((
                        agv,
                        agv.task,
                        "Nav2 goal was not accepted within 5s",
                    ))

        for agv, task, reason in victims:
            self._return_task_to_queue(agv, task, reason)

    def _lower_priority_agv(self, left: AGVState, right: AGVState) -> AGVState:
        if left.task and not right.task:
            return left
        if right.task and not left.task:
            return right
        left_priority = left.task.priority if left.task else 0
        right_priority = right.task.priority if right.task else 0
        if left_priority == right_priority:
            return max(left, right, key=lambda item: item.aid)
        return left if left_priority < right_priority else right

    def _publish_stop(self, aid: str):
        publisher = self.cmd_publishers.get(aid)
        if not publisher:
            return
        publisher.publish(Twist())

    def _pub_assign(self, agv: AGVState, task: Task):
        msg = String()
        msg.data = json.dumps({
            "agv": agv.aid,
            "tid": task.tid,
            "shelf": task.shelf,
            "shelf_center": list(task.shelf_center_xy),
            "pick": list(task.pick_xy),
            "pick_yaw": task.pick_yaw,
            "aisle_exit": list(task.aisle_exit_xy),
            "drop": list(task.drop_xy),
            "priority": task.priority,
            "nav_action": agv.nav_action,
            "base_frame": agv.base_frame,
            "pose_source": agv.pose_source,
        }, ensure_ascii=False)
        self.pub_assign.publish(msg)

    def _pub_status(self):
        with self.lock:
            done = sum(1 for task in self.history if task.status == "done")
            payload = {
                "pending": len(self.queue),
                "completed": done,
                "queue": [
                    {
                        "tid": task.tid,
                        "shelf": task.shelf,
                        "status": task.status,
                        "requested_agv": task.requested_agv,
                        "retry_in": max(
                            0.0, round(task.retry_after - time.time(), 1)),
                        "last_error": task.last_error,
                    }
                    for task in self.queue
                ],
                "reservations": {
                    zone: {
                        "agv": reservation.agv_id,
                        "task": reservation.task_id,
                        "stage": reservation.stage,
                        "expires_in": round(
                            reservation.expires_at - time.time(), 1),
                    }
                    for zone, reservation in self.route_reservations.items()
                },
                "traffic_zones": {
                    zone_id: {
                        "type": zone.zone_type,
                        "wait_points": {
                            aid: [
                                round(wait.xy[0], 2),
                                round(wait.xy[1], 2),
                                round(wait.yaw, 2),
                            ]
                            for aid, wait in zone.wait_points.items()
                        },
                    }
                    for zone_id, zone in self.traffic_zones.items()
                },
                "fleet": {
                    aid: {
                        "state": agv.state.value,
                        "pos": [round(agv.x, 2), round(agv.y, 2)],
                        "pose_source": agv.pose_source,
                        "base_frame": agv.base_frame,
                        "battery": round(agv.battery, 1),
                        "vx": agv.vx,
                        "wz": agv.wz,
                        "task": agv.task.tid if agv.task else None,
                        "nav_action": agv.nav_action,
                        "odom_topic": agv.odom_topic,
                        "cmd_vel_topic": agv.cmd_vel_topic,
                        "goal": (
                            [round(agv.current_goal_xy[0], 2),
                             round(agv.current_goal_xy[1], 2)]
                            if agv.current_goal_xy else None
                        ),
                        "goal_active": agv.current_goal_handle is not None,
                        "yielding_to": agv.yielding_to or None,
                        "wait_reason": agv.wait_reason or None,
                        "wait_zone": agv.wait_zone or None,
                        "wait_point": (
                            [round(agv.wait_point_xy[0], 2),
                             round(agv.wait_point_xy[1], 2),
                             round(agv.wait_point_yaw, 2)]
                            if agv.wait_point_xy else None
                        ),
                        "resume_goal": (
                            [round(agv.resume_goal_xy[0], 2),
                             round(agv.resume_goal_xy[1], 2)]
                            if agv.resume_goal_xy else None
                        ),
                        "reserved_stage": agv.reserved_stage or None,
                        "reserved_zones": sorted(agv.reserved_zones),
                    }
                    for aid, agv in self.agvs.items()
                },
            }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_sched.publish(msg)

    def _auto_demo(self):
        if not self.auto_demo_enabled or self._demo_n >= 8:
            return
        import random

        shelf = random.choice(list(self.shelves.keys()))
        priority = random.randint(1, 5)
        msg = String()
        msg.data = json.dumps({
            "tid": f"AUTO_{self._demo_n + 1:03d}",
            "shelf": shelf,
            "priority": priority,
        }, ensure_ascii=False)
        self._on_task(msg)
        self._demo_n += 1


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
