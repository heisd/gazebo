#!/usr/bin/env python3
"""
AGV warehouse scheduler with fleet-level collision avoidance.

The node keeps the original single-AGV defaults, but can be configured with
multiple AGVs by passing per-vehicle odom, cmd_vel and Nav2 action names.
"""

import json
import math
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
from std_msgs.msg import String


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
class AGVState:
    aid: str
    nav_action: str
    odom_topic: str
    cmd_vel_topic: str
    status_topic: str
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
    pause_until: float = 0.0
    pending_goal_xy: Optional[Tuple[float, float]] = None
    pending_goal_yaw: float = 0.0
    pending_goal_label: str = ""
    resume_state: State = State.IDLE
    resume_goal_xy: Optional[Tuple[float, float]] = None
    resume_goal_yaw: float = 0.0
    wait_until: float = 0.0
    wait_point_xy: Optional[Tuple[float, float]] = None
    yielding_to: str = ""
    yield_cooldown_until: float = 0.0


@dataclass
class RouteReservation:
    agv_id: str
    task_id: str
    zones: Set[str]
    expires_at: float


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
    DEFAULT_STATION = (6.4, 0.0)
    DEFAULT_CHARGING = (9.0, -8.0)
    DEFAULT_AISLE_EXIT_X = 5.5

    def __init__(self):
        super().__init__("agv_scheduler")
        self._declare_params()

        self.shelves, self.station_xy, self.charging_xy = (
            self._load_warehouse_layout())
        self.route_cell_size = float(
            self.get_parameter("route_cell_size").value)
        self.route_hold_timeout = float(
            self.get_parameter("route_hold_timeout").value)
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
        self.auto_demo_enabled = bool(
            self.get_parameter("auto_demo_enabled").value)

        agv_ids = self._string_list_param("agv_ids", ["agv_01"])
        nav_actions = self._expanded_param(
            "nav_action_names", ["navigate_to_pose"], len(agv_ids))
        odom_topics = self._expanded_param(
            "odom_topics", ["/agv/odom"], len(agv_ids))
        cmd_vel_topics = self._expanded_param(
            "cmd_vel_topics", ["/agv/cmd_vel"], len(agv_ids))
        status_topics = self._expanded_param(
            "status_topics", ["/agv/agv_status"], len(agv_ids))

        self.agvs: Dict[str, AGVState] = {}
        self.nav_clients: Dict[str, ActionClient] = {}
        self.cmd_publishers: Dict[str, object] = {}
        for idx, aid in enumerate(agv_ids):
            agv = AGVState(
                aid=aid,
                nav_action=nav_actions[idx],
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

        for topic in sorted({agv.status_topic for agv in self.agvs.values()}):
            self.create_subscription(String, topic, self._on_status, 10)

        self.queue: List[Task] = []
        self.history: List[Task] = []
        self.route_reservations: Dict[str, RouteReservation] = {}
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
        self.create_timer(15.0, self._auto_demo)

        self.get_logger().info("=" * 50)
        self.get_logger().info(
            " AGV warehouse scheduler with fleet safety started")
        self.get_logger().info("=" * 50)
        self.get_logger().info(
            f"Shelves: {len(self.shelves)} | "
            f"station=({self.station_xy[0]:.1f},{self.station_xy[1]:.1f}) | "
            f"AGVs: {', '.join(self.agvs)}")

    def _declare_params(self):
        self.declare_parameter("agv_ids", ["agv_01"])
        self.declare_parameter("nav_action_names", ["navigate_to_pose"])
        self.declare_parameter("odom_topics", ["/agv/odom"])
        self.declare_parameter("cmd_vel_topics", ["/agv/cmd_vel"])
        self.declare_parameter("status_topics", ["/agv/agv_status"])
        self.declare_parameter("route_cell_size", 2.0)
        self.declare_parameter("route_hold_timeout", 180.0)
        self.declare_parameter("safety_stop_distance", 1.0)
        self.declare_parameter("right_of_way_release_distance", 1.6)
        self.declare_parameter("yield_hold_duration", 2.0)
        self.declare_parameter("yield_cooldown_duration", 3.0)
        self.declare_parameter("pickup_pause_duration", 4.0)
        self.declare_parameter("auto_demo_enabled", True)
        self.declare_parameter("shelf_layout_file", "")

    def _load_warehouse_layout(
            self) -> Tuple[Dict[str, ShelfLocation],
                           Tuple[float, float],
                           Tuple[float, float]]:
        layout_file = str(self.get_parameter("shelf_layout_file").value or "")
        if not layout_file:
            self.get_logger().warn(
                "No shelf_layout_file configured; using built-in layout")
            return (
                dict(self.DEFAULT_SHELVES),
                self.DEFAULT_STATION,
                self.DEFAULT_CHARGING,
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

        self.get_logger().info(f"Loaded shelf layout: {layout_file}")
        return shelves, station, charging

    def _xy_from_config(self, value, name: str) -> Tuple[float, float]:
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            raise ValueError(f"{name} must be a two-item [x, y] list")
        return (float(value[0]), float(value[1]))

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

    def _on_odom(self, aid: str, msg: Odometry):
        p = msg.pose.pose
        q = p.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        with self.lock:
            agv = self.agvs.get(aid)
            if not agv:
                return
            agv.x = round(p.position.x, 3)
            agv.y = round(p.position.y, 3)
            agv.yaw = round(yaw, 3)
            agv.vx = round(msg.twist.twist.linear.x, 3)
            agv.wz = round(msg.twist.twist.angular.z, 3)
            agv.last_odom_ts = time.time()

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

    def _sched_loop(self):
        assignment = None
        with self.lock:
            self._cleanup_route_reservations_locked()
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
                    key=lambda agv: math.hypot(agv.x - task.pick_xy[0],
                                               agv.y - task.pick_xy[1]))
                for agv in candidates:
                    blocker = self._reserve_route_locked(agv, task)
                    if blocker:
                        task.status = f"waiting:{blocker}"
                        continue

                    self.queue.pop(task_idx)
                    task.agv = agv.aid
                    task.status = "running"
                    agv.state = State.TO_SHELF
                    agv.task = task
                    assignment = (agv.aid, task)
                    break
                if assignment:
                    break

        if not assignment:
            return

        aid, task = assignment
        agv = self.agvs[aid]
        self.get_logger().info(
            f"[ASSIGN] {task.tid} -> {aid} shelf={task.shelf} "
            f"center=({task.shelf_center_xy[0]:.1f},"
            f"{task.shelf_center_xy[1]:.1f}) "
            f"pickup=({task.pick_xy[0]:.1f},{task.pick_xy[1]:.1f},"
            f"yaw={task.pick_yaw:.2f})")
        self._pub_assign(agv, task)
        if not self._send_nav(task.pick_xy, task.pick_yaw, task, agv):
            self._return_task_to_queue(agv, task, "Nav2 server not ready")
            return
        with self.lock:
            self.history.append(task)

    def _reserve_route_locked(self, agv: AGVState, task: Task) -> str:
        zones = self._zones_for_task(agv, task)
        for zone in zones:
            reservation = self.route_reservations.get(zone)
            if reservation and reservation.agv_id != agv.aid:
                return reservation.agv_id

        expires_at = time.time() + self.route_hold_timeout
        reservation = RouteReservation(
            agv_id=agv.aid,
            task_id=task.tid,
            zones=zones,
            expires_at=expires_at,
        )
        for zone in zones:
            self.route_reservations[zone] = reservation
        return ""

    def _cleanup_route_reservations_locked(self):
        now = time.time()
        expired = [
            zone for zone, reservation in self.route_reservations.items()
            if reservation.expires_at < now
        ]
        for zone in expired:
            del self.route_reservations[zone]

    def _release_route_locked(self, aid: str):
        for zone in list(self.route_reservations):
            if self.route_reservations[zone].agv_id == aid:
                del self.route_reservations[zone]

    def _zones_for_task(self, agv: AGVState, task: Task) -> Set[str]:
        zones: Set[str] = set()
        zones.update(self._segment_zones((agv.x, agv.y), task.pick_xy))
        zones.update(self._segment_zones(task.pick_xy, task.aisle_exit_xy))
        zones.update(self._segment_zones(task.aisle_exit_xy, task.drop_xy))
        zones.add(f"shelf:{task.shelf}")
        zones.add("dock:station")
        return zones

    def _segment_zones(
            self,
            start: Tuple[float, float],
            end: Tuple[float, float]) -> Set[str]:
        sx, sy = start
        ex, ey = end
        dist = max(math.hypot(ex - sx, ey - sy), self.route_cell_size)
        steps = max(1, int(math.ceil(dist / self.route_cell_size)))
        zones = set()
        for idx in range(steps + 1):
            ratio = idx / steps
            x = sx + (ex - sx) * ratio
            y = sy + (ey - sy) * ratio
            gx = math.floor(x / self.route_cell_size)
            gy = math.floor(y / self.route_cell_size)
            zones.add(f"cell:{gx}:{gy}")
        return zones

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
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(xy[0])
        goal.pose.pose.position.y = float(xy[1])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        with self.lock:
            current = self.agvs.get(agv.aid)
            if current:
                current.current_goal_handle = None
                current.current_goal_xy = xy
                current.current_goal_yaw = yaw
                current.nav_goal_sent_ts = time.time()
                current.nav_goal_accepted_ts = 0.0

        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._nav_accepted(f, task, agv))
        self.get_logger().info(
            f"[Nav2] {agv.aid} goal=({xy[0]:.1f},{xy[1]:.1f},"
            f"yaw={yaw:.2f}) "
            f"via {agv.nav_action}")
        return True

    def _nav_accepted(self, future, task: Task, agv: AGVState):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._return_task_to_queue(
                agv, task, f"goal response failed: {exc}")
            return

        if goal_handle is None:
            self._return_task_to_queue(agv, task, "empty goal response")
            return

        if not goal_handle.accepted:
            self.get_logger().warn(
                f"[Nav2] goal rejected: {task.tid} on {agv.aid}")
            self._return_task_to_queue(agv, task, "goal rejected")
            return

        with self.lock:
            current = self.agvs.get(agv.aid)
            if not current or not current.task or current.task.tid != task.tid:
                goal_handle.cancel_goal_async()
                return
            if current.state == State.WAITING:
                goal_handle.cancel_goal_async()
                return
            current.current_goal_handle = goal_handle
            current.nav_goal_accepted_ts = time.time()

        goal_handle.get_result_async().add_done_callback(
            lambda f, gh=goal_handle: self._nav_done(f, task, agv, gh))

    def _nav_done(self, future, task: Task, agv: AGVState, goal_handle):
        result = future.result()
        status = getattr(result, "status", None)

        with self.lock:
            current = self.agvs.get(agv.aid)
            if not current or not current.task or current.task.tid != task.tid:
                return
            if current.current_goal_handle is not goal_handle:
                return
            current.current_goal_handle = None

        if status != GoalStatus.STATUS_SUCCEEDED:
            self._return_task_to_queue(
                agv, task, f"navigation status {status}")
            return

        with self.lock:
            if agv.state == State.TO_SHELF:
                if self.pickup_pause_duration > 0.0:
                    agv.state = State.PICKING
                    agv.pause_until = time.time() + self.pickup_pause_duration
                    agv.pending_goal_xy = task.aisle_exit_xy
                    agv.pending_goal_yaw = 0.0
                    agv.pending_goal_label = "aisle exit"
                    task.status = "picking"
                    self.get_logger().info(
                        f"[PICKING] {agv.aid} reached {task.shelf}, "
                        f"waiting {self.pickup_pause_duration:.1f}s")
                    self._publish_stop(agv.aid)
                    return
                agv.state = State.TO_AISLE_EXIT
                next_xy = task.aisle_exit_xy
                next_yaw = 0.0
                next_label = "aisle exit"
            elif agv.state == State.TO_AISLE_EXIT:
                agv.state = State.TO_STATION
                next_xy = self.station_xy
                next_yaw = 0.0
                next_label = "station"
            elif agv.state == State.TO_STATION:
                agv.state = State.IDLE
                agv.task = None
                agv.current_goal_xy = None
                agv.nav_goal_sent_ts = 0.0
                agv.nav_goal_accepted_ts = 0.0
                task.status = "done"
                self._release_route_locked(agv.aid)
                self.get_logger().info(
                    f"[DONE] {task.tid} completed by {agv.aid}")
                return
            else:
                return

        self.get_logger().info(
            f"[ARRIVE] {agv.aid} reached {task.shelf} step, "
            f"heading to {next_label}")
        if not self._send_nav(next_xy, next_yaw, task, agv):
            self._return_task_to_queue(
                agv, task, f"{next_label} navigation unavailable")

    def _return_task_to_queue(self, agv: AGVState, task: Task, reason: str):
        with self.lock:
            current = self.agvs.get(agv.aid)
            if current:
                current.state = State.IDLE
                current.task = None
                current.current_goal_handle = None
                current.current_goal_xy = None
                current.nav_goal_sent_ts = 0.0
                current.nav_goal_accepted_ts = 0.0
                current.pause_until = 0.0
                current.pending_goal_xy = None
                current.pending_goal_yaw = 0.0
                current.pending_goal_label = ""
                current.resume_state = State.IDLE
                current.resume_goal_xy = None
                current.resume_goal_yaw = 0.0
                current.wait_until = 0.0
                current.wait_point_xy = None
                current.yielding_to = ""
                current.yield_cooldown_until = 0.0
                self._release_route_locked(current.aid)
            task.status = "pending"
            task.agv = ""
            task.last_error = reason
            task.retry_after = time.time() + 5.0
            if not any(existing.tid == task.tid for existing in self.queue):
                self.queue.append(task)
                self.queue.sort()
        self._publish_stop(agv.aid)
        self.get_logger().warn(f"[REQUEUE] {task.tid}: {reason}")

    def _safety_loop(self):
        yield_requests = []
        with self.lock:
            now = time.time()
            agvs = list(self.agvs.values())
            for i, left in enumerate(agvs):
                for right in agvs[i + 1:]:
                    if left.state == State.IDLE and right.state == State.IDLE:
                        continue
                    dist = math.hypot(left.x - right.x, left.y - right.y)
                    if dist >= self.safety_stop_distance:
                        continue

                    victim = self._right_of_way_victim(left, right, now)
                    if victim and victim.task:
                        other = (
                            right.aid
                            if victim.aid == left.aid
                            else left.aid
                        )
                        yield_requests.append((
                            victim.aid,
                            other,
                            dist,
                        ))

        for aid, other, dist in yield_requests:
            self._yield_for_right_of_way(aid, other, dist)

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
            agv: AGVState) -> Tuple[Optional[Tuple[float, float]], float]:
        task = agv.task
        if not task:
            return None, 0.0
        if agv.state == State.TO_SHELF:
            return task.pick_xy, task.pick_yaw
        if agv.state == State.TO_AISLE_EXIT:
            return task.aisle_exit_xy, 0.0
        if agv.state == State.TO_STATION:
            return self.station_xy, 0.0
        return agv.current_goal_xy, agv.current_goal_yaw

    def _yield_for_right_of_way(self, aid: str, other: str, dist: float):
        goal_handle = None
        with self.lock:
            agv = self.agvs[aid]
            if not agv.task or agv.state == State.WAITING:
                return
            now = time.time()
            if agv.yield_cooldown_until > now:
                return

            resume_goal_xy, resume_goal_yaw = self._goal_for_state(agv)
            if not resume_goal_xy:
                self._publish_stop(agv.aid)
                return

            goal_handle = agv.current_goal_handle
            old_state = agv.state
            agv.current_goal_handle = None
            agv.state = State.WAITING
            agv.resume_state = old_state
            agv.resume_goal_xy = resume_goal_xy
            agv.resume_goal_yaw = resume_goal_yaw
            agv.wait_until = now + self.yield_hold_duration
            agv.wait_point_xy = (agv.x, agv.y)
            agv.yielding_to = other
            agv.yield_cooldown_until = (
                now + self.yield_hold_duration + self.yield_cooldown_duration)
            agv.nav_goal_sent_ts = 0.0
            agv.nav_goal_accepted_ts = 0.0
            agv.task.status = f"waiting:{other}"

        if goal_handle:
            goal_handle.cancel_goal_async()
        self._publish_stop(aid)
        self.get_logger().warn(
            f"[YIELD] {aid} yields to {other}: "
            f"{dist:.2f}m < {self.safety_stop_distance:.2f}m")

    def _right_of_way_loop(self):
        resumes = []
        now = time.time()
        with self.lock:
            for agv in self.agvs.values():
                if agv.state != State.WAITING or not agv.task:
                    continue
                if now < agv.wait_until:
                    continue

                blocker = self.agvs.get(agv.yielding_to)
                if blocker:
                    dist = math.hypot(agv.x - blocker.x, agv.y - blocker.y)
                    if dist < self.right_of_way_release_distance:
                        self._publish_stop(agv.aid)
                        continue

                if not agv.resume_goal_xy:
                    continue

                task = agv.task
                resume_state = agv.resume_state
                resume_goal_xy = agv.resume_goal_xy
                resume_goal_yaw = agv.resume_goal_yaw
                agv.state = resume_state
                agv.resume_state = State.IDLE
                agv.resume_goal_xy = None
                agv.resume_goal_yaw = 0.0
                agv.wait_until = 0.0
                agv.wait_point_xy = None
                agv.yielding_to = ""
                task.status = "running"
                resumes.append((
                    agv,
                    task,
                    resume_goal_xy,
                    resume_goal_yaw,
                    resume_state,
                ))

        for agv, task, goal_xy, goal_yaw, resume_state in resumes:
            self.get_logger().info(
                f"[RESUME] {agv.aid} resumes {resume_state.value} "
                f"toward ({goal_xy[0]:.1f},{goal_xy[1]:.1f})")
            if not self._send_nav(goal_xy, goal_yaw, task, agv):
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
                if not agv.pending_goal_xy:
                    continue

                task = agv.task
                next_xy = agv.pending_goal_xy
                next_yaw = agv.pending_goal_yaw
                next_label = agv.pending_goal_label or "next goal"
                agv.state = State.TO_AISLE_EXIT
                agv.pause_until = 0.0
                agv.pending_goal_xy = None
                agv.pending_goal_yaw = 0.0
                agv.pending_goal_label = ""
                task.status = "running"
                dispatches.append((agv, task, next_xy, next_yaw, next_label))

        for agv, task, next_xy, next_yaw, next_label in dispatches:
            self.get_logger().info(
                f"[ARRIVE] {agv.aid} finished pickup pause, "
                f"heading to {next_label}")
            if not self._send_nav(next_xy, next_yaw, task, agv):
                self._return_task_to_queue(
                    agv, task, f"{next_label} navigation unavailable")

    def _nav_watchdog(self):
        victims = []
        now = time.time()
        with self.lock:
            for agv in self.agvs.values():
                if agv.state == State.IDLE or not agv.task:
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
                    }
                    for zone, reservation in self.route_reservations.items()
                },
                "fleet": {
                    aid: {
                        "state": agv.state.value,
                        "pos": [round(agv.x, 2), round(agv.y, 2)],
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
                        "wait_point": (
                            [round(agv.wait_point_xy[0], 2),
                             round(agv.wait_point_xy[1], 2)]
                            if agv.wait_point_xy else None
                        ),
                        "resume_goal": (
                            [round(agv.resume_goal_xy[0], 2),
                             round(agv.resume_goal_xy[1], 2)]
                            if agv.resume_goal_xy else None
                        ),
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
