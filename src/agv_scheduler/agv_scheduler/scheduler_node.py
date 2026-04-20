#!/usr/bin/env python3
"""
AGV 仓储自动调度节点
复现截图中：rostopic pub 发布控制指令 + 控制器响应的完整流程
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
import json, time, math, threading
from enum import Enum
from dataclasses import dataclass, field
from typing import Optional, List


class State(Enum):
    IDLE       = "idle"
    TO_SHELF   = "to_shelf"
    PICKING    = "picking"
    TO_STATION = "to_station"
    DELIVERING = "delivering"
    TO_CHARGE  = "to_charge"
    CHARGING   = "charging"
    ERROR      = "error"


@dataclass
class Task:
    tid:      str
    shelf:    str
    pick_xy:  tuple
    drop_xy:  tuple
    priority: int = 1
    ts:       float = field(default_factory=time.time)
    agv:      str = ""
    status:   str = "pending"
    def __lt__(self, o): return self.priority > o.priority


@dataclass
class AGVState:
    aid:     str
    x:       float = 0.0
    y:       float = 0.0
    yaw:     float = 0.0
    state:   State = State.IDLE
    battery: float = 100.0
    task:    Optional[Task] = None
    vx:      float = 0.0
    wz:      float = 0.0


class AGVScheduler(Node):

    SHELVES = {
        "A1": (-9.0,  7.0), "A2": (-5.0,  7.0),
        "A3": (-1.0,  7.0), "A4": ( 3.0,  7.0),
        "B1": (-9.0,  3.0), "B2": (-5.0,  3.0),
        "B3": (-1.0,  3.0), "B4": ( 3.0,  3.0),
        "C1": (-9.0, -3.0), "C2": (-5.0, -3.0),
        "C3": (-1.0, -3.0), "C4": ( 3.0, -3.0),
        "D1": (-9.0, -7.0), "D2": (-5.0, -7.0),
        "D3": (-1.0, -7.0), "D4": ( 3.0, -7.0),
    }
    STATION  = (9.0,  0.0)
    CHARGING = (9.0, -8.0)

    def __init__(self):
        super().__init__('agv_scheduler')
        self.get_logger().info('=' * 50)
        self.get_logger().info(' AGV 仓储调度系统 v2.0 启动')
        self.get_logger().info('=' * 50)

        self.agvs: dict[str, AGVState] = {
            "agv_01": AGVState("agv_01", x=0.0, y=0.0),
        }
        self.queue: List[Task] = []
        self.history: List[Task] = []
        self.lock = threading.Lock()
        self._task_cnt = 0

        # ── 订阅 ──
        self.create_subscription(
            String, '/agv/task_request', self._on_task, 10)
        self.create_subscription(
            Odometry, '/agv/odom', self._on_odom, 10)
        self.create_subscription(
            String, '/agv/agv_status', self._on_status, 10)

        # ── 发布 ──
        self.pub_assign = self.create_publisher(String, '/agv/task_assigned', 10)
        self.pub_sched  = self.create_publisher(String, '/agv/scheduler_status', 10)
        self.pub_vel    = self.create_publisher(Twist,  '/agv/cmd_vel', 10)

        # ── Nav2 Action ──
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── 定时器 ──
        self.create_timer(1.0,  self._sched_loop)
        self.create_timer(0.5,  self._pub_status)
        self.create_timer(15.0, self._auto_demo)

        self._demo_n = 0
        self.get_logger().info('调度系统就绪，等待任务...')
        self.get_logger().info(
            f'货架节点: {len(self.SHELVES)} 个 | AGV数量: {len(self.agvs)} 台')

    # ── 里程计回调 ──────────────────────────────
    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose
        q = p.orientation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny, cosy)
        with self.lock:
            if "agv_01" in self.agvs:
                a = self.agvs["agv_01"]
                a.x = round(p.position.x, 3)
                a.y = round(p.position.y, 3)
                a.yaw = round(yaw, 3)
                a.vx  = round(msg.twist.twist.linear.x,  3)
                a.wz  = round(msg.twist.twist.angular.z, 3)

    # ── 状态回调 ──────────────────────────────
    def _on_status(self, msg: String):
        try:
            d = json.loads(msg.data)
            aid = d.get("agv_id")
            if aid and aid in self.agvs:
                with self.lock:
                    a = self.agvs[aid]
                    a.state   = State(d.get("state", "idle"))
                    a.battery = float(d.get("battery", a.battery))
        except Exception as e:
            self.get_logger().warn(f'状态解析错误: {e}')

    # ── 任务请求回调 ──────────────────────────
    def _on_task(self, msg: String):
        try:
            d = json.loads(msg.data)
            shelf = d.get("shelf", "A1")
            if shelf not in self.SHELVES:
                self.get_logger().warn(f'未知货架: {shelf}')
                return
            self._task_cnt += 1
            task = Task(
                tid=d.get("tid", f"T{self._task_cnt:04d}"),
                shelf=shelf,
                pick_xy=self.SHELVES[shelf],
                drop_xy=self.STATION,
                priority=int(d.get("priority", 1)),
            )
            with self.lock:
                self.queue.append(task)
                self.queue.sort()
            self.get_logger().info(
                f'[入队] {task.tid} 货架={shelf} 优先级={task.priority} '
                f'队列={len(self.queue)}')
        except Exception as e:
            self.get_logger().error(f'任务解析失败: {e}')

    # ── 主调度循环 ────────────────────────────
    def _sched_loop(self):
        with self.lock:
            if not self.queue:
                return
            idle = [a for a in self.agvs.values()
                    if a.state == State.IDLE and a.battery > 15]
            if not idle:
                return
            task = self.queue.pop(0)
            agv = self._nearest(idle, task.pick_xy)
            if self._conflict(agv, task):
                self.queue.insert(0, task)
                self.get_logger().warn(
                    f'[冲突] {task.tid} 路径冲突,重新排队')
                return
            task.agv    = agv.aid
            task.status = "running"
            agv.state   = State.TO_SHELF
            agv.task    = task
            self.get_logger().info(
                f'[分配] {task.tid} → {agv.aid} 前往 {task.shelf} '
                f'({task.pick_xy[0]:.1f},{task.pick_xy[1]:.1f})')
            self._pub_assign(agv, task)
            self._send_nav(task.pick_xy, task, agv)
            self.history.append(task)

    # ── 最近邻选车 ──────────────────────────
    def _nearest(self, agvs, xy):
        return min(agvs, key=lambda a: math.hypot(a.x-xy[0], a.y-xy[1]))

    # ── 简单冲突检测 ─────────────────────────
    def _conflict(self, agv, task, r=1.5):
        for a in self.agvs.values():
            if a.aid == agv.aid or a.state == State.IDLE:
                continue
            if math.hypot(a.x-task.pick_xy[0], a.y-task.pick_xy[1]) < r:
                return True
        return False

    # ── 发送 Nav2 目标 ──────────────────────
    def _send_nav(self, xy, task, agv):
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 未就绪，跳过本次导航')
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(xy[0])
        goal.pose.pose.position.y = float(xy[1])
        goal.pose.pose.orientation.w = 1.0
        fut = self.nav.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._nav_accepted(f, task, agv))
        self.get_logger().info(
            f'[Nav2] 发送目标 ({xy[0]:.1f},{xy[1]:.1f})')

    def _nav_accepted(self, fut, task, agv):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn(f'[Nav2] 目标拒绝: {task.tid}')
            with self.lock: agv.state=State.IDLE; agv.task=None
            return
        gh.get_result_async().add_done_callback(
            lambda f: self._nav_done(f, task, agv))

    def _nav_done(self, fut, task, agv):
        with self.lock:
            if agv.state == State.TO_SHELF:
                agv.state = State.TO_STATION
                self.get_logger().info(
                    f'[到达] {agv.aid} 到达货架 {task.shelf}，前往出货站')
                self._send_nav(self.STATION, task, agv)
            elif agv.state == State.TO_STATION:
                agv.state = State.IDLE
                agv.task  = None
                task.status = "done"
                self.get_logger().info(
                    f'[完成] {task.tid} 任务完成！')
            elif agv.state == State.TO_CHARGE:
                agv.state = State.CHARGING
                self.get_logger().info(f'[充电] {agv.aid} 开始充电')

    # ── 发布分配消息 ─────────────────────────
    def _pub_assign(self, agv, task):
        m = String()
        m.data = json.dumps({
            "agv": agv.aid, "tid": task.tid,
            "shelf": task.shelf,
            "pick": list(task.pick_xy),
            "drop": list(task.drop_xy),
            "priority": task.priority,
        }, ensure_ascii=False)
        self.pub_assign.publish(m)

    # ── 发布调度状态 ─────────────────────────
    def _pub_status(self):
        done = sum(1 for t in self.history if t.status == "done")
        with self.lock:
            payload = {
                "pending": len(self.queue),
                "completed": done,
                "fleet": {
                    aid: {
                        "state":   a.state.value,
                        "pos":     [round(a.x,2), round(a.y,2)],
                        "battery": round(a.battery, 1),
                        "vx":      a.vx, "wz": a.wz,
                        "task":    a.task.tid if a.task else None,
                    }
                    for aid, a in self.agvs.items()
                }
            }
        m = String(); m.data = json.dumps(payload, ensure_ascii=False)
        self.pub_sched.publish(m)

    # ── 自动演示任务 ─────────────────────────
    def _auto_demo(self):
        if self._demo_n >= 8: return
        import random
        shelf = random.choice(list(self.SHELVES.keys()))
        prio  = random.randint(1, 5)
        m = String()
        m.data = json.dumps({
            "tid": f"AUTO_{self._demo_n+1:03d}",
            "shelf": shelf, "priority": prio
        }, ensure_ascii=False)
        self._on_task(m)
        self._demo_n += 1


def main(args=None):
    rclpy.init(args=args)
    node = AGVScheduler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n调度系统关闭')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
