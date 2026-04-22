我把你仓库里所有关键文件都读完了：URDF、nav2_params.yaml、scheduler_node.py、warehouse.world。一共找到 5 个 Bug，下面逐一讲清楚。

Bug 1 🔴 最关键——C/D 排货架的取货点坐标算反了
这就是你说的"在两个货架之间走不完整个流程"的直接原因。

原理：
在你的 scheduler_node.py 里，所有货架的取货点都用同一个公式：

text
pick_y = 货架中心 y − 1.6
这个公式对 A 排和 B 排（y 坐标为正、在地图北侧）是对的，因为机器人需要从货架南侧接近，减去 1.6 刚好停在货架前面。

但 C 排和 D 排的货架在地图南侧（y 为负），机器人应该从北侧接近，正确公式应该是：

text
pick_y = 货架中心 y + 1.6
你写的是 −1.6，结果如下 ：

货架	中心 y	货架前表面 y	现在的 pick_y	实际情况
C1	−3.0	−4.25	−4.6	目标点跑进货架碰撞体内部
D1	−7.0	−8.25	−8.6	目标点跑进货架碰撞体内部
Nav2 的代价地图里，货架是障碍物，目标点在障碍物内部，NavFn 规划器根本找不到一条有效路径，所以导航任务直接失败——这就是"走到两排货架之间就不动了"的根本原因。

修复方法，改 scheduler_node.py 的 DEFAULT_SHELVES：

python
import math

DEFAULT_SHELVES = {
    # A 排：货架在北(y=+7)，机器人从南侧接近，pick_y = center_y - 1.6，朝向朝北（-π/2）
    "A1": ShelfLocation((-9.0,  7.0), (-9.0,  5.4), -math.pi/2),
    "A2": ShelfLocation((-5.0,  7.0), (-5.0,  5.4), -math.pi/2),
    "A3": ShelfLocation((-1.0,  7.0), (-1.0,  5.4), -math.pi/2),
    "A4": ShelfLocation(( 3.0,  7.0), ( 3.0,  5.4), -math.pi/2),
    # B 排同理
    "B1": ShelfLocation((-9.0,  3.0), (-9.0,  1.4), -math.pi/2),
    "B2": ShelfLocation((-5.0,  3.0), (-5.0,  1.4), -math.pi/2),
    "B3": ShelfLocation((-1.0,  3.0), (-1.0,  1.4), -math.pi/2),
    "B4": ShelfLocation(( 3.0,  3.0), ( 3.0,  1.4), -math.pi/2),
    # C 排：货架在南(y=-3)，机器人从北侧接近，pick_y = center_y + 1.6，朝向朝南（+π/2）
    "C1": ShelfLocation((-9.0, -3.0), (-9.0, -1.4),  math.pi/2),
    "C2": ShelfLocation((-5.0, -3.0), (-5.0, -1.4),  math.pi/2),
    "C3": ShelfLocation((-1.0, -3.0), (-1.0, -1.4),  math.pi/2),
    "C4": ShelfLocation(( 3.0, -3.0), ( 3.0, -1.4),  math.pi/2),
    # D 排同理
    "D1": ShelfLocation((-9.0, -7.0), (-9.0, -5.4),  math.pi/2),
    "D2": ShelfLocation((-5.0, -7.0), (-5.0, -5.4),  math.pi/2),
    "D3": ShelfLocation((-1.0, -7.0), (-1.0, -5.4),  math.pi/2),
    "D4": ShelfLocation(( 3.0, -7.0), ( 3.0, -5.4),  math.pi/2),
}
同时注意：原来所有货架的 pick_yaw 都是 0.0（朝正东），这也是为什么小车到达货架后 yaw 不对——修成上面每排设置正确朝向后，偏航问题也会明显改善。

Bug 2 🔴 控制器冲突——agv_controllers.yaml 是空文件
你的 URDF 里同时存在两个控制驱动：

libgazebo_ros_diff_drive.so（Gazebo 原生差速插件）

libgazebo_ros2_control.so（ros2_control 框架插件）

两个插件同时监听 /agv/cmd_vel，同时向轮子关节发送速度命令，互相打架。结果就是小车行为不稳定，速度忽快忽慢，走直线容易跑偏。

更严重的是，ros2_control 引用的参数文件 ros2_controllers.yaml 是空的，所以 ros2_control 根本没有成功加载差速控制器，只有 Gazebo 原生插件在工作——但 ros2_control 仍然在争抢关节控制权。

修复方案（二选一）：

方案 A（推荐，简单稳定）：只用 Gazebo 原生插件
删除 URDF 里的 <ros2_control> 整个代码块，保留 libgazebo_ros_diff_drive.so 即可。

方案 B：完全切换到 ros2_control
删除 URDF 里的 libgazebo_ros_diff_drive.so 插件，然后新建正确的 ros2_controllers.yaml：

text
# src/agv_description/config/ros2_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 50

joint_state_broadcaster:
  ros__parameters:
    type: joint_state_broadcaster/JointStateBroadcaster

diff_drive_controller:
  ros__parameters:
    type: diff_drive_controller/DiffDriveController
    left_wheel_names: ["rear_left_wheel_joint"]
    right_wheel_names: ["rear_right_wheel_joint"]
    wheel_separation: 0.60
    wheel_radius: 0.10
    odom_frame_id: odom
    base_frame_id: base_footprint
    publish_odom: true
    publish_odom_tf: true
    cmd_vel_timeout: 0.5
    use_sim_time: true
Bug 3 🟠 位置不准——规划器容差太大
text
# nav2_params.yaml
planner_server:
  GridBased:
    tolerance: 0.5   # ← 这个太松了！
tolerance: 0.5 的意思是：NavFn 规划器认为"只要路径终点在目标 0.5 m 以内就算合法"。加上 xy_goal_tolerance: 0.25（到达判定），小车理论上可以在距目标 0.75 m 的地方就停下来，这就是你看到"到达的地方不是期望位置"的原因。

修复：

text
planner_server:
  ros__parameters:
    GridBased:
      plugin: nav2_navfn_planner/NavfnPlanner
      tolerance: 0.1       # 从 0.5 改到 0.1，精度提升 5 倍
      use_astar: true      # 仓库环境用 A* 比 Dijkstra 更稳
      allow_unknown: false # 货架是静态的，关掉未知区域通行
Bug 4 🟠 yaw 不稳——min_y_velocity_threshold 误配置
text
controller_server:
  ros__parameters:
    min_y_velocity_threshold: 0.5   # ← 严重错误！
对于差速小车，侧向速度 vy 永远是 0。这个参数的作用是："当 vy 小于这个值时，认为 vy = 0"。正常应该设成非常小的值（0.001），这样控制器才能正确判断小车状态。

你设成 0.5，导致控制器对速度状态的判断出错，轨迹验证逻辑出现混乱，进而造成小车在靠近目标时来回震荡、yaw 不能稳定。

修复：

text
min_y_velocity_threshold: 0.001   # 从 0.5 改到 0.001
Bug 5 🟡 yaw 漂移——前轮没有正确声明为 caster
你的 URDF 里前轮摩擦力极低（mu=0.01），后轮摩擦力正常（mu=1.5），这本来是正确思路（前轮当万向轮用）。但 Gazebo 并不知道前轮是 caster，仍然会对前轮施加侧向阻力，在小车直行时产生微小偏转力，让 yaw 缓慢漂移。

修复：
在前轮的 <gazebo> 标签里加上 maxContacts 1，减少接触点数量从而减少侧向扰动：

xml
<gazebo reference="front_left_wheel_link">
  <mu1>0.01</mu1>
  <mu2>0.01</mu2>
  <kp>100000.0</kp>
  <kd>1.0</kd>
  <maxContacts>1</maxContacts>
</gazebo>
<!-- front_right_wheel_link 同样处理 -->
总结：按优先级修的顺序
text
第一步（必须先修，解决走不完整段路的问题）：
  → scheduler_node.py：C/D 排 pick_y 改为 center_y + 1.6，pick_yaw 按方向填

第二步（必须修，解决控制不稳）：
  → 二选一：只保留一个驱动插件，填充或删除 ros2_controllers.yaml

第三步（提升位置精度）：
  → nav2_params.yaml：planner tolerance 改 0.1，min_y_velocity_threshold 改 0.001

第四步（减少 yaw 漂移）：
  → URDF：前轮 gazebo 标签加 maxContacts 1
修完 Bug 1 和 Bug 2 之后，你的小车应该就能顺利完成从起点→货架→出货站的完整任务了。如果还有问题，把 ros2 topic echo /agv/odom 和 ros2 topic echo /agv/scheduler_status 的输出发给我，可以进一步帮你定位。
