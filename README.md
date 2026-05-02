# AGV 仓储两车仿真工作区

这个仓库是一个 ROS 2 Humble 仓储 AGV 仿真工作区，包含机器人模型、Gazebo 仓库世界、Nav2 导航配置和中央调度器。当前重点是让两台 AGV 在共享仓库里独立导航，并按交通规则并行执行货架任务。

## 包结构

| 路径 | 作用 |
| --- | --- |
| `src/agv_description` | AGV 机器人模型、TF、轮组和传感器描述 |
| `src/agv_gazebo` | 仓库 Gazebo world、货架、站台和充电区 |
| `src/agv_navigation` | Nav2、地图、定位和两车导航启动入口 |
| `src/agv_scheduler` | 任务队列、阶段式交通预约、等待点让行和任务恢复 |
| `src/agv_bringup` | 单车/两车仿真启动入口 |
| `src/agv_corridor_layer` | 走廊占用 costmap layer 试验包，当前未并入主流程 |

更细的包级说明在各包自己的 `README.md` 里。

## 构建

```bash
colcon build --symlink-install
source install/setup.zsh
```

只改调度器时可以缩小构建范围：

```bash
colcon build --symlink-install --packages-select agv_scheduler
source install/setup.zsh
```

## 两车启动流程

先启动两车 Gazebo 仿真底座：

```bash
ros2 launch agv_bringup two_agv_sim.launch.py gui:=false rviz:=false
```

再启动两套隔离的 Nav2：

```bash
ros2 launch agv_navigation two_agv_localization_navigation.launch.py \
  map:=$(pwd)/src/agv_navigation/maps/warehouse.yaml
```

最后启动两车调度器：

```bash
ros2 launch agv_scheduler two_agv_scheduler.launch.py
```

两车模式下，关键接口是独立的：

```text
/agv_01/navigate_to_pose    /agv_02/navigate_to_pose
/agv_01/cmd_vel             /agv_02/cmd_vel
/agv_01/odom                /agv_02/odom
/agv_01/scan                /agv_02/scan
/agv_01/agv_status          /agv_02/agv_status
/agv_01_base_footprint      /agv_02_base_footprint
```

默认停车/出生点位于仓库右侧空旷区域：`agv_01` 在 `(10.8, 4.0)`，
`agv_02` 在 `(10.8, -4.0)`，两台车都朝向仓库内部。

确认 Nav2 和 TF 已就绪：

```bash
ros2 lifecycle get /agv_01/bt_navigator
ros2 lifecycle get /agv_02/bt_navigator
ros2 run tf2_ros tf2_echo map agv_01_base_footprint
ros2 run tf2_ros tf2_echo map agv_02_base_footprint
```

正常情况下 `bt_navigator` 应返回 `active [3]`，`tf2_echo` 能持续输出 map 位姿。

## 调度器升级点

当前 `agv_scheduler` 已经做了这几项结构性升级：

1. 共享交通优先使用 `map` 坐标
   - 参数默认 `pose_source: map_then_odom`
   - TF 不可用时再回退到 `/agv_xx/odom`
2. 路线预约改为分阶段
   - `TO_SHELF`
   - `TO_AISLE_EXIT`
   - `TO_STATION`
3. 新增交通区配置
   - `main_corridor`
   - `station_lane`
   - `station_queue`
4. 让行时优先驶向固定等待点，不再默认原地堵在路中央
5. reservation 周期续约，任务完成或异常时主动释放

这意味着系统已经不只是“快撞了才停车”，而是开始在进入冲突区域前做排队和路权控制。

## 发布并行任务

向 `/agv/task_request` 发布任务。每个任务的 `tid` 必须唯一，否则状态里会出现
两台车都显示同一个任务号，调试时很难判断是哪一单。

```bash
ros2 topic pub --once /agv/task_request std_msgs/msg/String \
  "{data: '{\"tid\":\"T1001\",\"shelf\":\"A1\",\"priority\":5}'}"
```

```bash
ros2 topic pub --once /agv/task_request std_msgs/msg/String \
  "{data: '{\"tid\":\"T1002\",\"shelf\":\"B1\",\"priority\":4}'}"
```

如需指定某台车执行任务，加上 `agv_id` 字段：

```bash
ros2 topic pub --once /agv/task_request std_msgs/msg/String \
  "{data: '{\"tid\":\"T1003\",\"shelf\":\"C2\",\"priority\":3,\"agv_id\":\"agv_01\"}'}"
```

电量低于 15% 的车辆不会被派发新任务，调度器会跳过并等待其他空闲车辆。

查看调度状态：

```bash
ros2 topic echo --field data /agv/scheduler_status
```

推荐重点看这些字段：

- `fleet.agv_01.pose_source`
- `fleet.agv_01.reserved_stage`
- `fleet.agv_01.reserved_zones`
- `fleet.agv_01.wait_reason`
- `fleet.agv_01.wait_zone`
- `fleet.agv_01.wait_point`
- `reservations`

如果两台车同时运动，`fleet` 中两台车会各自出现：

```json
"goal_active": true
```

并且 `state` 会类似 `to_shelf`、`picking`、`to_aisle_exit` 或 `to_station`。

## 货架侧停靠等待

当前调度器发送的是 Nav2 `NavigateToPose` action，不是 `FollowWaypoints`。
因此 `src/agv_navigation/config/nav2_params.yaml` 里的
`waypoint_follower.wait_at_waypoint.waypoint_pause_duration` 不控制货架侧等待。

货架侧等待由调度器参数控制：

```yaml
pickup_pause_duration: 4.0
```

配置位置：

```text
src/agv_scheduler/config/two_agv_scheduler.yaml
```

车辆到达货架 `pickup` 点后进入 `PICKING`，等待该时长，再申请下一阶段资源并前往 `aisle_exit`。

## 两车并行与避碰

两车的 Nav2 action、速度话题、里程计和 TF 都按 namespace 隔离，所以系统支持两台车同时运动。调度器会按空闲车辆和任务路线进行分配，并通过两层机制降低冲突：

1. 路径区域预约：每个阶段开始前，调度器会把当前位置到目标点的线路按 `route_cell_size` 划成粗粒度区域。若区域已被其他车辆预约，新阶段会等待，不会立刻下发 Nav2 目标。
2. 路权让行：运行中若两车距离低于 `safety_stop_distance`，调度器会选择一台车让行，优先让它驶向配置好的等待点；距离恢复到 `right_of_way_release_distance` 以上后，再恢复原目标继续任务。

路权选择规则：

- 一方有任务、一方无任务时，有任务的一方让行；若它没有当前 Nav2 目标，则保持停车。
- 一方正在运动、一方已停靠或等待时，运动的一方让行。
- 两方都在执行任务时，低优先级任务让行。
- 优先级相同时，车辆 ID 较大的车让行；两车配置下通常是 `agv_02`。

相关参数在 `src/agv_scheduler/config/two_agv_scheduler.yaml`：

```yaml
safety_stop_distance: 1.0
right_of_way_release_distance: 1.6
yield_hold_duration: 2.0
yield_cooldown_duration: 3.0
```

让行中的车辆会在 `/agv/scheduler_status` 里显示 `state: "waiting"`、`yielding_to`、`wait_reason`、`wait_point` 和 `resume_goal`。

## 交通区与等待点

调度器会从 `src/agv_scheduler/config/warehouse_layout.yaml` 读取交通区：

```yaml
traffic_zones:
  main_corridor:
    type: exclusive
  station_lane:
    type: exclusive
  station_queue:
    type: queue
```

含义：

- `exclusive`: 同一时刻只允许一台车占用
- `queue`: 主要用于排队可视化和等待点选择，不做强互斥

每个关键区都可以配置 `wait_points.agv_01`、`wait_points.agv_02`。当某台车因为阶段预约失败或近距离让行需要退出冲突区时，调度器会优先把它送到这些固定安全点。

当前调度器仍是简化版多车避碰，不是完整的多智能体路径规划。后续如果要提高并行度，可以把路线预约替换为 Nav2 真实 global path 采样或拓扑地图路权。

## 常用检查命令

```bash
ros2 node list
ros2 action list | grep navigate
ros2 topic list | grep agv_
ros2 topic echo --once --field data /agv/scheduler_status
ros2 topic echo --once /agv_01/odom
ros2 topic echo --once /agv_02/odom
```

查看某台车 Nav2 是否激活：

```bash
ros2 lifecycle get /agv_01/controller_server
ros2 lifecycle get /agv_01/bt_navigator
ros2 lifecycle get /agv_02/controller_server
ros2 lifecycle get /agv_02/bt_navigator
```

## 自动演示模式

调度器内置自动演示功能，可在启动后自动随机发出最多 8 个任务，方便快速验证仿真环境是否正常。

默认配置（`two_agv_scheduler.yaml`）已关闭该功能：

```yaml
auto_demo_enabled: false
```

如需开启，改为 `true` 并重启调度器节点。开启后调度器每 15 秒发一个随机货架任务，到第 8 个任务后停止自动发送。

## 当前注意点

- 修改 Python 调度代码后，需要重启 `agv_scheduler` 节点；运行中的节点不会热加载源码。
- 发布多个任务时使用不同 `tid`，例如 `T1001`、`T1002`。
- 电量低于 15% 的车辆不会被派发任务；调度器会跳过该车，等待其他空闲车辆。
- 这版调度器已经是”阶段式交通控制”，但路线采样仍是调度器内部估算（`route_cell_size: 1.5` 米粒度），不是 Nav2 `ComputePathToPose` 的真实 global path。
- `src/agv_corridor_layer` 目前还是试验包；如果后续要把预约路径直接注入 costmap，可以继续把它接入 `agv_navigation`。
- `pickup_pause_duration` 是秒；`waypoint_pause_duration` 是 Nav2 waypoint follower 的毫秒参数，当前调度流程不依赖它。
- 如果 `lifecycle_manager_navigation` 报 `bt_navigator/get_state service client: async_send_request failed`，先检查对应 Nav2 节点是否已经 `active [3]`，以及 `/agv_01/navigate_to_pose`、`/agv_02/navigate_to_pose` action 是否存在。
