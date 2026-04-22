# AGV 仓储两车仿真工作区

这个仓库是一个 ROS 2 Humble 仓储 AGV 仿真工作区，包含机器人模型、
Gazebo 仓库世界、Nav2 导航配置和多车任务调度节点。当前重点是两台 AGV
在同一仓库内独立导航，并由调度器把货架任务分配给不同车辆执行。

## 包结构

| 路径 | 作用 |
| --- | --- |
| `src/agv_description` | AGV 机器人模型、TF、轮组和传感器描述 |
| `src/agv_gazebo` | 仓库 Gazebo world、货架、站台和充电区 |
| `src/agv_navigation` | Nav2、地图、定位和两车导航启动入口 |
| `src/agv_scheduler` | 任务队列、车辆分配、避碰和取货停靠等待 |
| `src/agv_bringup` | 单车/两车仿真启动入口 |

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
```

确认 Nav2 已激活：

```bash
ros2 lifecycle get /agv_01/bt_navigator
ros2 lifecycle get /agv_02/bt_navigator
```

正常应返回 `active [3]`。

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

查看调度状态：

```bash
ros2 topic echo --field data /agv/scheduler_status
```

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

单位是秒。车辆到达货架 `pickup` 点后会进入 `PICKING` 状态，等待该时长，
然后继续去通道出口和出货站。

## 两车并行与避碰

两车的 Nav2 action、速度话题、里程计和 TF 都按 namespace 隔离，所以系统支持
两台车同时运动。调度器会按空闲车辆和任务路线进行分配，并通过两层机制降低冲突：

1. 路径区域预约：任务分配前先预约粗粒度路线区域。
2. 路权让行：运行中若两车距离低于 `safety_stop_distance`，调度器会选择一台车
   让行，在当前位置作为临时等待点停车；距离恢复到
   `right_of_way_release_distance` 以上后，再恢复原目标继续任务。

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

让行中的车辆会在 `/agv/scheduler_status` 里显示 `state: "waiting"`、
`yielding_to`、`wait_point` 和 `resume_goal`。

当前调度器仍是简化版多车避碰，不是完整的多智能体路径规划。后续如果要提高并行度，
可以把出货站 `dock:station` 从“任务开始即预约”改成“进入 `TO_STATION` 前再预约”，
这样两台车可以更充分地并行取货，只在进入出货站时排队。

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

## 当前注意点

- 修改 Python 调度代码后，需要重启 `agv_scheduler` 节点；运行中的节点不会热加载源码。
- 发布多个任务时使用不同 `tid`，例如 `T1001`、`T1002`。
- `pickup_pause_duration` 是秒；`waypoint_pause_duration` 是 Nav2 waypoint follower 的毫秒参数，当前调度流程不依赖它。
- 如果 `lifecycle_manager_navigation` 报 `bt_navigator/get_state service client: async_send_request failed`，先检查对应 Nav2 节点是否已经 `active [3]`，以及 `/agv_01/navigate_to_pose`、`/agv_02/navigate_to_pose` action 是否存在。
