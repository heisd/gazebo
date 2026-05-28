# agv_bringup

`agv_bringup` 是系统总启动包，负责把仿真世界、机器人描述、Gazebo 底盘插件、SLAM、调度节点和 RViz 按时间顺序拉起来。它是信息流里的编排入口。

## 包内容

| 路径 | 作用 |
| --- | --- |
| `launch/agv_full.launch.py` | 完整 AGV 仓储系统启动文件 |
| `launch/agv_sim.launch.py` | 仅启动仿真底座，用于建图和正式导航前的 Gazebo/机器人准备 |
| `launch/two_agv_sim.launch.py` | 启动两台相同 AGV 模型，并用不同 namespace/topic 区分 |

## 启动顺序

`agv_full.launch.py` 当前按 `TimerAction` 分阶段启动完整链路：

| 时间 | 组件 | 来源包 |
| --- | --- | --- |
| 0s | Gazebo + `warehouse.world` | `agv_gazebo` |
| 5s | `robot_state_publisher` | `agv_description` |
| 5.5s | `joint_state_publisher` | `agv_description` |
| 7s | `spawn_entity.py` 生成 AGV | `gazebo_ros` + `agv_description` |
| 12s | `slam_toolbox` | `agv_navigation` 配置入口 |
| 16s | Nav2 navigation stack | `agv_navigation` |
| 20s | `agv_scheduler` | `agv_scheduler` |
| 22s | `rviz2` | RViz |

`agv_sim.launch.py` 只启动底座链路：

| 时间 | 组件 | 来源包 |
| --- | --- | --- |
| 0s | Gazebo + `warehouse.world` | `agv_gazebo` |
| 5s | `robot_state_publisher` | `agv_description` |
| 5.5s | `joint_state_publisher` | `agv_description` |
| 7s | `spawn_entity.py` 生成 AGV | `gazebo_ros` + `agv_description` |
| 12s | `rviz2` | RViz |

`two_agv_sim.launch.py` 启动两台相同模型的 AGV：

| 时间 | 组件 | 说明 |
| --- | --- | --- |
| 0s | `gzserver` + `warehouse.world` | 仓库仿真世界 |
| 0s | `gzclient` | 默认启动，可用 `gui:=false` 关闭 |
| 0s | 两个 `robot_state_publisher` | 分别在 `/agv_01` 和 `/agv_02` 下运行 |
| 0s | 两个 `joint_state_publisher` | 分别发布两台车的轮关节状态 |
| 3s | `spawn_entity.py` 生成两台 AGV | 实体名分别为 `agv_01`、`agv_02` |
| 10s | `rviz2` | 默认启动，可用 `rviz:=false` 关闭 |

单车和两车仿真模式都使用 `gazebo_ros_diff_drive` 插件直接处理底盘速度和里程计，不启动 `ros2_control` spawner，避免控制链路争用同一车辆运动模型。

## 在信息流中的位置

```text
agv_bringup
  |
  +--> agv_gazebo: 启动仓库 world
  +--> agv_description: 展开 robot_description 并生成实体
  +--> joint_state_publisher: 发布 wheel continuous joints 的默认 joint_states
  +--> gazebo_ros_diff_drive: 处理 cmd_vel、odom 和 odom TF
  +--> agv_navigation: 启动 SLAM 和 Nav2 导航栈
  +--> agv_scheduler: 启动任务调度
  +--> RViz: 可视化
```

这个包本身不发布业务 topic，而是决定其他包何时进入信息流。

## 使用方式

构建全部包：

```bash
colcon build --symlink-install
source install/setup.zsh
```

启动完整系统：

```bash
ros2 launch agv_bringup agv_full.launch.py
```

仅启动仿真底座，供建图或正式导航复用：

```bash
ros2 launch agv_bringup agv_sim.launch.py
```

启动两车仿真底座：

```bash
ros2 launch agv_bringup two_agv_sim.launch.py
```

无桌面或只需要后台仿真时关闭 Gazebo GUI 和 RViz：

```bash
ros2 launch agv_bringup two_agv_sim.launch.py gui:=false rviz:=false
```

启动后检查两车话题：

```bash
ros2 topic list | grep agv_
```

应看到类似：

```text
/agv_01/cmd_vel
/agv_01/odom
/agv_01/scan
/agv_02/cmd_vel
/agv_02/odom
/agv_02/scan
```

检查两车 TF：

```bash
ros2 run tf2_ros tf2_echo agv_01_odom agv_01_base_footprint
ros2 run tf2_ros tf2_echo agv_02_odom agv_02_base_footprint
```

`tf2_echo` 在 Gazebo 实体生成前可能先打印 `Invalid frame ID`，这是 TF buffer 等待首帧变换的正常现象；若 10 秒后仍持续出现，先确认 `ros2 node list` 中还有 `/agv_01/robot_state_publisher`、`/agv_02/robot_state_publisher` 和 Gazebo diff drive 插件节点。

显式传入仿真时间参数：

```bash
ros2 launch agv_bringup agv_full.launch.py use_sim_time:=true
```

启动后发布测试任务：

```bash
ros2 topic pub --once /agv/task_request std_msgs/msg/String \
  "{data: '{\"tid\":\"T1001\",\"shelf\":\"B2\",\"priority\":3}'}"
```

观察系统状态：

```bash
ros2 topic echo /agv/scheduler_status
ros2 topic echo /agv/odom
```

## 当前注意点

- 推荐建图和正式导航时使用 `agv_sim.launch.py`，再分别启动 `agv_navigation mapping.launch.py` 或 `localization_navigation.launch.py`，避免调度器自动任务干扰手动 Nav2 Goal。
- 启动文件通过 `subprocess.run(['xacro', urdf])` 展开机器人描述。如果环境里没有 `xacro` 命令，`robot_description` 会为空。
- 当前会先启动 `slam_toolbox`，再启动 Nav2 navigation stack；`navigate_to_pose` action 由 Nav2 提供。
- URDF 中的 `lidar_link` 已包含 Gazebo ray laser 插件，`slam_toolbox` 通过 `/scan -> /agv/scan` 重映射读取雷达数据。
- 控制器加载依赖 Gazebo 内部 `/controller_manager` 初始化完成，因此启动文件用定时延迟。机器较慢时可适当增加 12s 和 14s 两个加载延迟。
- 两车模式只完成仿真底座和话题/TF 隔离；Nav2 多车导航还需要为 `/agv_01/navigate_to_pose` 和 `/agv_02/navigate_to_pose` 各启动一套 Nav2。
