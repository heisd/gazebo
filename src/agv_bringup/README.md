# agv_bringup

`agv_bringup` 是系统总启动包，负责把仿真世界、机器人描述、控制器、SLAM、调度节点和 RViz 按时间顺序拉起来。它是信息流里的编排入口。

## 包内容

| 路径 | 作用 |
| --- | --- |
| `launch/agv_full.launch.py` | 完整 AGV 仓储系统启动文件 |
| `launch/agv_sim.launch.py` | 仅启动仿真底座，用于建图和正式导航前的 Gazebo/机器人/控制器准备 |

## 启动顺序

`agv_full.launch.py` 当前按 `TimerAction` 分阶段启动完整链路：

| 时间 | 组件 | 来源包 |
| --- | --- | --- |
| 0s | Gazebo + `warehouse.world` | `agv_gazebo` |
| 5s | `robot_state_publisher` | `agv_description` |
| 5.5s | `joint_state_publisher` | `agv_description` |
| 7s | `spawn_entity.py` 生成 AGV | `gazebo_ros` + `agv_description` |
| 12s | `joint_state_broadcaster` | `ros2_control` |
| 14s | `diff_drive_controller` | `ros2_control` |
| 16s | `slam_toolbox` | `agv_navigation` 配置入口 |
| 20s | Nav2 navigation stack | `agv_navigation` |
| 24s | `agv_scheduler` | `agv_scheduler` |
| 26s | `rviz2` | RViz |

`agv_sim.launch.py` 只启动底座链路：

| 时间 | 组件 | 来源包 |
| --- | --- | --- |
| 0s | Gazebo + `warehouse.world` | `agv_gazebo` |
| 5s | `robot_state_publisher` | `agv_description` |
| 5.5s | `joint_state_publisher` | `agv_description` |
| 7s | `spawn_entity.py` 生成 AGV | `gazebo_ros` + `agv_description` |
| 12s | `joint_state_broadcaster` | `ros2_control` |
| 14s | `diff_drive_controller` | `ros2_control` |
| 16s | `rviz2` | RViz |

## 在信息流中的位置

```text
agv_bringup
  |
  +--> agv_gazebo: 启动仓库 world
  +--> agv_description: 展开 robot_description 并生成实体
  +--> joint_state_publisher: 发布 wheel continuous joints 的默认 joint_states
  +--> ros2_control: 激活轮式控制器
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
