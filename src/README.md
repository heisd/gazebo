# AGV ROS 2 Workspace Packages

`src` 目录包含一个仓储 AGV 仿真系统的 5 个 ROS 2 包。整体目标是把仓库地图、机器人模型、仿真环境、导航入口和调度逻辑串成一条可启动的信息流。

## 包列表

| 包 | 角色 | 信息流位置 |
| --- | --- | --- |
| `agv_description` | 机器人结构、控制器配置 | 定义 AGV 的物理模型、TF、轮组和控制接口，是仿真和控制链路的机器人源数据 |
| `agv_gazebo` | 仓库仿真世界 | 提供货架、墙体、出货站、充电区等环境，是传感、定位和任务坐标的空间来源 |
| `agv_navigation` | 导航/SLAM 配置入口 | 承接仿真传感数据并为 Nav2、SLAM 参数提供安装路径 |
| `agv_scheduler` | 调度决策节点 | 接收任务和车辆状态，选择 AGV，向 Nav2 发送目标点，并发布调度状态 |
| `agv_bringup` | 总启动入口 | 按顺序启动 Gazebo、机器人、控制器、SLAM、调度节点和 RViz |

## 系统信息流

```text
任务请求 /agv/task_request
        |
        v
agv_scheduler
  - 解析 JSON 任务
  - 根据货架坐标和优先级入队
  - 读取 /agv/odom 和 /agv/agv_status
  - 选择空闲 AGV
        |
        +--> /agv/task_assigned
        +--> /agv/scheduler_status
        |
        v
Nav2 action: navigate_to_pose
        |
        v
底盘控制 /agv/cmd_vel
        |
        v
Gazebo + ros2_control / diff drive
        |
        +--> /agv/odom
        +--> TF: odom -> base_footprint -> base_link
        |
        v
SLAM / RViz 可视化
```

当前代码中，`agv_scheduler` 已经创建 `/agv/cmd_vel` 发布器，但主要运动指令由 Nav2 action 目标驱动。`agv_navigation/config/nav2_params.yaml` 当前为空，只作为参数文件入口安装；URDF 中存在 `lidar_link` 外形，但还没有激光传感器插件发布 `/agv/scan`，因此 SLAM 的 `/scan -> /agv/scan` 重映射需要后续补上传感器后才完整。

## 使用方式

在工作区根目录构建：

```bash
colcon build --symlink-install
source install/setup.zsh
```

启动完整仿真链路：

```bash
ros2 launch agv_bringup agv_full.launch.py
```

发布一条任务请求：

```bash
ros2 topic pub --once /agv/task_request std_msgs/msg/String \
  "{data: '{\"tid\":\"T1001\",\"shelf\":\"A1\",\"priority\":5}'}"
```

查看调度状态：

```bash
ros2 topic echo /agv/scheduler_status
```

查看任务分配：

```bash
ros2 topic echo /agv/task_assigned
```

单独运行调度节点：

```bash
ros2 run agv_scheduler scheduler_node
```

## 常见扩展点

- 在 `agv_description/urdf/agv_robot.urdf.xacro` 中增加激光雷达传感器插件，让 `/agv/scan` 成为真实数据流。
- 在 `agv_navigation/config/nav2_params.yaml` 中补齐 Nav2 controller、planner、behavior tree、costmap、AMCL 或 SLAM 参数。
- 在 `agv_bringup/launch/agv_full.launch.py` 中增加 Nav2 bringup 节点，否则 `agv_scheduler` 的 `navigate_to_pose` action 可能找不到服务端。
- 在 `agv_scheduler` 中从单车扩展为多车，并把 `/agv/odom`、`/agv/cmd_vel` 改为按车辆命名空间隔离。
