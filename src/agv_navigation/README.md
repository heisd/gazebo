# agv_navigation

`agv_navigation` 是导航配置包，负责承载 Nav2、SLAM、地图和导航启动资源。当前采用在线 SLAM 模式：Gazebo 虚拟雷达发布 `/agv/scan`，`slam_toolbox` 生成 `map -> odom`，Nav2 读取地图、雷达和里程计后输出速度到 `/agv/cmd_vel`。

## 包内容

| 路径 | 作用 |
| --- | --- |
| `config/nav2_params.yaml` | Nav2 参数，包含 BT Navigator、controller、planner、costmap、velocity smoother 等配置 |
| `maps/` | 地图文件目录，当前没有地图文件 |
| `launch/mapping.launch.py` | 在线建图入口，启动 `slam_toolbox` 和 `map_saver_server` |
| `launch/navigation.launch.py` | 仅启动 Nav2 navigation stack，适合在线 SLAM 同时运行时调试 |
| `launch/localization_navigation.launch.py` | 正式导航入口，加载保存地图，启动 AMCL + Nav2 |
| `launch/two_agv_localization_navigation.launch.py` | 两车导航入口，为 `agv_01`、`agv_02` 各启动一套带 frame 前缀的 AMCL + Nav2 |

## 在信息流中的位置

```text
Gazebo /agv/scan
        |
        v
slam_toolbox
        |
        +--> map / odom 相关定位和建图信息
        |
        v
Nav2
        |
        +--> navigate_to_pose action
        |
        v
agv_scheduler 发送目标点
```

推荐使用两阶段流程：

```text
阶段 1: 建图
agv_bringup/agv_sim.launch.py
        |
        +--> Gazebo + robot_state_publisher + controller + /agv/scan + /agv/odom
        |
agv_navigation/mapping.launch.py
        |
        +--> slam_toolbox 发布 /map 和 map -> odom
        +--> map_saver_server 保存 warehouse.yaml / warehouse.pgm

阶段 2: 正式导航
agv_bringup/agv_sim.launch.py
        |
        +--> Gazebo + 机器人底座 + /agv/scan + /agv/odom
        |
agv_navigation/localization_navigation.launch.py
        |
        +--> map_server 加载 warehouse.yaml
        +--> AMCL 使用 /agv/scan 定位
        +--> Nav2 接收 /navigate_to_pose 并输出 /agv/cmd_vel
```

## 使用方式

构建并安装导航资源：

```bash
colcon build --symlink-install --packages-select agv_navigation
source install/setup.zsh
```

### 阶段 1: 建图

先启动仿真底座：

```bash
ros2 launch agv_bringup agv_sim.launch.py
```

再启动建图：

```bash
ros2 launch agv_navigation mapping.launch.py
```

用键盘控制小车慢慢跑完整个仓库：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/agv/cmd_vel
```

检查地图是否生成：

```bash
ros2 topic echo --once /map
ros2 run tf2_ros tf2_echo map base_footprint
```

保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f src/agv_navigation/maps/warehouse
```

保存后会得到：

```text
src/agv_navigation/maps/warehouse.yaml
src/agv_navigation/maps/warehouse.pgm
```

### 阶段 2: 正式导航

重新启动仿真底座：

```bash
ros2 launch agv_bringup agv_sim.launch.py
```

启动定位和导航：

```bash
ros2 launch agv_navigation localization_navigation.launch.py
```

如果地图不在默认路径，可以显式传入：

```bash
ros2 launch agv_navigation localization_navigation.launch.py \
  map:=$(pwd)/src/agv_navigation/maps/warehouse.yaml
```

检查 Nav2 action 和定位：

```bash
ros2 action list | grep navigate
ros2 run tf2_ros tf2_echo map base_footprint
```

在 RViz 中使用 `Nav2 Goal` 发布目标点，或者用命令行发送目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### 两车导航

先启动两车仿真底座：

```bash
ros2 launch agv_bringup two_agv_sim.launch.py gui:=false rviz:=false
```

再启动两车定位和导航：

```bash
ros2 launch agv_navigation two_agv_localization_navigation.launch.py \
  map:=$(pwd)/src/agv_navigation/maps/warehouse.yaml
```

这个启动文件会从单车 `nav2_params.yaml` 自动生成两份运行时参数，把 Nav2 的 frame 和话题改成：

```text
agv_01_odom / agv_01_base_footprint / /agv_01/scan / /agv_01/cmd_vel
agv_02_odom / agv_02_base_footprint / /agv_02/scan / /agv_02/cmd_vel
```

对应 action server 为：

```text
/agv_01/navigate_to_pose
/agv_02/navigate_to_pose
```

检查两车定位 TF：

```bash
ros2 run tf2_ros tf2_echo map agv_01_base_footprint
ros2 run tf2_ros tf2_echo map agv_02_base_footprint
```

`map` frame 由 AMCL 发布。AMCL 需要先完成 lifecycle 激活并收到第一帧激光，因此 `tf2_echo` 刚启动时可能先打印 `Invalid frame ID "map"`。如果随后能持续输出矩阵，TF 链路就是可用的。

检查配置是否已安装：

```bash
ros2 pkg prefix agv_navigation
```

## 与其他包的关系

- 从 `agv_gazebo` 接收仓库环境中的仿真空间语义。
- 依赖 `agv_description` 中 `lidar_link` 的 Gazebo ray laser 提供 `/agv/scan` 传感器数据。
- 为 `agv_scheduler` 的 `navigate_to_pose` action 提供导航服务端。
- 由 `agv_bringup` 统一启动。

## 当前待补齐项

- `maps/` 目录初始为空。第一次建图后需要保存 `warehouse.yaml` 和 `warehouse.pgm`。
- `config/nav2_params.yaml` 是初始可用参数，速度、膨胀半径、footprint、DWB critics 需要结合仿真效果继续调参。
