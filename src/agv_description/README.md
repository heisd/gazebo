# agv_description

`agv_description` 定义 AGV 机器人本体，是整条信息流里“机器人是什么、怎么被控制、发布什么坐标关系”的源头包。

## 包内容

| 路径 | 作用 |
| --- | --- |
| `urdf/agv_robot.urdf.xacro` | AGV 车体、四轮、雷达外形、惯量、Gazebo 插件和 `ros2_control` 接口 |
| `config/ros2_controllers.yaml` | `joint_state_broadcaster` 和 `diff_drive_controller` 控制器配置 |
| `launch/` | 预留的描述包启动目录，当前没有启动文件 |

## 在信息流中的位置

```text
agv_description
  |
  +--> robot_description
  |      |
  |      +--> robot_state_publisher 发布 TF
  |      +--> gazebo_ros spawn_entity.py 生成仿真实体
  |
  +--> ros2_controllers.yaml
         |
         +--> Gazebo 内的 /controller_manager
         +--> joint_state_broadcaster
         +--> diff_drive_controller
```

这个包不直接处理任务，也不做路径规划。它给 `agv_bringup` 提供 Xacro 文件，由 `robot_state_publisher` 发布坐标树，并由 Gazebo 插件加载控制器配置。

## 关键接口

机器人命名空间在 URDF 的 Gazebo diff drive 插件中设置为 `/agv`：

- 输入：`/agv/cmd_vel`
- 输出：`/agv/odom`
- 输出：`/agv/scan`
- 输出 TF：`odom -> base_footprint -> base_link`
- 控制器管理器：`/controller_manager`

`ros2_control` 中声明的轮关节：

- `front_left_wheel_joint`
- `front_right_wheel_joint`
- `rear_left_wheel_joint`
- `rear_right_wheel_joint`

## 使用方式

单独检查 Xacro 是否能展开：

```bash
ros2 run xacro xacro src/agv_description/urdf/agv_robot.urdf.xacro
```

构建并安装描述资源：

```bash
colcon build --symlink-install --packages-select agv_description
source install/setup.zsh
```

完整系统会通过下面命令间接使用本包：

```bash
ros2 launch agv_bringup agv_full.launch.py
```

## 当前注意点

- `lidar_link` 搭载 Gazebo ray laser，输出 `sensor_msgs/msg/LaserScan` 到 `/agv/scan`，frame 为 `lidar_link`。
- URDF 中同时包含 `gazebo_ros2_control` 和 `gazebo_ros_diff_drive` 相关配置；后续如果只走 `ros2_control` 的 `diff_drive_controller`，应统一控制链路，避免两个底盘插件争用同一车辆运动模型。
- `config/ros2_controllers.yaml` 里的 `wheel_radius` 是 `0.12`，而 Xacro 里的 `wheel_r` 是 `0.10`。如果里程计或速度比例异常，应优先校准这两个参数。
