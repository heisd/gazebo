# agv_gazebo

`agv_gazebo` 提供仓储仿真环境，是信息流里的“世界模型”和“任务坐标参考”。调度节点里的货架坐标、出货站坐标、充电坐标都应和本包的 Gazebo world 保持一致。

## 包内容

| 路径 | 作用 |
| --- | --- |
| `worlds/warehouse.world` | 仓库仿真世界，包含地面、墙体、16 个货架、出货站和充电区 |
| `models/` | Gazebo 模型目录，当前只有目录占位 |
| `launch/` | 预留的 Gazebo 单独启动目录，当前没有启动文件 |

## 在信息流中的位置

```text
agv_gazebo/worlds/warehouse.world
        |
        v
Gazebo 仿真环境
        |
        +--> 为 agv_description 生成的 AGV 实体提供碰撞和运动环境
        +--> 为 agv_scheduler 的货架/站点坐标提供空间语义
        +--> 为 SLAM/Nav2 提供地图来源
```

`agv_bringup` 通过 `gazebo --verbose worlds/warehouse.world` 启动本包的世界文件，然后再把 `agv_description` 中的机器人实体放入该世界。

## 仓库坐标

世界文件中定义了 4 排货架，每排 4 个：

| 区域 | 坐标 |
| --- | --- |
| A1-A4 | `(-9, 7)`、`(-5, 7)`、`(-1, 7)`、`(3, 7)` |
| B1-B4 | `(-9, 3)`、`(-5, 3)`、`(-1, 3)`、`(3, 3)` |
| C1-C4 | `(-9, -3)`、`(-5, -3)`、`(-1, -3)`、`(3, -3)` |
| D1-D4 | `(-9, -7)`、`(-5, -7)`、`(-1, -7)`、`(3, -7)` |
| 出货站 | `(9, 0)` |
| 充电区 | `(9, -8)` |

这些坐标已经在 `agv_scheduler` 的 `SHELVES`、`STATION`、`CHARGING` 中复用。修改 world 布局后，需要同步调度节点中的坐标。

## 使用方式

构建并安装世界资源：

```bash
colcon build --symlink-install --packages-select agv_gazebo
source install/setup.zsh
```

完整系统启动：

```bash
ros2 launch agv_bringup agv_full.launch.py
```

仅调试 world 文件时，可以在 source 工作区后直接启动 Gazebo：

```bash
gazebo --verbose install/agv_gazebo/share/agv_gazebo/worlds/warehouse.world \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so
```

## 当前注意点

- 本包只定义静态仓库环境，不发布 ROS topic。
- 传感器数据来自机器人模型中的 Gazebo sensor/plugin；当前 `agv_description` 尚未定义实际激光雷达插件，因此 `agv_gazebo` 单独不能产生 `/agv/scan`。
- 货架碰撞体较窄，后续做 Nav2 costmap 时应给机器人 footprint 和 obstacle inflation 留足安全距离。
