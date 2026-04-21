# agv_scheduler

`agv_scheduler` 是仓储 AGV 的任务调度包，负责把上层任务请求转换成车辆分配和导航目标。它是信息流里的决策中心，并在多车模式下做路径区域预约和安全停车。

## 包内容

| 路径 | 作用 |
| --- | --- |
| `agv_scheduler/scheduler_node.py` | 调度节点主实现 |
| `config/two_agv_scheduler.yaml` | 两车调度参数示例 |
| `config/warehouse_layout.yaml` | 货架中心点、取货停靠点、出货站和充电区坐标 |
| `launch/two_agv_scheduler.launch.py` | 两车调度节点启动入口 |
| `setup.py` | 注册 `scheduler_node` 命令 |
| `test/` | Python lint/版权/docstring 测试模板 |

## 在信息流中的位置

```text
/agv/task_request
        |
        v
agv_scheduler
  - 解析任务 JSON
  - 按 priority 排队
  - 从 /agv/odom 更新车辆位置
  - 从 /agv/agv_status 更新车辆状态和电量
  - 选择最近空闲车辆
  - 简单冲突检测
        |
        +--> /agv/task_assigned
        +--> /agv/scheduler_status
        |
        v
navigate_to_pose action
        |
        v
Nav2 / 底盘控制链路
```

默认仍按单车兼容模式启动：`agv_01` 使用 `/agv/odom`、`/agv/cmd_vel` 和全局 `navigate_to_pose` action。两车模式通过参数文件启用，示例中使用：

```text
/agv_01/odom              /agv_02/odom
/agv_01/cmd_vel           /agv_02/cmd_vel
/agv_01/agv_status        /agv_02/agv_status
/agv_01/navigate_to_pose  /agv_02/navigate_to_pose
```

调度节点会自动生成演示任务，也可以手动向 `/agv/task_request` 发布任务。两车配置里默认关闭自动演示任务，便于手动验证避碰。

## 仓库布局配置

调度器从 `config/warehouse_layout.yaml` 读取业务坐标。每个货架有货架中心、
取货停靠点和停靠朝向：

| 字段 | 含义 |
| --- | --- |
| `center` | Gazebo world 中货架模型中心，用于业务记录和显示 |
| `pickup` | AGV 实际导航到的取货停靠点，避开货架碰撞体 |
| `pickup_yaw` | AGV 到达取货点后的车头朝向，单位是弧度，`0.0` 表示朝地图 `+x` 方向 |

例如：

```yaml
A1:
  center: [-9.0, 7.0]
  pickup: [-9.0, 5.4]
  pickup_yaw: 0.0
```

收到 `{"shelf":"A1"}` 后，调度器会记录货架中心 `(-9.0, 7.0)`，但发送给 Nav2 的目标点是停靠点 `(-9.0, 5.4)`。
`pickup_yaw: 0.0` 会让车头按图示方向朝右停好。如果更习惯角度，也可以
写 `pickup_yaw_deg: 90.0`，调度器会自动换算成弧度。

## 订阅接口

| Topic | 类型 | 说明 |
| --- | --- | --- |
| `/agv/task_request` | `std_msgs/msg/String` | JSON 任务请求，例如 `{"tid":"T1001","shelf":"A1","priority":5}` |
| `/agv/odom` 或参数指定的 odom topic | `nav_msgs/msg/Odometry` | 车辆位置、朝向、线速度和角速度 |
| `/agv/agv_status` 或参数指定的 status topic | `std_msgs/msg/String` | JSON 状态上报，例如 `{"agv_id":"agv_01","state":"idle","battery":90}` |

## 发布接口

| Topic / Action | 类型 | 说明 |
| --- | --- | --- |
| `/agv/task_assigned` | `std_msgs/msg/String` | 任务分配结果，包含 AGV、任务号、货架中心、取货停靠点、取货朝向和投放点 |
| `/agv/scheduler_status` | `std_msgs/msg/String` | 调度器周期状态，包含待处理数、完成数、车队状态 |
| `/agv/cmd_vel` 或参数指定的 cmd_vel topic | `geometry_msgs/msg/Twist` | 安全停车时发布零速度 |
| `navigate_to_pose` 或参数指定的 action | `nav2_msgs/action/NavigateToPose` | 向对应车辆的 Nav2 发送目标点 |

## 多车避碰策略

调度器现在包含两层避碰：

1. 路径区域预约：分配任务前，调度器会把车辆当前位置到取货点、取货点到出货站的线路按 `route_cell_size` 划成粗粒度区域。若区域已被其他车辆预约，新任务会等待，不会立刻下发 Nav2 目标。
2. 安全距离停车：运行中周期检查车辆间距。若低于 `safety_stop_distance`，调度器会取消低优先级车辆的当前 Nav2 goal、发布零速度，并把任务重新放回队列。

这不是完整的多智能体路径规划，但适合仓储仿真先解决“同一通道/路口抢路”和“近距离碰撞”问题。后续可以把区域预约替换为 Nav2 的真实路径采样或拓扑地图路权。

## 使用方式

构建并安装：

```bash
colcon build --symlink-install --packages-select agv_scheduler
source install/setup.zsh
```

单独运行调度节点：

```bash
ros2 run agv_scheduler scheduler_node
```

两车调度模式：

```bash
ros2 launch agv_scheduler two_agv_scheduler.launch.py
```

或显式传入参数：

```bash
ros2 run agv_scheduler scheduler_node --ros-args \
  --params-file install/agv_scheduler/share/agv_scheduler/config/two_agv_scheduler.yaml \
  -p shelf_layout_file:=$(pwd)/install/agv_scheduler/share/agv_scheduler/config/warehouse_layout.yaml
```

发布任务：

```bash
ros2 topic pub --once /agv/task_request std_msgs/msg/String \
  "{data: '{\"tid\":\"T1001\",\"shelf\":\"A1\",\"priority\":5}'}"
```

查看分配结果：

```bash
ros2 topic echo /agv/task_assigned
```

查看调度状态：

```bash
ros2 topic echo /agv/scheduler_status
```

模拟车辆状态：

```bash
ros2 topic pub --once /agv/agv_status std_msgs/msg/String \
  "{data: '{\"agv_id\":\"agv_01\",\"state\":\"idle\",\"battery\":88.5}'}"
```

## 任务状态机

| 状态 | 含义 |
| --- | --- |
| `IDLE` | 空闲，允许接任务 |
| `TO_SHELF` | 前往货架 |
| `TO_STATION` | 前往出货站 |
| `TO_CHARGE` | 前往充电区 |
| `CHARGING` | 充电中 |
| `ERROR` | 异常 |

代码中还定义了 `PICKING` 和 `DELIVERING`，当前流程没有显式停留在这两个状态。

## 当前注意点

- 调度器依赖每台车的 `navigate_to_pose` action 服务端。如果某台车 Nav2 没有启动，任务会重新入队。
- `/agv/agv_status` 没有对应发布节点，当前需要外部节点或手动 topic 提供车辆状态。
- 两车模式要求仿真、TF、Nav2、里程计和速度控制已经按车辆命名空间隔离；否则两台车会互相覆盖 topic 或 TF。
- `two_agv_scheduler.launch.py` 会自动传入 `warehouse_layout.yaml`。如果直接 `ros2 run`，需要手动传入 `shelf_layout_file`，否则节点会使用内置兜底布局。
- 调度器内置 15 秒一次的自动演示任务，最多生成 8 个任务。两车配置默认关闭；单独运行节点时要注意它会自动往队列加任务。
