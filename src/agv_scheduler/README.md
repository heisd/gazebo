# agv_scheduler

`agv_scheduler` 是仓储 AGV 的任务调度包，负责把上层任务请求转换成车辆分配和导航目标。它是信息流里的决策中心。

## 包内容

| 路径 | 作用 |
| --- | --- |
| `agv_scheduler/scheduler_node.py` | 调度节点主实现 |
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

当前只初始化一台车：`agv_01`。调度节点会自动生成演示任务，也可以手动向 `/agv/task_request` 发布任务。

## 订阅接口

| Topic | 类型 | 说明 |
| --- | --- | --- |
| `/agv/task_request` | `std_msgs/msg/String` | JSON 任务请求，例如 `{"tid":"T1001","shelf":"A1","priority":5}` |
| `/agv/odom` | `nav_msgs/msg/Odometry` | 车辆位置、朝向、线速度和角速度 |
| `/agv/agv_status` | `std_msgs/msg/String` | JSON 状态上报，例如 `{"agv_id":"agv_01","state":"idle","battery":90}` |

## 发布接口

| Topic / Action | 类型 | 说明 |
| --- | --- | --- |
| `/agv/task_assigned` | `std_msgs/msg/String` | 任务分配结果，包含 AGV、任务号、货架、取货点和投放点 |
| `/agv/scheduler_status` | `std_msgs/msg/String` | 调度器周期状态，包含待处理数、完成数、车队状态 |
| `/agv/cmd_vel` | `geometry_msgs/msg/Twist` | 已创建发布器，当前代码未主动使用，预留给直接速度控制 |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 向 Nav2 发送目标点 |

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

- 调度器依赖 `navigate_to_pose` action 服务端。如果 Nav2 没有启动，节点会记录 `Nav2 未就绪，跳过本次导航`，任务状态可能停在已分配阶段。
- `/agv/agv_status` 没有对应发布节点，当前需要外部节点或手动 topic 提供车辆状态。
- 多车结构已经用字典表示，但当前只注册 `agv_01`。扩展多车时，应同步命名空间、里程计、控制 topic 和冲突检测逻辑。
- 调度器内置 15 秒一次的自动演示任务，最多生成 8 个任务。做手动测试时要注意它会自动往队列加任务。
