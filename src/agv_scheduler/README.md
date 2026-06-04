# agv_scheduler

`agv_scheduler` 是仓储 AGV 的任务调度包，负责把上层任务请求转换成车辆分配、阶段式路径预约和 Nav2 目标下发。当前实现重点不是“两个机器人各跑各的”，而是一个更接近 fleet manager 的中央调度器。

## 包内容

| 路径 | 作用 |
| --- | --- |
| `agv_scheduler/scheduler_node.py` | 调度节点主实现 |
| `config/two_agv_scheduler.yaml` | 两车调度参数示例 |
| `config/warehouse_layout.yaml` | 货架、站台、交通区和固定等待点配置 |
| `launch/two_agv_scheduler.launch.py` | 两车调度节点启动入口 |
| `setup.py` | 注册 `scheduler_node` 命令 |
| `test/` | Python lint/版权/docstring 测试模板 |
| `docs/scheduler_data_flow.md` | 调度器代码分析与 Mermaid 数据流图 |

## 当前调度思路

```text
/agv/task_request
        |
        v
agv_scheduler
  - 解析任务 JSON
  - 按 priority 排队
  - 从 TF 优先读取 map -> base_footprint 位置
  - TF 不可用时回退到 odom
  - 每个调度周期为所有空闲车批量分配任务（按距离 + 电量成本择优）
  - 按阶段预约交通资源
  - 冲突时把低优先级车送到固定等待点
  - 向对应 Nav2 action 发送目标
        |
        +--> /agv/task_assigned
        +--> /agv/scheduler_status
        |
        v
/agv_XX/navigate_to_pose
```

和旧版相比，核心变化有七个：

1. 共享交通判断优先使用 `map` 坐标，不再默认直接拿 `/odom` 当全局真值。
2. 预约从“整条任务一次锁死”改成“`TO_SHELF` / `TO_AISLE_EXIT` / `TO_STATION` 分阶段锁定”。
3. `dock:station` 只在进入 `TO_STATION` 前预约，不再一接单就抢占出货位。
4. 让行时优先去配置好的固定等待点，而不是在当前位置原地堵住通道。
5. 任务分配从“每周期只派一台车”改成“每周期批量派完所有空闲车”，并用
   距离 + 电量成本函数 (`_assignment_cost`) 择优，吞吐随车队规模线性提升。
6. `station_lane` 由 `exclusive` 改为 `queue`：每个任务都要穿过这条车道，
   整条独占会把车队串行化；改 `queue` 后由 `cell` 预约防撞，放开并行。
7. 送货完成后返航 home 停车位（`RETURNING`），不再空闲霸占共享出货 dock，
   消除"空闲车堵死 dock"导致的死锁。

> 这些改动均用离线闭环台架 `tools/closed_loop_sim.py` 验证过：双车 4 任务
> 场景下并行度从 1 升到 2、makespan 从 85s 降到 54s、让行震荡从 47 次降到 0。

## 仓库布局配置

调度器从 `config/warehouse_layout.yaml` 读取三类数据：

1. 业务点位：货架 `center` / `pickup` / `pickup_yaw`
2. 站台和充电区：`station.dock`、`charging.center`
3. 交通控制：`traffic_zones`、`wait_points`

当前两车默认停车点也复用这些等待点：`agv_01` 在 `(10.8, 4.0)`，
`agv_02` 在 `(10.8, -4.0)`，用于任务前停放和冲突让行。

示例：

```yaml
traffic_zones:
  main_corridor:
    type: exclusive
    polygon:
      - [-10.5, -1.5]
      - [6.2, -1.5]
      - [6.2, 1.5]
      - [-10.5, 1.5]
    wait_points:
      agv_01: [10.8, 4.0, 3.1416]
      agv_02: [10.8, -4.0, 3.1416]
```

字段说明：

| 字段 | 含义 |
| --- | --- |
| `type: exclusive` | 同一时刻只允许一台车占用该交通区 |
| `type: queue` | 只用于状态可视化和等待点选择，不作为强互斥锁；车道内防撞依赖更细粒度的 `cell` 预约 |
| `polygon` | 交通区多边形，调度器会把当前阶段路线采样点投进去命中区域 |
| `wait_points` | 固定安全等待点，按 `agv_id` 绑定 |
| `wait_points` 顶层节点 | 没命中专属交通区时的兜底等待点 |

## 阶段式预约

任务不再一开始就锁住全部路线。当前分成三个执行阶段：

1. `TO_SHELF`
   - 预约当前位置到货架 `pickup` 的路线
   - 锁住对应 `shelf:<id>` 资源
2. `TO_AISLE_EXIT`
   - 预约货架停靠点到 `aisle_exit` 的路线
   - 继续保护该货架相关区域
3. `TO_STATION`
   - 预约通道出口到 `station.dock` 的路线
   - 此时才追加 `dock:station`

当前 `station.dock` 是 `(8.0, 0.0)`，对应 Gazebo 里的开放式出货 dock 中心车道。

这样做的直接效果是：两台车可以并行去不同货架，只在真正进入共享通道和出货区时排队。

## 位置来源

参数文件里新增了：

```yaml
pose_source: map_then_odom
map_frame: map
base_frames:
  - agv_01_base_footprint
  - agv_02_base_footprint
```

含义：

- 优先从 TF 读取 `map -> agv_xx_base_footprint`
- 如果 TF 暂时不可用或超时，再回退到对应 `/agv_xx/odom`
- `/agv/scheduler_status` 会输出每台车当前使用的 `pose_source`

## 让行和等待点

当前让行逻辑分两层：

1. 主逻辑：区域预约
   - 任务分配前或阶段切换前，如果目标阶段资源被占用，车辆进入 `WAITING`
   - reservation wait 默认原地停止并重试；避免把等待点路径变成未预约的
     隐式导航
2. 最后安全层：近距离让行
   - 当两车距离小于 `safety_stop_distance` 时，仍会触发强制让行
   - 但 winner / loser 会在一次冲突周期内锁定，避免来回震荡

路权选择规则（优先级从高到低）：

1. **低电量/充电优先（最高）**：一方处于 `TO_CHARGE` 或电量 < `battery_low_threshold` 时，另一方让行。
2. 一方有任务、一方无任务 → 有任务的让行。
3. 一方运动、一方停止 → 运动的让行。
4. 两方都有任务 → 低优先级让行。
5. 优先级相同 → agv_id 较大的让行（通常 `agv_02`）。

恢复任务前会重新为当前阶段申请资源。当前默认不再为 reservation/yield
等待发送未预约的 wait-point 导航目标，而是原地保持并等待预约重试。

## 预约保活

旧版预约主要靠：

```yaml
route_hold_timeout: 180.0
```

现在超时只保留为兜底保护。只要任务仍在执行，调度器会周期性刷新 reservation；任务完成、重试或异常时才主动释放。

相关参数：

```yaml
reservation_refresh_interval: 2.0
pose_stale_timeout: 8.0
```

如果位置来源长时间中断，watchdog 会把任务重新入队，而不是静默把路线锁丢掉。

## 订阅接口

| Topic | 类型 | 说明 |
| --- | --- | --- |
| `/agv/task_request` | `std_msgs/msg/String` | JSON 任务请求，例如 `{"tid":"T1001","shelf":"A1","priority":5}` |
| `/agv_01/odom`、`/agv_02/odom` | `nav_msgs/msg/Odometry` | 里程计位置、朝向和速度 |
| `/agv_01/agv_status`、`/agv_02/agv_status` | `std_msgs/msg/String` | JSON 状态上报，例如 `{"agv_id":"agv_01","state":"idle","battery":90}` |

## 发布接口

| Topic / Action | 类型 | 说明 |
| --- | --- | --- |
| `/agv/task_assigned` | `std_msgs/msg/String` | 任务分配结果，包含 AGV、任务号、货架中心、取货停靠点、通道出口点和投放点 |
| `/agv/scheduler_status` | `std_msgs/msg/String` | 调度器状态、车队状态、预约状态、交通区配置 |
| `/agv_01/cmd_vel`、`/agv_02/cmd_vel` | `geometry_msgs/msg/Twist` | 强制停车或等待点停车时发布零速度 |
| `/agv_01/navigate_to_pose`、`/agv_02/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 各车导航目标 |

## 电量消耗与自动充电

调度器内置电量消耗和自动充电模块，每秒运行一次 `_battery_loop`。

**消耗逻辑：**
- 运动中（`|vx| > 0.01` 或 `|wz| > 0.01`）：每秒扣 `battery_drain_moving`%
- 静止时：每秒扣 `battery_drain_idle`%
- 前往充电站途中（`TO_CHARGE`）或等待 `CHARGE_` 内部任务重试时正常消耗，
  不再重复触发新的充电中断

**充电触发（最高优先级）：**

| 条件 | 行为 | 入口方法 |
|---|---|---|
| 电量 < `battery_critical_threshold`（10%），任意状态 | 立刻中断当前任务，任务重新入队，先预约去充电站的路线；若被阻塞则原地等待重试 | `_emergency_charge` |
| 电量 < `battery_low_threshold`（20%），`state == IDLE` | 创建 `CHARGE_` 内部任务，预约充电路线后再发导航 | `_send_to_charge` |

`_emergency_charge` 原子地完成以下操作（持锁）：
1. 取消当前 Nav2 goal handle
2. 将被中断任务置回 `pending`，`retry_after + 10s` 后重新参与调度
3. 创建 `CHARGE_<agv_id>` 内部任务，通过 `_prepare_stage_dispatch_locked()` /
   `_reserve_stage_locked()` 申请充电路线；预约成功后才下发 Nav2 目标，
   被阻塞或 Nav2 暂不可用时进入 `WAITING` 原地重试

充电到达后 `state` 切换为 `CHARGING`，每秒回复 `battery_charge_rate`%，达到 `battery_full_threshold` 后回 `IDLE`。

**参数：**

```yaml
battery_drain_moving: 0.3        # %/秒
battery_drain_idle: 0.02         # %/秒
battery_charge_rate: 1.0         # %/秒
battery_low_threshold: 20.0      # 空闲时触发充电 (%)
battery_critical_threshold: 10.0 # 紧急中断任务充电 (%)
battery_full_threshold: 95.0     # 停止充电 (%)
battery_min_dispatch: 15.0       # 低于此电量不再派发新任务 (%)
```

`battery_min_dispatch` 替换了旧代码里写死的魔法数字 `15`，现在可在
`two_agv_scheduler.yaml` 里配置；`_sched_loop` 只会把电量高于该阈值的空闲车
纳入候选。

充电任务（tid = `CHARGE_<agv_id>`）不进入普通任务队列；`_return_task_to_queue`
识别 `CHARGE_` 前缀并保留内部任务等待重试，不会把它加入普通货架任务队列。

## 停靠等待

当前调度器使用 `NavigateToPose` action，不使用 Nav2 的 `FollowWaypoints`
流程。因此 `nav2_params.yaml` 里的
`waypoint_follower.wait_at_waypoint.waypoint_pause_duration` 不会控制货架侧
等待时间。

货架侧等待由调度器参数 `pickup_pause_duration` 控制，单位是秒。车辆到达
`pickup` 停靠点后会进入 `PICKING` 状态，等待该时长，再申请下一阶段资源并继续去通道出口。

## 使用方式

构建并安装：

```bash
colcon build --symlink-install --packages-select agv_scheduler
source install/setup.zsh
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

查看调度状态：

```bash
ros2 topic echo --field data /agv/scheduler_status
```

重点关注这些字段：

- `pose_source`
- `reserved_stage`
- `reserved_zones`
- `wait_reason`
- `wait_zone`
- `wait_point`

## 任务状态机

| 状态 | 含义 |
| --- | --- |
| `IDLE` | 空闲，允许接任务 |
| `TO_SHELF` | 前往货架停靠点 |
| `PICKING` | 已到货架停靠点，按 `pickup_pause_duration` 等待 |
| `TO_AISLE_EXIT` | 从货架侧退回到通道出口 |
| `TO_STATION` | 前往出货站 |
| `TO_CHARGE` | 前往充电区 |
| `CHARGING` | 充电中 |
| `RETURNING` | 送货完成后返回各自 home 停车位（不滞留在共享出货站）|
| `WAITING` | 资源冲突或安全让行中，必要时前往固定等待点 |
| `ERROR` | 异常 |

代码中还定义了 `DELIVERING`，当前流程没有显式停留在这个状态。

### 送货后返航（避免霸占共享 dock）

`TO_STATION` 完成后，AGV **不会**在出货站 dock 处空闲滞留，而是进入
`RETURNING`，导航回自己的 home 停车位（`warehouse_layout.yaml` 里的
`wait_points.parking_<agv_id>`）再转 `IDLE`。

原因：出货 dock 是所有任务的共享终点。如果一台车送完货空闲停在 dock 上，
另一台带任务的车永远到不了 dock，而"有任务车给无任务车让行"的路权规则会让
带任务车反复退避——形成**死锁/让行震荡**。返航后 home 停车位彼此独立且不在
任何交通区内，从根本上消除了这种争用。

返航任务 `tid = RETURN_<agv_id>`，`priority = 0`：

- 不进入普通任务队列，`_return_task_to_queue` 识别 `RETURN_` 前缀并保留内部
  任务等待重试，不会加入普通货架任务队列；
- 和普通阶段一样先通过 `_prepare_stage_dispatch_locked()` /
  `_reserve_stage_locked()` 申请返航路径，确保 `reserved_zones` 与
  `route_reservations` 覆盖离开 dock、穿过 `station_lane`、回到 home 的
  整段路线；若返航路径被预约阻塞或 Nav2 失败，AGV 原地等待并重试，
  不会把 home/parking wait point 当成未预约的等待导航目标，也不会把失败的
  返航直接当作已到家处理；
- 现有路权规则会让低优先级（0）的返航车给真实任务让行，无需改安全逻辑；
- `RETURNING` 车（以及等待返航预约的 `RETURN_` 内部任务）仍可被调度：若队列有待办任务，
  `_sched_loop` 会中断返航直接接新任务（取消 home/等待目标），避免繁忙时浪费返航行程。

## 当前边界

这版已经从“任务调度 + 紧急停车”升级到“任务调度 + 交通区预约 + 固定等待点 + 预约续约”，但还没做到完整的 fleet manager：

- 当前阶段路线仍是调度器自己的采样路径，不是 Nav2 `ComputePathToPose` 返回的真实 global path。
- `traffic_zones` 目前只支持简单 polygon 命中和 `exclusive` / `queue` 两类。
- 调度器还在一个 Python 文件里，后续继续拆成 `TaskManager`、`TrafficManager`、`Nav2Commander` 会更稳。
