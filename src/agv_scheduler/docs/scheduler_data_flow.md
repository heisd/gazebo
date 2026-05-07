# AGV 调度器代码分析与数据流图

本文基于 `agv_scheduler/scheduler_node.py` 分析调度器的数据入口、内部数据存储、周期性处理流程、Nav2 交互和状态输出。

## 1. 调度器定位

`AGVScheduler` 是一个 ROS 2 中央调度节点，负责把 `/agv/task_request` 中的上层任务请求转换为：

- 指定 AGV 的任务分配消息；
- 分阶段的交通资源预约；
- Nav2 `NavigateToPose` action goal；
- 安全让行、等待点停靠和恢复调度；
- 电量消耗、低电量返航充电和紧急中断充电；
- `/agv/scheduler_status` 运行状态快照。

核心状态对象包括：

| 数据结构 | 作用 |
| --- | --- |
| `Task` | 任务实体，包含货架、取货点、通道出口、投递点、优先级、指定车辆、重试时间和错误信息。 |
| `AGVState` | 单车运行态，包含 pose、电量、状态机、当前任务、Nav2 goal、等待/恢复信息和预约资源。 |
| `RouteReservation` | 以 cell、货架、dock、traffic zone 为 key 的阶段式互斥资源预约。 |
| `ConflictLock` | 近距离冲突中的 winner/loser 锁，避免两车反复互让。 |
| `TrafficZone` / `WaitPoint` | 仓库交通区和安全等待点配置。 |

## 2. 顶层数据流图

```mermaid
flowchart LR
    Operator[上层系统 / 操作员] -->|String JSON\n/agv/task_request| OnTask[_on_task\n解析任务请求]
    Layout[(warehouse_layout.yaml\n货架/站台/充电区/交通区/等待点)] --> LoadLayout[_load_warehouse_layout]
    Params[(ROS 参数\ntwo_agv_scheduler.yaml)] --> Init[AGVScheduler.__init__]
    LoadLayout --> Stores
    Init --> Stores

    subgraph Stores[调度器内部数据存储]
        Queue[(queue: pending Task)]
        History[(history: completed/running Task)]
        Fleet[(agvs: AGVState by agv_id)]
        Reservations[(route_reservations\nzone -> RouteReservation)]
        Conflicts[(conflict_locks\npair -> ConflictLock)]
    end

    OnTask -->|Task 入队并按 priority 排序| Queue

    Odom[/AGV Odometry\n/agv_XX/odom/] -->|_on_odom| Fleet
    TF[/TF map -> base_frame/] -->|_refresh_map_poses| Fleet
    AgvStatus[/AGV status JSON\n/agv_XX/agv_status/] -->|_on_status| Fleet

    Queue --> Sched[_sched_loop\n选择任务和空闲车]
    Fleet --> Sched
    Reservations --> Sched
    Sched -->|_prepare_stage_dispatch_locked| Reserve[_reserve_stage_locked\n计算并申请阶段资源]
    Reserve --> Reservations
    Sched -->|任务绑定 AGV| Fleet
    Sched -->|_pub_assign| AssignPub[/String JSON\n/agv/task_assigned/]
    Sched -->|_send_nav| Nav2[/Nav2 NavigateToPose\n/agv_XX/navigate_to_pose/]

    Nav2 -->|goal accepted/result| NavCallbacks[_nav_accepted / _nav_done]
    NavCallbacks -->|阶段切换/完成/重入队| Fleet
    NavCallbacks -->|释放或重新申请资源| Reservations
    NavCallbacks --> Queue
    NavCallbacks -->|必要时零速度| CmdVel[/Twist zero\n/agv_XX/cmd_vel/]

    Fleet --> Safety[_safety_loop\n近距离安全检查]
    Reservations --> Safety
    Safety --> Conflicts
    Safety -->|_yield_for_right_of_way| Wait[_set_wait_state_locked\n选择等待点/保存恢复目标]
    Wait --> Fleet
    Wait -->|_send_wait_nav_or_stop| Nav2
    Wait --> CmdVel

    Fleet --> ROW[_right_of_way_loop\n等待释放与恢复]
    Reservations --> ROW
    Conflicts --> ROW
    ROW -->|重新预约成功| Nav2
    ROW --> Reservations
    ROW --> Fleet

    Fleet --> Pause[_pause_loop\n取货停靠结束后调度下一阶段]
    Pause --> Reservations
    Pause --> Nav2

    Fleet --> Battery[_battery_loop\n消耗/充电/低电量处理]
    Battery -->|_send_to_charge / _emergency_charge| Nav2
    Battery -->|紧急中断任务重入队| Queue
    Battery --> Reservations
    Battery --> Fleet

    Fleet --> Watchdog[_nav_watchdog\npose/goal 超时检查]
    Watchdog -->|_return_task_to_queue| Queue
    Watchdog --> Reservations
    Watchdog --> CmdVel

    Fleet --> Status[_pub_status\n生成状态快照]
    Queue --> Status
    Reservations --> Status
    Status --> SchedPub[/String JSON\n/agv/scheduler_status/]
```

## 3. 任务主流程数据流

```mermaid
sequenceDiagram
    participant Up as 上层任务源
    participant Sch as AGVScheduler
    participant Q as queue
    participant F as agvs/AGVState
    participant R as route_reservations
    participant N as Nav2 Action
    participant Pub as ROS 输出话题

    Up->>Sch: /agv/task_request JSON
    Sch->>Sch: _on_task 校验 shelf，补全 pick/aisle_exit/drop
    Sch->>Q: Task 入队，按 priority 降序排序

    loop 每 1s _sched_loop
        Sch->>Q: 取可重试的 pending task
        Sch->>F: 筛选 IDLE 且电量 > 15 的 AGV
        Sch->>F: 按 AGV 到 pick 点距离排序
        Sch->>R: _reserve_stage_locked 预约 TO_SHELF 资源
        alt 预约成功
            Sch->>Q: 从队列移除任务
            Sch->>F: 绑定 task，state=TO_SHELF
            Sch->>Pub: /agv/task_assigned
            Sch->>N: NavigateToPose(pick_xy, pick_yaw)
        else 资源被占用
            Sch->>Q: task.status=waiting:blocker
        end
    end

    N-->>Sch: _nav_done: 到达 pick
    Sch->>F: state=PICKING，task.status=picking
    Sch->>Pub: /agv_XX/cmd_vel 零速度

    loop _pause_loop 等 pickup_pause_duration
        Sch->>R: 预约 TO_AISLE_EXIT
        Sch->>N: NavigateToPose(aisle_exit_xy)
    end

    N-->>Sch: _nav_done: 到达 aisle_exit
    Sch->>R: 预约 TO_STATION，追加 dock:station
    Sch->>N: NavigateToPose(station_xy)

    N-->>Sch: _nav_done: 到达 station
    Sch->>F: 清空 task，state=IDLE
    Sch->>R: 释放该 AGV 的所有预约
    Sch->>Sch: task.status=done，写入 history
```

## 4. 阶段式预约与冲突控制

```mermaid
flowchart TD
    Stage[准备派发某阶段\n_prepare_stage_dispatch_locked] --> Goal[_goal_for_state\n得到目标点和阶段标签]
    Goal --> Zones[_zones_for_stage\n路线采样 + 资源 key 生成]
    Zones --> Route[_route_points_for_stage\nTO_STATION 可走 L 形路线]
    Route --> Sample[_sample_path_points]
    Sample --> PathZones[_path_zones\ncell:x:y + traffic:zone_id]
    Zones --> Extra{阶段类型}
    Extra -->|TO_SHELF / TO_AISLE_EXIT| Shelf[shelf:<id>]
    Extra -->|TO_STATION| Dock[dock:station]
    PathZones --> Reserve[_reserve_stage_locked]
    Shelf --> Reserve
    Dock --> Reserve

    Reserve --> Check{互斥 zone\n是否已被其他 AGV 占用?}
    Check -->|否| Grant[释放本车旧预约\n写入 RouteReservation\n更新 AGV reserved_*]
    Check -->|是| Block[返回 blocker 和 blocked_zones]
    Block --> WaitPolicy{wait_on_block?}
    WaitPolicy -->|false| Pending[任务保持 waiting:blocker]
    WaitPolicy -->|true| Wait[_set_wait_state_locked\n进入 WAITING 并选择等待点]
    Wait --> WaitNav[_send_wait_nav_or_stop\n去等待点或原地停车]
```

资源 key 的来源：

| key 类型 | 生成位置 | 用途 |
| --- | --- | --- |
| `cell:<gx>:<gy>` | 对阶段路线采样后按 `route_cell_size` 栅格化 | 通用路径互斥。 |
| `traffic:<zone_id>` | 采样点落入 `warehouse_layout.yaml` 的 `traffic_zones` 多边形 | 主通道、站台车道等交通区控制。 |
| `shelf:<id>` | `TO_SHELF` 和 `TO_AISLE_EXIT` 阶段追加 | 保护货架取货/离开区域。 |
| `dock:station` | `TO_STATION` 阶段追加 | 保护出货站台。 |

## 5. 让行、等待和恢复数据流

```mermaid
stateDiagram-v2
    [*] --> RUNNING: TO_SHELF / TO_AISLE_EXIT / TO_STATION
    RUNNING --> WAITING: 预约被 blocker 占用\n或 safety distance 冲突
    WAITING --> WAIT_NAV: 有等待点且未到达
    WAITING --> HOLD: 无等待点或已在等待点
    WAIT_NAV --> HOLD: Nav2 到达等待点\n释放旧预约
    HOLD --> WAITING: blocker 未释放\n或距离仍小于 release distance
    HOLD --> RUNNING: _right_of_way_loop\n重新预约当前阶段成功
    RUNNING --> IDLE: TO_STATION 成功完成
    RUNNING --> IDLE: Nav2 失败 / watchdog 超时\n任务重入队
```

关键数据变化：

1. `_safety_loop` 发现两车距离小于 `safety_stop_distance` 后，为 pair 写入 `ConflictLock`，固定 winner/loser。
2. loser 通过 `_yield_for_right_of_way` 取消当前 Nav2 goal，并把当前阶段目标保存到 `resume_goal_xy` / `resume_goal_yaw`。
3. `_set_wait_state_locked` 将 AGV 切到 `WAITING`，写入 `wait_reason`、`wait_zone`、`yielding_to`、`wait_point_xy`。
4. `_send_wait_nav_or_stop` 将车辆导航到安全等待点；若没有等待点，则发布零速度原地等待。
5. `_right_of_way_loop` 在 hold 时间结束、距离满足 `right_of_way_release_distance` 且重新预约成功后，恢复原阶段 Nav2 goal。

## 6. 电量与充电数据流

```mermaid
flowchart TD
    Tick[_battery_loop 每 1s] --> PerAgv[遍历 AGVState]
    PerAgv --> Charging{state == CHARGING?}
    Charging -->|是| Add[按 battery_charge_rate 充电]
    Add --> Full{battery >= battery_full_threshold?}
    Full -->|是| Idle[切回 IDLE\n释放预约]
    Full -->|否| End1[继续充电]

    Charging -->|否| Drain[按速度选择 moving/idle 消耗]
    Drain --> ToCharge{state == TO_CHARGE?}
    ToCharge -->|是| End2[保持前往充电站]
    ToCharge -->|否| Critical{battery < critical?}
    Critical -->|是| Emergency[_emergency_charge\n中断任务并重入队]
    Critical -->|否| Low{battery < low\n且 IDLE?}
    Low -->|是| Normal[_send_to_charge\n创建 CHARGE_<agv_id> 临时任务]
    Low -->|否| End3[无动作]
    Emergency --> Nav[Nav2 -> charging_xy]
    Normal --> Nav
```

## 7. 输出状态快照

`_pub_status` 每 0.5 秒发布一次 `/agv/scheduler_status`，其 JSON 主要包含：

- `pending` / `completed`：队列和已完成任务计数；
- `queue`：待执行任务、指定车辆、重试剩余时间、最近错误；
- `reservations`：每个预约 zone 当前归属 AGV、任务、阶段和过期时间；
- `traffic_zones`：交通区类型和等待点；
- `fleet`：每台 AGV 的状态、位置、pose 来源、电量、速度、任务、当前目标、等待/让行信息和已预约资源。

这使外部监控可以从单个话题复原“任务队列 -> 车辆状态 -> 交通预约 -> Nav2 目标”的完整调度上下文。
