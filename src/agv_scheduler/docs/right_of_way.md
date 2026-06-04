# AGV 调度器路权（Right-of-Way）设计说明

本文解释 `agv_scheduler/scheduler_node.py` 里**路权 / 让行**逻辑的设计：它要解决
什么问题、和预约系统是什么关系、谁让谁、怎么让、何时恢复，以及为防活锁/防抖
做了哪些约束。所有方法名、字段名都对应源码，便于对照阅读。

---

## 1. 两层防撞模型：预约（主动） + 路权（被动安全网）

调度器对"两车撞上"这件事有**两道独立的防线**：

| 层 | 机制 | 何时介入 | 入口 |
| --- | --- | --- | --- |
| **第 1 层：路线预约**（主动、事前） | 每个执行阶段把途经的 `cell:` / `shelf:` / `dock:` / `traffic:` 资源**互斥预约**下来；拿不到就先等/原地停，**根本不发 Nav2 goal** | 派车、阶段切换之前 | `_reserve_stage_locked` / `_prepare_stage_dispatch_locked` |
| **第 2 层：路权让行**（被动、事后） | 两车**物理距离**逼近到安全阈值时，按优先级让其中一辆退避，腾出空间再续行 | 两车都在动、且 `dist < safety_stop_distance` | `_safety_loop` → `_yield_for_right_of_way` |

**为什么要两层。** 预约层用 2m 栅格做离散预约，能把"路径交叉"在发车前就串行化
（典型如 `cross` / `headon` 冲突场景：两车永远不会同时进独占走廊）。但栅格是离散
的、定位会漂、`queue` 型车道允许多车并存——总有"预约判不冲突、物理却贴得很近"
的残余情况。路权层就是这层**运行时安全网**（典型如 `stress` 强冲突场景：两车在非
独占 `station_queue` 里贴到 0.94m 触发实时让行）。

> 二者互补：绝大多数冲突在第 1 层就被串行化掉了；第 2 层只兜第 1 层漏过的近距离
> 残余冲突。

---

## 2. 相关数据结构与字段

```python
@dataclass
class ConflictLock:
    key: str            # _pair_key(a, b)，与左右顺序无关
    winner_agv: str     # 有路权、继续走的车
    loser_agv: str      # 让行、退避的车
    zones: Set[str]     # 冲突时两车共享的 traffic zone id
    created_at: float
```

`ConflictLock` 是路权逻辑的**记忆**：一旦为某一对车判定了 winner/loser，就把结论存
下来，**避免每个 tick 反复重判、两车互让**（见 §6 防活锁）。

`AGVState` 中与让行相关的字段：

| 字段 | 含义 |
| --- | --- |
| `state == WAITING` | 正在让行/等待预约（让行期间的统一状态） |
| `wait_reason` | `"yield"`（让行）/ `"reservation"`（预约被占）/ `"internal_retry"` |
| `yielding_to` | 让给了哪辆车（winner 的 aid），状态快照里可见 |
| `resume_state` / `resume_goal_xy` / `resume_goal_yaw` | 让行前的目标，解除后据此续行 |
| `wait_point_xy` | 退避到的安全等待点 |
| `wait_until` | 最短保持时间（`yield_hold_duration`）/ 重试退避截止 |
| `yield_cooldown_until` | 让行冷却截止：这之前不会被再次选为 loser，也不会再让 |

定时器（`__init__` 中注册，均 5Hz）：

- `create_timer(0.2, self._safety_loop)` —— 检测逼近、触发让行；
- `create_timer(0.2, self._right_of_way_loop)` —— 判断并恢复已让行的车。

---

## 3. 触发流程：`_safety_loop`（5Hz）

```mermaid
flowchart TD
    Start[每 0.2s] --> Clean[_cleanup_conflict_locks_locked\n清理过期/已分离的锁]
    Clean --> Pairs{遍历每一对车 left,right}
    Pairs -->|两车都 IDLE| Skip1[跳过]
    Pairs --> Dist{dist < safety_stop_distance?<br/>默认 1.0m}
    Dist -->|否| Skip2[跳过]
    Dist -->|是| HasLock{该对已有 ConflictLock?}
    HasLock -->|有| UseLock[沿用既有 winner/loser<br/>不重判]
    HasLock -->|无| Victim[_right_of_way_victim<br/>判定谁让行]
    Victim -->|无 victim 或 victim 无任务| Skip3[跳过]
    Victim -->|有| NewLock[创建 ConflictLock<br/>记住 winner/loser]
    UseLock --> Emit
    NewLock --> Emit[loser 有任务 → 入让行队列]
    Emit --> Exec[_yield_for_right_of_way 逐个执行]
```

要点：

- **先看是否已有锁**：已有就直接沿用既有 loser，不重新判定——这是防止两车互让的
  关键。
- 共享区 `shared_zones = _shared_traffic_zone_ids(left, right)`：取两车"已预约的
  traffic zone"∪"当前所在 traffic zone"的**交集**，用于挑退避等待点。
- 末尾还有一条独立安全兜底：任何"非 IDLE 但已无任务"的车一律 `_publish_stop`。

---

## 4. 谁让行：路权判定优先级（`_right_of_way_victim`）

返回**要让行的那辆车**（victim/loser）。先有两个**否决条件**：

1. 任一车已是 `WAITING` → 返回 `None`（已经在让/在等，不重复处理）；
2. 任一车处于让行冷却 `yield_cooldown_until > now` → 返回 `None`（刚让过，给它喘息，
   避免被反复拉去让行）。

否则按下面的**优先级阶梯**，自上而下第一个命中即决定：

```mermaid
flowchart TD
    A{一方低电量/在去充电,<br/>另一方不是?} -->|是| A1[高电量方让行<br/>低电量/充电车有最高路权]
    A -->|否/都满足| B{一方有任务,<br/>另一方空闲?}
    B -->|是| B1[有任务的车让行<br/>给停着不动的空闲车让路]
    B -->|否/都有| C{一方在移动,<br/>另一方已停?}
    C -->|是| C1[移动中的车让行<br/>更易改道退避]
    C -->|否/都同态| D[_lower_priority_agv<br/>比任务优先级]
    D --> D1[任务优先级低者让行<br/>相等则 aid 较大者让<br/>如 agv_02 让 agv_01]
```

设计意图：

- **低电量/充电优先**（`battery < battery_low_threshold` 或 `TO_CHARGE`）：电量是硬约
  束，快没电的车必须先走，对应 `_emergency_charge` 抢占的同一价值取向。
- **让给"动不了"的车**：空闲车没有目标、停在原地不会主动避让，所以让有任务、能
  重新规划的车去绕。
- **让给已停住的车**：已停的车可能在取货/已占位，移动中的车改道成本更低。
- **最终用确定性的优先级 + aid 平局规则**收敛，保证同一对车每次判出**相同**的
  loser（防抖、可复现）。

---

## 5. 怎么让行：退避到安全等待点（`_yield_for_right_of_way`）

```mermaid
sequenceDiagram
    participant SL as _safety_loop
    participant Y as _yield_for_right_of_way
    participant WS as _set_wait_state_locked
    participant WP as _select_wait_point
    SL->>Y: (loser, winner, dist, zones)
    Y->>Y: 已 WAITING / 冷却中 → 直接返回
    Y->>Y: 记下 resume 目标 = 当前阶段目标
    Y->>WS: reason="yield", use_wait_point=True,<br/>hold_until=now+yield_hold_duration
    WS->>WP: 在冲突 traffic zone 里挑本车等待点
    WP-->>WS: 最近的等待点(或回退到全局/home)
    WS-->>Y: state=WAITING, yielding_to=winner
    Y->>Y: 设 yield_cooldown_until<br/>= now+hold+cooldown
    Y->>Y: 取消旧 Nav2 goal
    Y->>Y: _send_wait_nav_or_stop → 导航去等待点 / 原地停
    Y-->>Y: 打印 [YIELD] a yields to b: 0.96m < 1.00m
```

- **退避点选择**（`_select_wait_point`）：优先选**冲突 traffic zone 内**为本车配置的
  等待点；没有则选任意区里本车的等待点；再没有就用全局 `wait_points`；多个候选取
  **离当前位置最近**的。在出厂布局里 `main_corridor` 给两车各配了等待点
  `(10.8, ±4.0)`。
- **退避 vs 原地停**（`_send_wait_nav_or_stop`）：已在等待点容差内就原地停；否则发一
  个去等待点的 Nav2 goal；发不出去就原地停。
- 让行属于**真实任务阶段**，所以 `use_wait_point=True`（会退避腾道）。这正是修复 #2
  恢复的行为；与之相对，内部移动（charge/return/manual）阻塞时是**原地等**
  （`_wait_should_hold_position`），因为它们的等待点就是目标点本身。
- 设 `yield_cooldown_until = now + yield_hold_duration + yield_cooldown_duration`，让行后
  这段时间内既不会被再选为 loser，也不会再发起让行。

---

## 6. 何时恢复：`_right_of_way_loop`（5Hz）与防活锁

```mermaid
flowchart TD
    L[每 0.2s 遍历 WAITING 且有任务的车] --> G1{有在途 goal / 无 resume 目标?}
    G1 -->|是| Skip[跳过]
    G1 -->|否| Hold{wait_until 未到?}
    Hold -->|是| Skip
    Hold -->|否| Yield{wait_reason == yield?}
    Yield -->|是| Rel{与 winner 距离<br/>< right_of_way_release_distance?<br/>默认 1.6m}
    Rel -->|是,仍太近| Stop[原地停, 继续等]
    Rel -->|否,已分离| Try
    Yield -->|否| Try[_prepare_stage_dispatch_locked<br/>wait_on_block=False 重抢预约]
    Try -->|预约仍被占| Wait2[保持 WAITING<br/>internal_retry 则重置退避]
    Try -->|成功| Resume[清等待字段 + 清本车 ConflictLock<br/>_send_nav 续行, 打印 RESUME]
```

**四重防活锁 / 防抖机制**（核心设计）：

1. **冲突锁记忆**：winner/loser 一旦判定就存进 `conflict_locks`，后续 tick 只沿用、
   不重判——杜绝"A 让 B、B 又让 A"的来回互让。
2. **让行冷却**（`yield_cooldown_until`）：让过的车在 `hold+cooldown` 秒内免于再次被选
   为 loser、也不再让行。
3. **释放距离迟滞**（hysteresis）：触发让行用 `safety_stop_distance`（1.0m），但恢复要
   等到分离超过 `right_of_way_release_distance`（1.6m）。触发阈值 < 恢复阈值，留出
   缓冲带，避免在临界距离反复抖动。
4. **最短保持时间**（`wait_until = now + yield_hold_duration`）：让行后至少保持一段时间
   才考虑恢复，给 winner 通过的时间窗。

**冲突锁清理**（`_cleanup_conflict_locks_locked`，每个 loop 开头跑）——满足任一即删除：
winner IDLE 且 loser 不在 WAITING / 两车已分离到 `≥ release_distance` / 锁过期
（`max(route_hold_timeout, hold+cooldown+5)`）/ 车不存在。

---

## 7. 参数（可调，`_declare_params` 默认值）

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `safety_stop_distance` | `1.0` m | 触发让行的逼近阈值 |
| `right_of_way_release_distance` | `1.6` m | 恢复/清锁的分离阈值（> 触发值形成迟滞） |
| `yield_hold_duration` | `2.0` s | 让行后最短保持时间 |
| `yield_cooldown_duration` | `3.0` s | 在保持时间之上额外的让行冷却 |
| `battery_low_threshold` | `20.0` % | 低于此即获得"低电量最高路权" |

调参直觉：`release > stop` 决定迟滞带宽度；`hold + cooldown` 决定一辆车被反复让行的
最小间隔；调大可更稳但更慢，调小更激进但易抖。

---

## 8. 与状态快照 / 可观测性

`/agv/scheduler_status` 的 `fleet[aid]` 暴露 `state` / `wait_reason` / `yielding_to` /
`wait_point` / `resume_goal`，Web 控制面板据此把让行车高亮成 `⤳ <winner>`、把等待车
高亮成 `⏸ <reason>`，让路权过程肉眼可见。

---

## 9. 关键不变量

- 任一时刻，一对车至多存在一个 `ConflictLock`（`_pair_key` 与顺序无关）。
- 同一独占资源（exclusive `cell:` / `traffic:` / `dock:`）同一时刻至多一个预约持有者
  （第 1 层保证）；路权层不改这条，只处理物理逼近。
- 让行车进入 `WAITING` 后，其 `resume_state/resume_goal_*` 保存了让行前目标，恢复时
  原样续行，不丢任务。
- 让行触发（<1.0m）与恢复（≥1.6m）之间存在迟滞带，系统不会在临界距离震荡。

---

## 10. worked example：强冲突场景实测时间线

`tools/control_panel.py` 的 **Strong conflict (live yield)** 场景把两车送向
`station_queue` 内 `(9,-1.5)` 与 `(9,1.5)`，实测：

```
t=0.0 [MANUAL] agv_02 waiting for reserved route ... via agv_01   # 第1层:预约串行
t=4.0 [RESUME] agv_02 resumes ...
t=5.8 [YIELD]  agv_02 yields to agv_01: 0.96m < 1.00m             # 第2层:实时让行触发
t=5.8 [WAIT]   agv_02 -> main_corridor:agv_02 for yield (agv_01)  # 退避到等待点(§5)
t=7.8 [RESUME] agv_02 resumes returning toward (9.0,1.5)          # 分离后恢复(§6)
final: 两车都到位 (9,-1.5) / (9,1.5)                               # 化解,无死锁无碰撞
```

`tools/verify_panel.py` 对该场景断言：**至少触发一次 `[YIELD]` 且最小间距 <
`safety_stop_distance`**，同时**两车最终都抵达目标**（路权确实化解了冲突）。

---

## 11. 边界与注意事项

- 两车都 `IDLE` 时 `_safety_loop` 直接跳过（停着不算冲突）。
- victim 必须**有任务**才会真正让行（`loser.task` 判空）——纯空闲车不会被赶走。
- 内部移动（charge/return/manual）阻塞时**原地等**而非退避（其等待点即目标，导航过去
  会绕过失败的预约），见 `_wait_should_hold_position`。
- 路权层依赖 pose 的物理距离，因此 pose 质量（`pose_source` / TF 新鲜度）会影响触发
  时机；预约层（第 1 层）不依赖实时 pose，是更可靠的主防线。
