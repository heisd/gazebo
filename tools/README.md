# tools — 离线调度器闭环台架

`closed_loop_sim.py` 是一个**无 ROS 依赖**的闭环仿真台架，用来在没有
Gazebo / Nav2 的环境里直接验证 `agv_scheduler` 的真实调度逻辑。

## 为什么需要它

真正的 ROS 2 + Gazebo + Nav2 闭环依赖 Humble / Gazebo Classic（Ubuntu 22.04）。
在受限或 24.04 环境里跑不起来。这个台架把 ROS 的 plumbing（rclpy、Nav2 action、
TF、消息类型）用桩替换后**导入未经修改的 `scheduler_node.py`**，再配一个简单
运动学机器人 + 假 Nav2 action，驱动完整任务生命周期。

> 决策逻辑（任务分配、分阶段预约、让行、电量/充电、watchdog）跑的是
> `src/agv_scheduler/agv_scheduler/scheduler_node.py` 里的真实代码。

## 运行

```bash
python3 tools/closed_loop_sim.py
```

会跑两组对照实验并打印时间线、每车任务数、并行度（peak active）、makespan：

1. **REFERENCE** — 临时把 `station_lane` 设回 `exclusive`（旧瓶颈），车队被串行化。
2. **FIXED** — 用仓库里提交的当前配置（`station_lane: queue` + 返航去霸占），
   两车真正并行。

## 优化与验证历程（双车 4 任务场景）

| 阶段 | 并行度 | makespan | YIELD 次数 | 说明 |
|---|---|---|---|---|
| 原始（station_lane=exclusive） | 1 | 85s | 0 | 共享车道独占→全队串行 |
| 仅放开车道（queue，未修让行） | 2 | 306s | 47 | 暴露让行活锁，反而更慢 |
| **当前（queue + 返航去霸占）** | **2** | **54s** | **0** | 并行且无活锁，最快 |

根因：出货 dock 是所有任务共享终点；空闲车停在 dock 上 + "有任务车给无任务车
让行"的规则 → 带任务车反复退避形成死锁。修法：送货完成后空闲车返回各自 home
停车位（`RETURNING` 状态），dock 不再被霸占。

## 已验证的结论

- 批量任务分配（`_sched_loop`）生效：能并行时两车在同一调度周期被同时派出。
- `station_lane=queue` + `cell` 预约：放开并行的同时仍能防撞。
- 返航去霸占：消除"空闲车堵死共享 dock"的死锁（对应 `diffweakplan.md` 第 6 点）。
- 单车兼容、6 任务重负载、低电量自动充电场景均无死锁（YIELD=0）。

这个台架是确定性的，可作为后续优化让行 / 路权逻辑时的回归验证工具。

---

# control_panel.py — 双车 Web 控制面板

一个**零依赖、零构建**的 Web 控制台，直接驱动真实的 `AGVScheduler`：通过
调度器新增的算子指令通道 `/agv/agv_command` 下发命令，并订阅
`/agv/scheduler_status` 实时回显。

## 两种后端

| 模式 | 启动 | 用途 |
|---|---|---|
| **SIM**（默认） | `python3 tools/control_panel.py` | 在本机用 `closed_loop_sim.py` 的 ROS 桩 + 运动学世界跑真实调度逻辑，无需 ROS/Gazebo，可直接点按驱动两车 |
| **ROS** | `python3 tools/control_panel.py --ros` | 在装有 ROS 2 的机器人主机上，桥接到正在运行的调度器（发布 `/agv/agv_command`、订阅 `/agv/scheduler_status`） |

常用参数：`--port 8080`（端口）、`--host`（默认 `127.0.0.1` 仅本机；接口无鉴权，
要让局域网访问才显式传 `--host 0.0.0.0`，且只在可信网络里用）、`--rate 2`（SIM
倍速）。打开浏览器访问 `http://<host>:<port>`。活动日志在 SIM 模式来自仿真日志，
在 ROS 模式来自 `/rosout`（过滤 `agv_scheduler` 节点）。

## 面板能力

- **仓库地图**：货架、出货 dock、充电桩、交通区与让行等待点，两车实时位姿、
  目标连线与电量/状态标签；**点击地图即可让选中的车直接开过去**。
- **手动控制**（每车）：`Goto x,y`、`⚡ 充电`、`⌂ 返航回位`、`■ 停车`。
- **派发任务**：选货架 + 优先级 + 指定车（或自动），入队 `task`。
- **冲突测试**：一键制造两车路径冲突，用来检验冲突能否被化解（见下）。
- 实时队列、完成数、活动日志；车辆让行/等待会在车队表里高亮
  （`⤳ yields to ...` / `⏸ reservation`）。

## 冲突测试（检验冲突解决）

面板「Conflict tests」卡片里的按钮会下发一组预设命令，**故意**让两车在共享的
**独占主走廊**里交叉/迎面，从而触发预约 + 路权让行 + 等待点退避的完整链路：

| 场景 | 制造的冲突 | 期望的化解 |
|---|---|---|
| **Cross-corridor tasks** | agv_01→D1(南)、agv_02→A1(北)，两车必须反向穿越同一条独占走廊，并共享出货 dock | 走廊预约把两车串行化、败者退避到等待点；两单都完成，走廊任意时刻只有一辆车 |
| **Head-on goto** | 两条镜像手动移动在走廊里迎面 | 一车占用走廊先过、另一车原地等，过完再续；不撞、不死锁 |
| **Strong conflict (live yield)** | 两车穿越**非独占**的 `station_queue` 并贴近到 <1m（agv_01→(9,-1.5)、agv_02→(9,1.5)） | 触发**实时路权让行**：败者 YIELD 并**退避到安全等待点**再续行；两车都到位 |
| **Reset → home** | — | 两车停车并返回各自 home，便于重跑 |

这些场景纯粹是「一串算子命令」，所以在 SIM 和 ROS 两种模式下行为一致。

> 强冲突场景走的是 cross/headon 之外的另一条链路：不是靠预约提前串行化，而是
> 两车真的逼近到安全距离内，由 `_safety_loop` 触发右行让行 + 等待点退避（正是
> 修复 #2 恢复的行为）。实测时间线：
>
> ```
> t=5.8 [YIELD] agv_02 yields to agv_01: 0.96m < 1.00m
> t=5.8 [WAIT]  agv_02 -> main_corridor:agv_02 for yield (agv_01)   # 退避到等待点
> t=7.8 [RESUME] agv_02 resumes returning toward (9.0,1.5)          # 续行
> final: 两车都到位 (9,-1.5) / (9,1.5)
> ```

## 调度器侧的算子通道（`/agv/agv_command`）

JSON `{"cmd": ..., "agv_id": ...}`：

- `task` → 入队取货任务（等价旧 `/agv/task_request`）；
- `goto` → 让某车直接驶向 `x,y[,yaw]` 并停车（复用 `RETURNING` 单段 +
  分阶段预约，仍受交通控制约束）；
- `charge` / `return` / `stop` → 立即去充电 / 返航 / 取消当前目标并停住。

`goto/charge/return/stop` 会先把目标车 `force-idle`（取消在途目标、清预约），
因此可在任务执行中途打断并立刻响应。

---

# verify_panel.py — 控制面板 + Bug 修复闭环验证

```bash
python3 tools/verify_panel.py     # 13/13 通过则退出码 0
```

用 SIM 后端确定性地推进真实调度器，断言：

1. **面板控制**：`goto / stop / return / charge / task` 真能驱动两车（到点泊车、
   取消停住、返回 home、抵达充电桩进入 charging、4 任务闭环全部完成）。
2. **6 个评审 Bug 已修**：
   - #1 紧急充电会**抢占**他车的路线预约（不再被卡死掉电）；
   - #2 被阻塞的真实任务阶段/让行**退避到等待点**腾出走廊，而内部移动
     （charge/return/manual）仍**原地等**——等待点导航分支重新可达（#3）；
   - #4 内部/外部任务以 `Task.kind` 区分，外部用保留前缀的 tid 被拒绝；
   - #5 `history` 为有界环 + `completed` 独立单调计数（裁剪历史不回退）；
   - #6 仍被阻塞的内部重试会**重置退避**，不再每 tick 空转；
   - #2b 让行败者**永不被永久卡在 WAITING**：所让行的车一旦在释放距离内
     泊车（idle/无在途目标）或等待超过 `yield_max_hold_duration` 上限，败者
     就续行，由 `_safety_loop` 继续做实时防撞；
   - #1 让行败者抵达等待点后释放走廊预约的判定，改用**可标定参数**
     `wait_release_tolerance`（默认 1.0m）而非硬编码常量：抵达即释放、
     远点谎报不释放，阈值收紧能改变判定（证明读的是参数而非常量）；
   - #3 **停滞看门狗**：持预约却对已接受目标在 `nav_stall_timeout`（默认 30s）
     内毫无进展的车（Nav2 卡死但 odom 仍新鲜）被判停滞、重新入队并**释放预约格**，
     不再无限自动续期饿死他车；持续靠近目标的车不会被误判；
   - #4 **电量单一数据源**：外部遥测新鲜时由它独占 `agv.battery`，内部漂移/
     充电模型让位（不再双写打架）；无/过期遥测时回退内部模型（离线 sim 行为不变）；
   - #5 **公平让行**：等优先级路权打平时,**让行次数少的车这次让**,两车轮流而
     非固定让行者(`aid` 仅在让行次数也相等时兜底);优先级仍优先于公平兜底。
3. **冲突解决**：跑 `cross` / `headon` / `strong` 三个冲突场景，断言两车
   **从不同时进入独占走廊**（无碰撞风险）、强冲突下**实时 YIELD 真的触发**
   （贴近到 <安全距离）、且所有目标都达成（无死锁）。

当前结果：**33/33 通过**。
