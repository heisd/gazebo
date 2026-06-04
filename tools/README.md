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

常用参数：`--port 8080`（端口）、`--host 0.0.0.0`、`--rate 2`（SIM 倍速）。
打开浏览器访问 `http://<host>:<port>`。

## 面板能力

- **仓库地图**：货架、出货 dock、充电桩、交通区与让行等待点，两车实时位姿、
  目标连线与电量/状态标签；**点击地图即可让选中的车直接开过去**。
- **手动控制**（每车）：`Goto x,y`、`⚡ 充电`、`⌂ 返航回位`、`■ 停车`。
- **派发任务**：选货架 + 优先级 + 指定车（或自动），入队 `task`。
- 实时队列、完成数、活动日志。

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
   - #6 仍被阻塞的内部重试会**重置退避**，不再每 tick 空转。
