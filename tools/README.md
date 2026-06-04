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
