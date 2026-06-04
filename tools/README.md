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

1. **BASELINE** — 用仓库里提交的 `warehouse_layout.yaml`（`station_lane: exclusive`）。
2. **EXPERIMENT** — 把 `station_lane` 临时降级为 `queue`（写到临时文件，不改仓库配置），
   以放开并行路线。

## 已验证的结论

- 批量任务分配（`_sched_loop` 优化）生效：能并行时两台车在同一调度周期被同时派出。
- 提交的布局里 `station_lane` 为 `exclusive`，是整个车队的串行化瓶颈
  （每个任务都要穿过它）。
- 仅放开该瓶颈会暴露**让行活锁**（两车在共享出货站附近反复让行往返），
  makespan 反而变差——对应 `diffweakplan.md` 里第 6 点"让行震荡"。

这个台架是确定性的，可作为后续优化让行 / 路权逻辑时的回归验证工具。
