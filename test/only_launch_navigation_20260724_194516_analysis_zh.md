# Only Launch Navigation 实验分析

实验目录：`test/only_launch_navigation_20260724_194516`

原始数据：

- `system_metrics.csv`：Jetson `/proc`、`/sys` 直接采样，共 133 个样本。
- `node_cpu_metrics.csv`：ROS 进程 Top 20，共 132 个样本。
- `pipeline_rates.csv`：14 个监控信号，共 133 个 1 秒统计区间。
- `pipeline_events.csv`：8 个自动驾驶/导航事件。

## 总结

1. **导航控制链路的频率符合预期。** AUTO_DRIVE 稳定阶段，`/cmd_vel` 和 `/drive` 都是 20.000 Hz，`/ackermann_cmd` 是 19.986 Hz，`/odom` 是 49.951 Hz，没有持续掉频证据。
2. **本实验范围内可测的控制延迟正常。** 进入 AUTO_DRIVE 后 36.7 ms 出现首个非零 `/cmd_vel`，再经过 12.9 ms 出现最终非零 `/ackermann_cmd`，均小于一个 20 Hz 控制周期（50 ms）。
3. **不能声称所有 latency 均已验证。** 本次没有启动 Perception、CARKit Behavior 和 `stop_latency_monitor`，因此没有 YOLO、目标稳定、Behavior override 或实际停车分段延迟；规划完成事件也没有被记录。
4. **CPU 没有持续饱和，但 Navigation-only 基线偏高。** AUTO_DRIVE 阶段总 CPU 平均 64.43%、P95 66.97%、最大 67.60%，约使用 3.9 个六核 CPU 核心；启动瞬间最高 91.40%。
5. **没有热限制迹象。** AUTO_DRIVE 平均输入功率 8.87 W，CPU 最高 55.09°C，TJ 最高 55.03°C，频率平均 1606 MHz，温度和功率均稳定。

## 实验时间线

| 相对时间 | 事件 | 说明 |
|---:|---|---|
| 0.387 s | `navigation_failed` | 来自 Transient Local 状态的旧 `Route aborted by Nav2`，发生在本次路线请求前，应视为上一轮残留状态 |
| 28.247 s | 路线开始请求 | `start`，当时仍是非 AUTO 模式 |
| 28.265 s | Goal accepted | Nav2 接受 7 个 Pose，距请求 18.46 ms |
| 31.607 s | 进入 AUTO_DRIVE | 距 Goal accepted 3.341 s，这部分是操作顺序等待，不是系统处理延迟 |
| 31.643 s | 首个非零 `/cmd_vel` | 速度 1.5 m/s，距进入 AUTO 36.66 ms |
| 31.656 s | 首个非零 `/ackermann_cmd` | 速度 1.5 m/s，距 `/cmd_vel` 12.95 ms |
| 32.005 s | 确认车辆开始运动 | `/odom` 连续高于 0.08 m/s 达 0.220 s |
| 118.056 s | 退出 AUTO_DRIVE | 切换到 HUMAN_CONTROL |

没有记录到 `initial_pose_published`、`planning_completed`、`vehicle_stopped` 或 `route_completed`。因此这份数据只能确认一段约 86.45 秒的自动驾驶运动，不能证明是否完整跑完路线。

此外，本次先发送路线、后进入 AUTO。`pipeline_rate_monitor` 在进入 AUTO 时会重置内部 route 观察状态，从而忘记 3.34 秒前已经收到的 `goal_accepted`。如果 Foxglove 不再次发布 `Navigating through...` 状态，monitor 后续就无法把 `Route completed` 与该路线关联。因此缺少 `route_completed` 不一定表示 Nav2 没跑完，也可能是事件状态机与操作顺序不匹配。

## Topic Hz

统计窗口对周期 topic 使用稳定 AUTO 阶段约 33–118 s，避免把启动前后的 0 Hz 和边界半秒混入平均值。

| 信号 | 预期 | 实测 | P95 消息年龄 | 判断 |
|---|---:|---:|---:|---|
| `/cmd_vel` | 20 Hz，路线有效时 | 19.9996 Hz | 19.81 ms | 达标 |
| `/drive` | 跟随 `/cmd_vel`，20 Hz | 19.9996 Hz | 19.09 ms | 达标 |
| `/ackermann_cmd` | Control Center 20 Hz | 19.9863 Hz | 7.54 ms | 达标 |
| `/odom` | VESC 遥测链路约 50 Hz | 49.9506 Hz | 11.22 ms | 达标 |
| `/control_center/selected_cmd` | 仅来源改变时发布 | 全程 2 条 | 不适用 | 合理，不应按周期 Hz 判断 |

稳定窗口内：

- `/cmd_vel` 和 `/drive` 的 85 个区间中，83 个正好收到 20 条，另外两个分别是 19 和 21 条。两相邻区间合计仍为 40 条，这是 1 秒统计窗口边界抖动，不是持续丢消息。
- `/ackermann_cmd` 的 132 个有效区间中，126 个收到 20 条，聚合频率为 19.986 Hz。
- `/odom` 的 132 个有效区间中，124 个收到 50 条，聚合频率为 49.951 Hz。
- 最大消息年龄分别约为 `/cmd_vel` 72.84 ms、`/drive` 72.12 ms、`/ackermann_cmd` 59.97 ms、`/odom` 44.85 ms；这些最大值出现在启动/切换边界，P95 没有异常。

以下信号全程为 0 Hz，符合此次只启动底盘和导航、不启动 Perception/CARKit Behavior 的实验范围：

- `image_raw_rx`
- `yolo_inference_done`
- `behavior_detection_rx`
- `behavior_scan_rx`
- `behavior_plan_rx`
- `behavior_state`
- `behavior_override_active`
- `behavior_override_cmd`
- `behavior_stop_trace`

注意：`behavior_plan_rx` 是 Behavior 收到 `/plan` 后发布的轻量 pulse。它为 0 只说明 Behavior 没有运行，**不能说明 Nav2 没有发布 `/plan`**。这也是本次缺少 `planning_completed` 事件的原因。

## Latency

| 区间 | 实测 | 评价 |
|---|---:|---|
| 路线请求 → Goal accepted | 18.46 ms | 正常 |
| Goal accepted → 进入 AUTO | 3341.43 ms | 人工操作等待，不属于系统 latency |
| 进入 AUTO → 首个非零 `/cmd_vel` | 36.66 ms | 正常，小于一个 50 ms 控制周期 |
| `/cmd_vel` → `/ackermann_cmd` | 12.95 ms | 正常 |
| `/ackermann_cmd` → 运动确认事件 | 348.55 ms | 包含 220.12 ms 连续运动确认时间 |
| `/ackermann_cmd` → 推算首次越过 0.08 m/s | 约 128.42 ms | 车辆控制响应合理 |

`last_message_age_s` 表示 monitor 每秒采样时，距离该 topic 最近一次消息过去了多久；它是消息新鲜度，不是发布端到订阅端的 DDS 传输 latency。

本目录没有 `stop_latency_monitor` 的 CSV，因此以下延迟无法评价：

- 图像时间戳 → YOLO detection 发布
- 首帧检测 → 目标稳定确认
- 目标稳定 → Behavior 零速度 override
- 零速度 override → `/odom` 实际停车

## CPU、频率、功率和温度

System Monitor 是直接读取系统接口的数据，以下以它为总 CPU 权威来源。

| 阶段 | CPU平均 | CPU P95 | CPU最大 | 平均频率 | 平均VDD_IN功率 |
|---|---:|---:|---:|---:|---:|
| 全部 133 s | 64.51% | 67.68% | 91.40% | 1577 MHz | 8.82 W |
| 启动后、路线前 5–28 s | 65.05% | 81.60% | 86.40% | 1582 MHz | 8.72 W |
| AUTO_DRIVE 31.61–118.06 s | 64.43% | 66.97% | 67.60% | 1606 MHz | 8.87 W |
| 退出 AUTO 后 | 61.75% | 65.34% | 65.90% | 1443 MHz | 8.61 W |

AUTO_DRIVE 阶段每核平均占用率约为：

```text
CPU0 67.58%   CPU1 61.18%   CPU2 60.54%
CPU3 62.21%   CPU4 68.52%   CPU5 66.28%
```

每核 P95 为 68.33%–76.14%，没有核心持续达到 100%。各核最大值差的平均值约 15.9 个百分点，存在一般程度的不均衡，但没有单核长期瓶颈。

AUTO_DRIVE 阶段温度：

| 指标 | 平均 | 最大 |
|---|---:|---:|
| CPU温度 | 54.53°C | 55.09°C |
| GPU温度 | 53.10°C | 53.69°C |
| TJ温度 | 54.60°C | 55.03°C |
| VDD_IN功率 | 8.87 W | 9.10 W |

温度曲线稳定，没有看到热降频或功率限制特征。

## CPU最高的ROS进程

以下是 AUTO_DRIVE 阶段进程平均 CPU；进程 CPU 以单个逻辑核心为 100%，不是整机百分比。

| 进程 | 平均CPU | 最大CPU | 说明 |
|---|---:|---:|---|
| `amcl` | 71.10% | 78.80% | 最大CPU来源，运动/激光更新后较路线前约翻倍 |
| `odom_tf_broadcaster` | 34.08% | 39.00% | 第二大稳定开销 |
| `bt_navigator` | 26.05% | 30.20% | Nav2行为树 |
| `planner_server` | 25.18% | 32.10% | Nav2规划器 |
| `controller_server` | 22.22% | 24.90% | 20 Hz路径跟随控制器 |
| `foxglove_bridge` | 18.28% | 21.80% | 可视化桥接开销 |
| `behavior_server` | 17.51% | 21.00% | 这是 Nav2 Behavior Server，不是 CARKit Behavior Center |
| `smoother_server` | 17.11% | 19.00% | Nav2路径平滑服务 |

Node CPU CSV 只保留每秒 Top 20。某个进程没有出现在部分样本中，不能解释为它的 CPU 为零。`pipeline_rate_monitor` 通常不在 Top 20；出现时约为 3%–4%，不是主要负载来源。

## 综合判断与后续建议

- **Hz结论：通过。** 当前被实际激活的周期链路全部达到配置频率，未见 CPU 抢占造成的持续降频或消息吞吐下降。
- **Latency结论：部分通过。** 可测的 Nav2/Control 启动延迟正常，但规划完成、路线完成、停车和感知延迟没有数据，不能评价“所有 latency”。
- **CPU结论：可运行但基线偏高。** AUTO_DRIVE 稳定在约 64%，没有饱和或温度风险；但这还没有加入相机、YOLO和 CARKit Behavior，完整自动驾驶的 CPU 余量需要单独验证。
- 优先关注 `amcl` 和 `odom_tf_broadcaster`。AMCL 在运动阶段约占 0.71 个核心；`odom_tf_broadcaster` 在全程约占 0.34 个核心。
- 下次完整实验应在进入 AUTO_DRIVE 前启动 monitor，并且**先进入 AUTO、再发送 `start`**；这样不会在 Goal accepted 后重置 route 状态。保持记录直到收到 `route_completed` 和 `vehicle_stopped`。
- 若希望 Navigation-only 实验也记录规划完成，应让 monitor 直接获得 `/plan` 的轻量 pulse，而不是依赖未启动的 Behavior 节点。
- 开始完整实验时同时启动 `stop_latency_monitor`，才能回答 Stop Sign/红灯从图像到实际停车的分段延迟是否达标。
