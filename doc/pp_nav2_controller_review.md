  # PP（你现在的实现）问题梳理、改进建议，以及迁移到 Nav2 Controller 插件的方案

本文基于仓库当前实现（`src/pp_controller`）与 Nav2 Humble 的 `RegulatedPurePursuitController`（RPP）接口进行对比与迁移建议。

---

## 1. 你现在的 PP（pp_controller 节点）在做什么

当前 `pp_controller` 更像是“**路线执行器 + 控制器**”的组合：

- 路线来源：从 JSON 文件解析出多段直线（每段有 `start_spin/dir/target_v`）。
- 运行方式：内部状态机 `IDLE / SPINNING / LINE_TRACKING / COMPLETED`。
- 控制器核心：`LineTracker::compute()`（Pure Pursuit）+ `SpinController`（原地旋转）。
- 增强：曲率减速、横向偏差超限停车、LaserScan 简易障碍停车、OccupancyGrid 简易 cost 因子减速。

这套设计对于“**严格按直线段跑矩形 + 角点原地旋转**”这类需求很直观，但它与 Nav2 的职责分层不同（Nav2 里：路径/行为树负责动作编排，controller 负责跟踪）。

---

## 2. 你现在这版 PP 的主要问题（按影响优先级）

### 2.1 角点“原地旋转再走直线”与 Pure Pursuit 的天然行为冲突

Pure Pursuit（以及 Nav2 RPP）天然会“**切角**”：看前瞻点会导致转弯时走圆弧，而不是停下原地旋转。

你现在用状态机强行实现 stop-turn-go，这是可行的，但意味着：
- 一旦你要融入 Nav2（FollowPath），控制器插件**不应该**再自带“按段切换 + start_spin”的执行逻辑，否则会和 BT / goal checker / progress checker 互相打架。

结论：**角点原地旋转应该交给 Nav2 的行为/任务层**（比如分段目标、NavigateThroughPoses、或 BT 加 Spin 行为），controller 尽量只做 “FollowPath”。

### 2.2 `LineTracker` 的几何边界处理不够稳（容易在段端/超调时抖动）

`LineTracker::compute()` 里：
- `progress` 和 `lookahead_progress` 只做了 `min(progress+ld, path_length)`，没有对 **下界**做 clamp。
- 横向偏差用的是“点到无限延长直线距离”，在接近端点、投影落在段外时，可能对“该不该继续沿该段”判断不够贴合你“分段直线”的语义。
- 到点判断只用 `dist_to_end < position_tolerance`，未约束到点时的朝向（如果你要角点严格对齐，可能需要“到点后再对齐”的编排）。

建议：
- 将 `progress`、`lookahead_progress` 明确 clamp 到 `[0, path_length]`。
- 横向偏差改成“点到线段距离”（或至少在投影落段外时改用到端点距离）。
- “段完成”的判断建议同时参考：沿段方向的投影进度 + 端点距离（避免从侧面靠近端点时误判）。

### 2.3 TF 模式下 `current_vel_` 并非真实速度

TF 模式只更新位姿，不更新速度，`current_vel_` 变成“上一帧命令速度”，用于前瞻自适应会偏离真实运动（轮滑、限速器、底盘跟踪误差都会让真实速度不同）。

建议：
- 如果要 velocity-scaled lookahead：优先订阅 odom 的 twist 或用 `nav2_util::odometry_utils`。
- 或直接使用固定 lookahead / lookahead_time（Nav2 RPP 的做法）并在 Nav2 侧接入真实 odom。

### 2.4 Costmap / 障碍物处理属于“弱版”，且只看机器人脚下

你现在的 `getCostFactor()`：
- 仅查机器人当前位置的 OccupancyGrid cost，不能反映“前方轨迹是否会碰撞”；
- `cost_scaling_dist_` 参数目前没参与计算；
- LaserScan 的停车只看固定角度范围，不考虑 footprint、速度、转弯轨迹。

对比 Nav2 RPP：
- 会变换/裁剪全局路径到局部 costmap 范围；
- 可做 collision checking（footprint forward simulate 到 carrot）。

建议：
- 如果你要继续用“本地避障+跟踪”：优先直接用 Nav2 RPP / MPPI / DWB 这类已有 controller。
- 你这套更适合作为“无动态避障”的路线跟踪器（或配合更可靠的上层安全停车机制）。

---

## 3. PP vs Nav2 自带 RPP：差异总结（面向你的需求）

### 3.1 算法差异

- **Pure Pursuit（PP）**：核心是“找前瞻点 → 几何算曲率 → `ω = v * κ`”。优点是简单、可解释；缺点是容易切角、对速度/曲率/障碍缺乏系统约束。
- **Regulated Pure Pursuit（RPP）**：在 PP 基础上加了“调速/约束/碰撞检查/旋转策略”等，让它更像一个可落地的导航 controller。

### 3.2 能力差异（你现在实现 vs Nav2 RPP）

你现在的实现（`src/pp_controller`）已包含部分 RPP 思路（曲率调速、cost 因子），但与 Nav2 RPP 相比主要缺口在：

- 路径变换与裁剪（plan → local costmap frame）
- footprint 级碰撞预测（time-to-collision / forward simulate）
- 旋转到路径/目标航向的策略（避免大角度误差时边走边拧造成抖动）
- 反向（cusp）处理更系统（你是 per-segment `dir`，RPP 是从路径几何识别）
- 动态参数与调试可视化（carrot / arc / transformed plan）

### 3.3 对你“矩形路径 + 角点原地旋转 + 直线精度”的建议

- 如果你的优先级是：**稳定、少维护、要在有障碍的真实环境跑**  
  → 直接用 Nav2 RPP（或 MPPI）更合适。
- 如果你的优先级是：**严格 stop-turn-go、每段必须是直线、轨迹形状比避障更重要**  
  → 你的方案更贴合，但建议把“动作编排”上移（分段目标 / BT），controller 只负责跟踪。

---

## 4. 已完成：把 PP 改成 Nav2 Controller 插件格式（Humble）

已在 `pp_controller` 包内新增一个 Nav2 controller 插件：

- 插件类：`pp_controller::Nav2PurePursuitController`
- 插件描述：`src/pp_controller/pp_controller_nav2.xml`
- 代码：
  - `src/pp_controller/include/pp_controller/nav2_pure_pursuit_controller.hpp`
  - `src/pp_controller/src/nav2_pure_pursuit_controller.cpp`

这个插件做的事情是：

- `setPlan()` 接收 Nav2 的 `nav_msgs/Path`
- `computeVelocityCommands()`：
  - 在 path 上找前瞻点（按累计距离，带插值）
  - 把前瞻点变换到 `base_link`
  - 用纯 PP 计算曲率并输出 `TwistStamped`
  - 支持少量“RPP 风格”的调速参数（看下文）

注意：它是“轻量 PP 插件”，并不等价于完整 Nav2 RPP（不含 footprint 碰撞预测等）。

---

## 5. 如何在 Nav2 中启用你的 PP 插件（示例）

在 `controller_server` 的 `FollowPath` 插件配置中切换：

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "pp_controller::Nav2PurePursuitController"

      desired_linear_vel: 0.5
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      use_velocity_scaled_lookahead_dist: false

      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.6

      use_regulated_linear_velocity_scaling: true
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.25

      use_rotate_to_heading: false
      rotate_to_heading_min_angle: 0.785
      rotate_to_heading_angular_vel: 1.8

      allow_reversing: true
      max_angular_vel: 1.0
      transform_tolerance: 0.1
```

如果你要“角点必须原地旋转”，建议在上层用：
- `NavigateThroughPoses`（每个角点作为一个 pose goal，yaw 设置为下一段方向）
- 或 BT 在到达角点后插入 `Spin` 行为  
而不是让 controller 插件自己维护“分段状态机”。

---

## 6. 下一步可选改进（按收益排序）

1. **把你的 JSON route 执行器拆出来**：单独做一个“route executor”，负责把直线段/角点转换为一串 Nav2 goals / nav_msgs/Path，然后让 Nav2 controller 只负责跟踪。
2. **补齐 PP 插件的关键安全能力**：至少增加基于 costmap 的前向采样（沿 carrot arc / 未来短时间轨迹采样）来避免“脚下安全但前方碰撞”。
3. **更严格的直线段语义**：若一定要“直线 + 原地旋转”，考虑用“DriveOnHeading（直走）+ Spin（转向）”两类行为组合，而不是用 PP 去逼近直线。

---

## 7. 你现在这套能不能很好完成任务？

- 在“地图已知、障碍很少、路线固定、强调几何轨迹”的场景：可以，但建议修几何边界与速度/到点逻辑（第 2 节）。
- 在“局部动态障碍、需要可靠避障与安全约束”的场景：更推荐直接用 Nav2 RPP（或 MPPI），你这套的 cost/scan 处理不足以替代完整 collision checking。

