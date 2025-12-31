# Isaac Sim × ROS 2 Humble × Nav2 全栈仿真测试与算法路线图（差速/3D LiDAR/LIO融合/社会导航/50Hz/RL+MPC/TSDF+MINCO）

> 目标读者：从入门到进阶再到资深，想用 Isaac Sim + Nav2 做**可复现**、**可回归**、**可部署**的导航仿真与算法评测，并最终把自训局部控制器（RL/IL/MPC）迁移为 Nav2 `controller_server` 插件。  
> 你的约束：差速底盘，`v_max=1.0m/s`，`w_max=1.0rad/s`，`a_max≈1.0m/s²`（角加速度同量级约束），3D LiDAR；定位链路采用 **LIO + 融合（robot_localization）**；需要**同时验证“感知→跟踪→社会导航”整链**；控制频率目标 **50Hz**；训练走 GPU，推理需兼容 CPU/GPU，且支持 **TensorRT / OpenVINO**；控制输出 `cmd_vel`。

文档版本: 1.0  
最后更新: 2025-12-23

---

## 目录
- [1. 总体架构：分层、可替换、可评测](#1-总体架构分层可替换可评测)
- [2. 入门路线（0→1）：最小可信闭环](#2-入门路线01最小可信闭环)
- [3. 进阶路线（1→N）：定位评测、社会导航、系统化回归](#3-进阶路线1n定位评测社会导航系统化回归)
- [4. 资深路线（工程化落地）：自训局控插件、MPC实时化、TSDF/ESDF、MINCO](#4-资深路线工程化落地自训局控插件mpc实时化tsdfesdfminco)
- [5. 50Hz 实时与稳定性：性能预算与防抖](#5-50hz-实时与稳定性性能预算与防抖)
- [6. 评测体系：场景矩阵、指标、录包/回放、报告](#6-评测体系场景矩阵指标录包回放报告)
- [7. 风险清单与排雷手册（按出现频率排序）](#7-风险清单与排雷手册按出现频率排序)
- [8. 推荐的里程碑交付物（每阶段可验收）](#8-推荐的里程碑交付物每阶段可验收)
- [9. MuJoCo 补充路线：与 Isaac Sim 的分工与融合](#9-mujoco-补充路线与-isaac-sim-的分工与融合)
- [10. 视觉全链路：识别/追踪/定位/动态SLAM/安全走廊](#10-视觉全链路识别追踪定位动态slam安全走廊)
- [11. 分层学习路径（入门→进阶→资深）与时间规划](#11-分层学习路径入门进阶资深与时间规划)
- [12. 自研“感知-定位-规划-控制（归控）”方案（Batch-LIWO × 哨兵2025）](#12-自研感知-定位-规划-控制归控方案batch-liwo--哨兵2025)

---

## 1. 总体架构：分层、可替换、可评测

### 1.1 六层架构（强烈建议严格遵守）
1) **仿真与真值层（Isaac Sim）**  
   - 发布：`/clock`、`/tf`、`/tf_static`、`/odom`（或 wheel odom）、`/imu`、`/points_raw`（PointCloud2）、相机/深度等（可选）、`/ground_truth/*`（建议一定有）  
   - 订阅：`/cmd_vel`  
   - 关键：物理步长固定、传感器频率固定、噪声/延迟可控、随机种子可复现

2) **定位/里程计层（LIO + 融合）**  
   - 3D LiDAR + IMU：LIO 输出相对里程计（`odom`）或地图坐标（`map`）  
   - `robot_localization` 融合 wheel odom + IMU + LIO，输出平滑且低延迟的 `odom->base_link`，并（可选）输出 `map->odom`

3) **环境表征层（Nav2 costmap + 可选 TSDF/ESDF）**  
   - 3D 点云 →（滤波/地面分割/降采样）→ costmap 的体素/时空体素层  
   - 进阶：维护 TSDF/ESDF，供 MPC/优化型局控做连续距离查询与梯度

4) **导航编排层（Nav2 标准组件）**  
   - `bt_navigator`：策略与流程（重规划、恢复、让行、等待、换控制器等）  
   - `planner_server`：全局规划（Smac/Theta* 等）  
   - `controller_server`：局部控制（MPPI / RPP / 你的 RL/MPC 插件）  
   - `behavior_server`：恢复与机动（Spin/BackUp/DriveOnHeading）  
   - `collision_monitor` / `velocity_smoother`：安全最后防线与速度平滑

5) **学习/优化控制器层（可部署推理）**  
   - RL/IL policy：ONNX 单一格式 + 多后端（TensorRT/OpenVINO/CPU）  
   - MPC：CasADi（原型）→ acados（实时）  
   - 统一输入特征规范（否则无法回归对比）

6) **评测与回归层（必须优先建设）**  
   - `rosbag2` 录包与回放（同包多算法对比，保证公平可复现）  
   - 指标统计与报告（成功率、碰撞、人机距离、舒适性、实时性、定位误差等）

### 1.2 最重要的工程原则（决定你能否“持续迭代而不崩”）
- **KISS**：优先让系统“可跑、可测、可对比”，再追求“更先进”。  
- **YAGNI**：不要一上来同时上 LIO+TSDF+MPC+RL+社会导航；每一层先有 baseline。  
- **DRY**：观测特征、评测指标、录包清单、场景生成要做成共享模块，避免每个算法各搞一套。  
- **SOLID**：控制器/规划器/社会层/地图层用插件接口隔离，允许替换而不改 Nav2 主体。

---

## 2. 入门路线（0→1）：最小可信闭环

### 2.1 目标
在 1–2 周内完成以下闭环并能稳定回归：
- Isaac Sim → ROS 2 bridge 正常（时间/TF/传感器/控制）
- Nav2 baseline 能稳定到点（静态障碍）
- 3D 点云接入 costmap（可避障）
- 录包/回放能复现同样结果（为后续评测打基础）

### 2.2 最小必备 topic/TF 清单（不满足就不要往下做）
**时间**
- `/clock`（Isaac Sim 发布）
- 所有 ROS 节点：`use_sim_time:=true`

**TF**
- `map -> odom -> base_link -> lidar_link`（至少这条主干闭合）
- `imu_link`、`base_footprint`（可选但推荐规范化）

**里程计与传感器**
- `/odom`（nav_msgs/Odometry）：建议来自 wheel odom（高频、低延迟）
- `/imu`（sensor_msgs/Imu）
- `/points_raw`（sensor_msgs/PointCloud2）：3D LiDAR 点云

**控制**
- `/cmd_vel`（geometry_msgs/Twist）：Nav2 输出，Isaac Sim 底盘消费

### 2.3 Isaac Sim 侧建议（避免后续迁移失败）
**固定频率与步长**
- 物理步长（physics dt）固定，建议 `0.005~0.01s`；  
- 控制周期与步长整比：50Hz 控制 → `dt_control=0.02s`，物理可用 `0.005s`（4:1）或 `0.01s`（2:1）。

**传感器时间戳**
- 确保点云/IMU/odom 的 header stamp 单调递增且与 `/clock` 一致。

**噪声/延迟先留钩子**
- 入门阶段可先关闭复杂噪声；但务必在仿真图里预留：延迟、抖动、丢帧、噪声开关（后续做 domain randomization）。

### 2.4 3D LiDAR → costmap（推荐做法：点云给 costmap，2D scan 仅用于 AMCL/对比）
**避障：点云直进 costmap**
- 推荐：使用“时空体素/体素层”把点云融合到 2D costmap 障碍层。  
- 目标：动态障碍可衰减、静态障碍稳定、更新频率可控（10–20Hz 常见）。

**定位：投影成 `/scan`（仅用于 AMCL baseline 或对照）**
- 用 `pointcloud_to_laserscan`（或同类）从 `/points_raw` 投影到 `/scan`，作为 AMCL 的输入。
- 注意：这条链路是 baseline/对照用；你的主定位路线是 LIO+融合，但入门阶段 AMCL 对排障非常有帮助（可快速验证 TF 与 map 坐标一致性）。

### 2.5 Nav2 baseline 推荐组合（入门到进阶都能当对照组）
**为什么先用它**：后面你做 RL/MPC 都需要一个强基线（teacher + 对照），否则你无法证明改进有效。

- 全局规划（`planner_server`）：`Smac` 或 `Theta*`  
- 路径平滑（`smoother_server`）：先用 Nav2 自带平滑器  
- 局部控制（`controller_server`）：优先 `MPPI`；备选 `Regulated Pure Pursuit (RPP)`  
- 安全与平滑：`collision_monitor` + `velocity_smoother`（强烈建议必开）

### 2.6 入门阶段必须做的验证（不做会在后面付 10 倍代价）
**录包清单（v1：静态场景）**
- `/tf` `/tf_static` `/clock` `/odom` `/imu` `/points_raw` `/scan` `/cmd_vel`
- Nav2 关键输出（按你实际启用的 topic）：全局路径、局部路径、costmap 更新、行为树状态（如果有）

**回放与一致性**
- 同一个 bag 回放应当能复现“到点成功/失败”的结果（允许微小差异，但不能本质翻转）。

### 2.7 入门落地：推荐的启动拓扑与诊断命令（建议你照这个顺序排障）
> 下面给的是“最小但完整”的拓扑，便于把问题定位到某一层；每一层都能单独录包回放。

**启动拓扑（建议顺序）**
1. Isaac Sim：底盘 + 传感器 + ROS2 Bridge（先不启用复杂噪声/延迟）
2. `robot_state_publisher`：发布 URDF 静态 TF（`base_link->lidar_link/imu_link` 等）
3. wheel odom（仿真底盘输出）与 `robot_localization`（先只融 wheel+IMU，输出 `/odometry/filtered`）
4. 点云预处理（滤波/降采样/地面分割，可先关）
5. Nav2：先跑 baseline（MPPI/RPP + collision_monitor + velocity_smoother）
6. RViz2：确认 TF、costmap、路径、cmd_vel 行为符合预期

**高频诊断命令（建议你做成脚本）**
```bash
# topic 是否存在与频率是否稳定
ros2 topic list
ros2 topic hz /cmd_vel
ros2 topic hz /points_raw
ros2 topic hz /odometry/filtered

# TF 链路是否闭合、时间是否正常
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames  # 生成 frames.pdf

# 是否存在仿真时间没开导致的 extrapolation
ros2 param get /bt_navigator use_sim_time

# 录包（v1：静态）
ros2 bag record -o bags/v1_static \
  /clock /tf /tf_static /odom /odometry/filtered /imu /points_raw /scan \
  /cmd_vel /goal_pose
```

**入门阶段的“硬门槛”**
- `/cmd_vel` 输出频率稳定（接近 controller_frequency）
- `tf2_echo` 不出现频繁 extrapolation / lookup timeout
- costmap 更新稳定（在 RViz2 中障碍物不“跳帧”）

---

## 3. 进阶路线（1→N）：定位评测、社会导航、系统化回归

### 3.1 定位主线：LIO + robot_localization 融合（并同时做评测）

#### 3.1.1 推荐 TF/坐标系约定（建议固定下来别反复改）
- `base_link`：车体坐标系（控制输出以此为准）  
- `lidar_link`、`imu_link`：传感器安装坐标系（静态外参用 `tf_static`）  
- `odom`：连续、平滑、局部一致（不要求闭环）  
- `map`：全局一致（可闭环，漂移更小）  
- `world`/`sim`：仿真世界坐标（建议只在仿真内部用，ROS 侧尽量对齐到 `map`）

#### 3.1.2 融合架构（推荐“两层滤波”模式）
**局部滤波（高频、低延迟）**
- 输入：wheel odom、IMU（必要时加 LIO 的高频里程计）  
- 输出：`/odometry/filtered`（`odom->base_link`）  
- 用途：给控制器提供稳定速度/姿态估计

**全局约束（低频但全局一致）**
- 输入：LIO map pose（或建图/回环结果）  
- 输出：`map->odom`（或直接输出 map 下的 pose）  
- 用途：给 Nav2 全局规划与可视化提供全局一致坐标

> 注：不同 LIO 工程实现输出坐标系可能不同（有的输出 `odom`，有的输出 `map`）。原则是：你要显式决定谁负责 `map->odom`，避免 TF 冲突（TF 冲突是最常见“看起来能跑但偶尔炸”的根因之一）。

#### 3.1.3 LIO 选型与“接口契约”（你需要先把 I/O 约定写死）
> 不同 LIO（FAST-LIO/LIO-SAM/Point-LIO 等）的算法细节不同，但对 Nav2 来说最重要的是：**它输出什么坐标系的位姿、以什么频率、延迟多大、是否闭环**。  
> 由于你要“最新工具 + 可持续更新”，建议你把 LIO 当作黑盒：只要满足接口契约就能替换。

**你应该要求 LIO 满足的最小接口**
- 输入：
  - `/points_raw`（PointCloud2，`lidar_link`）
  - `/imu`（Imu，`imu_link`）
  - （可选）wheel odom（用于 deskew/辅助）
- 输出（任选其一，但要统一）：
  - 方案 A：输出 `odom->base_link` TF + `/lio/odom`（局部里程计，连续平滑）
  - 方案 B：输出 `map->base_link` TF + `/lio/pose`（全局/建图坐标，可能闭环）
- 频率与延迟建议：
  - 位姿输出频率：≥ 20Hz（越高越利于 50Hz 控制的平滑插值）
  - 延迟：P95 < 50ms（否则控制会“追过去的自己”）

**你必须明确的 TF 责任划分（强约束，写进设计文档）**
- 如果 LIO 输出 `odom->base_link`：
  - `robot_localization`（local ekf）发布 `odom->base_link`（或发布 `/odometry/filtered` 并由 RL 不发布 TF）
  - LIO 只发布 `/lio/odom`（不直接发 TF），由 EKF 融合后发 TF（避免双发布）
- 如果 LIO 输出 `map->base_link`：
  - `robot_localization` local ekf 仍负责 `odom->base_link`
  - 再引入 global ekf 或“map->odom 估计器”，输出 `map->odom`
  - 严禁同时由多个节点发布 `map->odom`

#### 3.1.4 `robot_localization`（EKF/UKF）配置模板（以 50Hz 输出为目标）
> 参数名可能随发行版/分支略有差异，请以 `ros2 param list /ekf_filter_node` 实测为准；下面给的是“工程常用骨架”，核心是输入/输出坐标系与频率策略。

**局部 EKF（odom 框架，给控制器用）**
```yaml
# ekf_local.yaml（示例骨架）
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    sensor_timeout: 0.05
    two_d_mode: true

    # 坐标系（按你的 TF 规范统一）
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # 输入 1：wheel odom（推荐高频、低延迟）
    odom0: /odom
    odom0_config: [true,  true,  false,
                   false, false, true,
                   true,  false, false,
                   false, false, true,
                   false, false, false]
    odom0_differential: false
    odom0_queue_size: 10

    # 输入 2：IMU（用于航向与角速度稳定）
    imu0: /imu
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  false, false, false]
    imu0_differential: false
    imu0_queue_size: 50

    # 输出
    publish_tf: true
    publish_acceleration: false
```

**全局 EKF（map 框架，给规划与评测用，可选）**
- 如果你的 LIO 提供“全局 pose”（如 `map` 下 pose），可以把它作为 `pose0` 输入给 global ekf，输出 `map->odom`。
- 如果 LIO 只提供 `odom` 下里程计，则全局一致性通常来自回环/建图；此时建议由建图模块输出 `map->odom`，而不是强行用 EKF 造全局。

#### 3.1.5 时间同步与外参（LIO 可靠性的第一性原理）
**时间同步**
- `imu` 与点云时间戳必须在同一时钟域（仿真用 `/clock`）  
- 若存在固定延迟（如 bridge/传感器管线），建议显式建模（固定 offset）并在回归中记录该 offset

**外参**
- `imu_link` 与 `lidar_link` 外参要通过 `tf_static` 固定发布  
- 仿真里外参“看似正确”但时间戳乱，也会导致 deskew 错误 → 里程计抖动 → 控制抖动

#### 3.1.6 定位评测方法（必须量化，不能靠感觉）
**仿真真值（强烈建议在 Isaac Sim 发布）**
- `/ground_truth/pose`（建议在 `map` 或 `world` 下定义清楚）  

**误差指标（建议至少统计以下）**
- 平移误差：RMSE、P95、最大值  
- 航向误差：RMSE、P95、最大值  
- 稳定性：跳变次数（例如瞬时 >0.5m 或 >20°）、TF 断链次数、重定位次数  
- 对导航的影响：到达率、平均耗时、碰撞率随定位误差的退化曲线

**对照实验建议**
- Baseline A：AMCL（`/scan` + 2D map）  
- Baseline B：LIO（不融合）  
- Baseline C：LIO + EKF/UKF（融合 wheel + IMU）  
- 可选：在不同“误差注入/延迟注入”强度下做退化曲线（这对 RL/MPC 迁移非常关键）

### 3.2 社会导航：必须做“感知→跟踪→social layer→控制策略”的整链验证

#### 3.2.1 整链目标
你要验证的不只是“有人的真值位置时能绕开”，而是：
- 感知输出抖动/丢失/延迟时，社会行为是否仍稳定？
- 人群密度变化时，机器人是否能合理让行、等待、绕行？
- 局部控制是否会产生“急停-猛转-抖动”的不舒适行为？

#### 3.2.2 推荐接口（避免和某个特定实现强绑定）
**人/障碍物跟踪输出（建议统一成其中一种）**
- `people_msgs/People`（含位置、速度、可选协方差）  
或
- `geometry_msgs/PoseArray` + `geometry_msgs/TwistArray`（自定义组合）  
或
- `vision_msgs/Detection3DArray` / `tracked_objects`（如果你走自动驾驶风格）

你后续要做的是：在 costmap social layer 和 MPC/RL 代价中统一消费同一个“人群状态消息”，避免 DRY 破坏。

#### 3.2.3 社会层（social costmap layer）设计建议（工程优先）
**空间形状：非对称高斯/椭圆势场（前方更敏感）**
- 行人前方：更大惩罚半径  
- 行人后方：较小惩罚半径  
- 横向：中等惩罚（模拟“擦身而过”的不舒适）

**时间衰减**
- 跟踪丢失后势场应快速衰减（避免“鬼影”长期阻塞）

**参数化（必须能在线调）**
- `comfort_radius_front/side/back`  
- `max_cost`、`decay_time`  
- `velocity_scaling`（人走得快，前方势场拉长）

#### 3.2.4 行为树（BT）层的社会策略（不要把策略全塞进局控黑盒）
建议把“策略切换”放在 BT 层：
- 人群密集/走廊会车：降低速度上限、提高重规划频率  
- 被阻挡：`Wait`（短停让行）→ `Replan` → `BackUp`/`Spin`（必要时）  
- 近距离交汇：执行“让行动作”（例如靠边等待）

> BT 的价值：可解释、可回放、可调参；你做 RL/MPC 时仍可用 BT 保底。

#### 3.2.5 感知与跟踪实现选项（你要求“同时验证”，建议三路并行）
为了既能“快速迭代”又能“验证整链”，建议你在仿真里同时跑三路数据源，并在评测脚本里分别统计：

**A 路：真值人群（上帝视角，仅用于快速验证导航策略上限）**
- Isaac Sim 发布 `/people/ground_truth`（建议对齐到 `map` 或 `odom`，并明确 frame）  
- 作用：快速验证 social layer 参数、BT 策略、控制器是否具备“合理让行/绕行”能力  
- 注意：该路不能代表真实部署表现，但对“策略/权重”调参极其高效

**B 路：LiDAR-based 人体检测 + 跟踪（推荐作为主验证路）**
- 点云流程（建议先从简单版做起，再逐步增强鲁棒性）：
  1) 地面分割（RANSAC/栅格高度滤波）
  2) ROI 裁剪（只保留近场，例如 0.2–10m）
  3) 聚类（欧式聚类/DBSCAN）
  4) 目标筛选（尺寸/形状/高度先验）
  5) 跟踪（Kalman/IMM/JPDA，输出速度与协方差）
- 输出：`people_msgs/People`（推荐含速度与协方差）
- 作用：可以自然引入误检/漏检/抖动/延迟，对社会导航整链最有价值

**C 路：Vision-based 检测 + 多目标跟踪（用于验证“多模态/遮挡”场景）**
- RGB/Depth：2D 检测（YOLO/RT-DETR 等）+ MOT（ByteTrack/DeepSORT 等）→ 深度投影到 3D → 跟踪融合  
- 输出同样统一成 `people_msgs/People`  
- 作用：在遮挡/远距离/稀疏点云时更稳，但工程复杂度更高

**统一约定（强制）**
- 所有 `People` 输出必须提供：
  - `header.stamp` 与 `/clock` 同步
  - `header.frame_id` 明确（建议 `map` 或 `odom`），并能稳定 TF 到 `base_link`
- 跟踪丢失要显式标记或让速度/协方差变大，供 social layer 做时间衰减与风险管理

### 3.3 3D 点云 → costmap：工程配置要点（体素/时空体素层）
> Nav2 主体是 2D costmap，但你可以用 3D 点云构建障碍层（并投影到 2D 占据/代价）。  
> 关键不是“能不能进”，而是“动态能不能衰减、频率稳不稳、CPU/GPU 开销可控不”。

**推荐的点云预处理（按成本从低到高）**
1) 体素降采样（减点）  
2) 地面分割（避免地面全被当障碍）  
3) 离群点滤除（防噪点炸 costmap）  
4) 动态目标保留（为 social nav/动态避障服务）

**costmap 关键参数（原则）**
- `global_frame`: `map`（全局 costmap）或 `odom`（局部 costmap，常用）  
- `robot_base_frame`: `base_link`  
- `transform_tolerance`: 适当放宽（但不能掩盖 TF 断链）  
- 观测源（observation source）：点云 topic、最大/最小高度、更新/保留时间、标记/清除策略

> 参数名与插件实现相关（voxel/stvl 等略有差异）。建议你把“配置骨架”固定在一个 YAML，并在升级时用 `ros2 param dump` 对比差异，避免暗坑。

### 3.4 Nav2 参数模板（差速 + 1m/s + 1rad/s + 50Hz 的建议起点）
> 下面给的是“可用起点”，不是最终最优。不同场景需要不同权重与安全距离；你应当通过第 6 节的指标回归来迭代。

**Controller Server（MPPI 起点）**
- 目标：先跑通，再调到“不撞、不卡、不过度保守”，最后再优化舒适/效率。
- 建议关键约束对齐你的车体能力：`vx_max=1.0`、`wz_max=1.0`、`ax_max≈1.0`（以及角加速度同级约束）

**Velocity Smoother（强烈建议启用）**
- 目标：抑制局控输出高频抖动，避免仿真与真实迁移时出现“抖动放大”

**Collision Monitor（最后防线）**
- 目标：当 costmap 或控制器出现异常时仍能保证安全停车/限速

**Progress/Goal Checker**
- 目标：卡住时触发恢复（让行/等待/后退/旋转），而不是无限抖动

> 参数名建议你以实际安装的 Nav2 版本为准，通过 `ros2 param list /controller_server` 与 `ros2 param dump` 获取准确键名；本节重点是“参数设计意图”和“约束一致性”。

#### 3.4.1 `nav2_params_sim.yaml`（骨架示例，按你差速+50Hz约束给的起点）
> 说明：这不是“复制粘贴即可跑”的最终配置（不同插件键名会略有差异），但它能帮助你把关键约束与组件关系一次性理清。你应当用 `ros2 param dump` 校验键名并做版本锁定。

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odometry/filtered

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 50.0
    min_x_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    controller_plugins: ["FollowPath"]

    # baseline：MPPI（建议作为 teacher 与对照）
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      # 约束（与你给定对齐）
      vx_max: 1.0
      vx_min: 0.0
      wz_max: 1.0
      ax_max: 1.0
      az_max: 1.0

      # 采样与模型（从保守值起步，先跑通）
      model_dt: 0.05
      time_steps: 15
      batch_size: 1000
      iteration_count: 1
      temperature: 0.3
      gamma: 0.015

      # 代价项（建议后续逐项对照调参）
      # 例如：路径跟踪、障碍距离、平滑等（以你版本支持的 critic 为准）

planner_server:
  ros__parameters:
    use_sim_time: true
    expected_planner_frequency: 2.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlanner2D"

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 10.0
      height: 10.0
      resolution: 0.05
      update_frequency: 10.0
      publish_frequency: 5.0
      transform_tolerance: 0.2
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: false
        origin_z: 0.0
        z_resolution: 0.1
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: pointcloud
        pointcloud:
          topic: /points_filtered
          data_type: PointCloud2
          marking: true
          clearing: true
          min_obstacle_height: 0.10
          max_obstacle_height: 2.0
          obstacle_range: 6.0
          raytrace_range: 8.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.55
        cost_scaling_factor: 2.5

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.05
      update_frequency: 2.0
      publish_frequency: 1.0
      transform_tolerance: 0.5
      plugins: ["static_layer", "voxel_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        observation_sources: pointcloud
        pointcloud:
          topic: /points_filtered
          data_type: PointCloud2
          marking: true
          clearing: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"

velocity_smoother:
  ros__parameters:
    use_sim_time: true
    smoothing_frequency: 50.0
    feedback: "OPEN_LOOP"
    max_velocity: [1.0, 0.0, 1.0]
    max_accel: [1.0, 0.0, 1.0]
    max_decel: [-1.0, 0.0, -1.0]

collision_monitor:
  ros__parameters:
    use_sim_time: true
    base_frame_id: base_link
    odom_frame_id: odom
    cmd_vel_in_topic: /cmd_vel_raw
    cmd_vel_out_topic: /cmd_vel
    # 观测源可用 scan 或 pointcloud，按你实际接入方式选择
    # observation_sources: ["scan"]
    # scan:
    #   type: "scan"
    #   topic: /scan
```

#### 3.4.2 3D 点云“工程化”建议（/points_raw → /points_filtered）
你最好把点云预处理独立成节点（便于 profile 与回归），输出 `/points_filtered`：
- 降采样：控制点数上限（影响 costmap CPU 开销）
- 地面分割：避免地面在局部 costmap 变成“连续障碍带”
- 时间同步：必要时做 deskew/近似 deskew（与 LIO 一致）

### 3.5 行为树（BT）模板：社会导航 + 控制器降级切换（RL/MPC/MPPI）
> 建议你把“先进控制器”接在 BT 的“优先路径”上，同时保留 MPPI/RPP 作为降级路径。

**典型策略（文字版 BT 逻辑）**
- 主路径：导航到目标（优先用 RL/MPC）
  - 若检测到人群密集：降低速度上限、提高重规划频率
  - 若局部不可行/卡住：短暂停靠让行 → 重规划 → 恢复（BackUp/Spin）
  - 若推理/求解超时：切换到 MPPI（保守模式）或刹停
- 终止：到点姿态满足 + 速度足够小（与 goal checker 一致）

**关键点**
- 控制器切换不要隐式发生在同一个插件里（难回放、难解释），优先通过 BT/参数集明确表达。

---

## 4. 资深路线（工程化落地）：自训局控插件、MPC实时化、TSDF/ESDF、MINCO

### 4.1 自训局部控制器（RL/IL）→ Nav2 `controller_server` 插件

#### 4.1.1 输入特征规范（强烈建议固定为“三块”并做版本管理）
所有输入都建议转换到 `base_link` 坐标系，避免 TF 漂移影响学习泛化：

1) **局部路径片段**  
- 从全局路径取 lookahead 段（例如 2–6m 或 30–80 个点）  
- 特征：点坐标、切向角、曲率（可选）、离散弧长（可选）

2) **局部障碍表征（两种并存，便于对比与迁移）**
- 表征 A（工程常用）：局部 costmap 裁剪栅格（例如 64×64，分辨率 0.05–0.1m）  
- 表征 B（传感器域）：点云投影到极坐标扇区距离（更贴近真实传感器退化）

3) **状态与约束**
- 当前速度 `v,w`（来自 `/odometry/filtered`）  
- 约束参数（`v_max,w_max,a_max`）作为显式输入（便于同模型迁移不同机器人）

输出：`cmd_vel`（`v_cmd,w_cmd`），并在插件侧做：
- 约束投影/限幅（硬约束）  
- 一阶低通或 `velocity_smoother` 配合（避免抖动）

#### 4.1.2 训练路线建议：先 IL 后 RL（更稳更快）
**IL（模仿学习）阶段：用 MPPI 做 teacher**
- 在大量场景（含社会场景）采样：`(obs, cmd_vel_teacher)`  
- 好处：快速得到“可用”策略；并且 teacher 是强基线，便于解释与对比

**RL（强化学习）阶段：在 IL 初始化上微调**
- 重点优化：动态交互、舒适性（jerk）、鲁棒性（定位误差/延迟下不崩）

#### 4.1.3 迁移关键：把“定位误差/延迟/噪声”纳入训练分布
你明确要评测定位，因此训练必须包含：
- 定位噪声注入（对 `map->odom` 或 pose 测量加噪/漂移/偶发跳变）  
- 传感器延迟与抖动（点云/IMU/odom）  
- 动力学随机化（摩擦、质量、质心、轮半径）  
- 人群行为随机化（速度分布、密度、轨迹规则）

> 经验法则：如果训练时没有把“误差与延迟”当成常态，部署时你会看到非常典型的“抖动、犹豫、急停、贴人”。

#### 4.1.4 可部署推理：ONNX + 多后端（TensorRT/OpenVINO/CPU）
建议在控制器插件内部抽象推理后端（遵守 SRP，推理与控制逻辑解耦）：
- 后端 1：TensorRT（GPU，最低延迟）  
- 后端 2：OpenVINO（Intel CPU/iGPU，部署面广）  
- 后端 3：ONNX Runtime CPU（兜底）

建议提供参数：
- `model_path`：ONNX 路径  
- `backend`：`tensorrt|openvino|onnxruntime_cpu`  
- `fp16`：是否 FP16（TRT 常用）  
- `max_infer_ms`：推理超时上限（超时→安全刹停/降级）  
- `fallback_controller`：降级到 MPPI/RPP（通过 BT 或控制器切换实现）

#### 4.1.5 安全护栏（必须有，否则很难上线）
- TF 过期/断链、costmap 不更新、推理超时、输出 NaN/越界：立刻输出安全刹停（`cmd_vel=0`）  
- 最小距离检查：基于 costmap 或 ESDF，距离小于阈值时强制限速/停车  
- 与 Nav2 `progress_checker`、`goal_checker` 配合：卡住时触发恢复而不是持续抖动

#### 4.1.6 Nav2 控制器插件接口（你需要实现/遵循的生命周期与关键方法）
> 你最终要做的是一个 `nav2_core::Controller` 插件，跑在 `controller_server` 的 lifecycle 管理下。核心要求是：**稳定、可回放、可降级、可测耗时**。

**关键生命周期（概念层）**
- `configure()`：读取参数、创建推理后端、申请固定内存（预分配）、创建调试发布器  
- `activate()`：开始发布调试话题（如需要），进入可输出控制状态  
- `deactivate()`：停止输出与调试发布  
- `cleanup()`：释放资源  

**关键控制接口（概念层）**
- `setPlan(path)`：接收全局路径（你应该在这里做一次性预处理：采样、去噪、生成局部索引）  
- `computeVelocityCommands(pose, velocity, goal_checker)`：50Hz 被调用的热点，必须短小、无阻塞、可预测耗时  
- `setSpeedLimit(speed_limit)`：BT/策略层可能动态下发限速（社会场景非常有用）

#### 4.1.7 50Hz 实时实现要点（避免“性能抖动导致控制抖动”）
**强制建议**
- 预分配：特征张量、costmap 裁剪 buffer、候选轨迹 buffer、people 编码 buffer  
- 避免动态内存：50Hz 热路径不要 `new/malloc`，不要频繁构造大对象  
- TF 查询要有超时与缓存：TF 查不到直接降级/刹停，不要阻塞等待  
- costmap 访问要可控：不要每次都全图遍历；只裁剪局部窗口  
- 推理/求解超时一票否决：超时则输出安全速度并计数（用于回归报告）

**建议你输出的实时日志（用于回归）**
- `infer_ms`（或 `solve_ms`）、`feature_ms`、`tf_ms`、`total_ms`（P50/P95/P99）  
- `fallback_reason` 计数（tf_timeout / infer_timeout / nan_output / costmap_stale / progress_stuck）

#### 4.1.8 推理后端落地建议（TRT / OpenVINO / CPU 的工程策略）
> 你的目标是“一份 ONNX 模型，多种部署”。建议把推理后端独立成库/类，控制器只依赖抽象接口（依赖倒置）。

**模型发布建议**
- `model.onnx` + `model_meta.yaml`（必须包含 `schema_version`、输入输出名称/shape、归一化参数、训练约束 `v_max/w_max/a_max`）  
- 每次训练产出一个版本号（例如 `YYYYMMDD_hhmm_gitsha`），评测报告引用版本号，避免“跑的是哪个模型说不清”

**TensorRT（GPU）建议**
- 只在部署机上构建 engine（与 GPU/driver/cuda 绑定），不要把 engine 当通用工件  
- 开启 FP16（若模型允许）  
- 设定推理超时（软策略：超时降级到 MPPI；硬策略：超时刹停）

**OpenVINO（Intel）建议**
- 目标是 CPU/iGPU 低成本部署；把“可用性”放在“极限性能”之前  
- 同样要求固定输入 shape，避免运行期重新编译导致延迟尖峰

**CPU 兜底建议**
- ONNX Runtime CPU 作为兜底后端，保证无 GPU 环境仍可回归/验证  
- CPU 后端下建议自动降低控制 aggressiveness（例如降低 `v_max` 或提高安全距离），避免因延迟变大而更危险

---

### 4.2 MPC：CasADi 原型 → acados 实时（50Hz）

#### 4.2.1 为什么 MPC 值得做
- 约束（速度/加速度/角速度）可以“显式且可解释”  
- 引入社会代价、距离场（ESDF）后，行为通常比纯学习更稳定  
- 适合做“资深阶段的可靠局控”，并作为学习策略的 safety shield 或 teacher

#### 4.2.2 差速模型与控制变量建议
建议从运动学 unicycle 开始（足够实用，易实时）：
- 状态：`x, y, theta, v, w`（或简化为 `x,y,theta`）  
- 控制：`u=[v_cmd, w_cmd]`，并用差分约束近似加速度  
或
- 控制：`u=[a, alpha]`（线加速度与角加速度），更贴近 `a_max` 约束（实现略复杂）

约束（按你给定）：
- `0 ≤ v ≤ 1.0`（若允许倒车则 `-v_min ≤ v ≤ 1.0`）  
- `|w| ≤ 1.0`  
- `|a| ≤ 1.0`（通过 `Δv/Δt` 约束或直接把 `a` 作为控制量）  
- 可加 `|Δw/Δt|` 抑制甩头

#### 4.2.3 代价函数（建议分解，便于调参和日志）
1) 路径跟踪：横向误差、航向误差、沿程进度  
2) 控制平滑：`Δv, Δw` 或 `a, alpha`  
3) 障碍距离：来自 costmap（离散）或 ESDF（连续且可导）  
4) 社会代价：对行人势场/人机距离/TTC 进行惩罚  
5) 终端项：到点姿态误差、低速停稳

#### 4.2.4 CasADi 阶段（原型验证）
目标是把问题“建正确”：
- 输入接口：路径片段、障碍表征、人群状态、当前 `v,w`  
- 输出：`cmd_vel`  
- 验证：在静态与简单动态场景能稳定 50Hz（先不强求极限性能）

#### 4.2.5 acados 阶段（实时工程）
目标是把同样的问题“跑到 50Hz 并可回归”：
- 频率：50Hz → `dt=0.02s`  
- 预测步数：`N=20~40`（预测时域 0.4~0.8s 常用）  
- Warm start：上一周期最优解移位作为初值  
- 超时：设置 `max_solve_time_ms`，超时触发降级/刹停  
- 不可行：引入松弛变量（soft constraint）避免解崩掉导致抖动

#### 4.2.6 MPC 插件化（Nav2 Controller）
将 MPC 作为 `nav2_core::Controller` 插件：
- 输入：路径片段、TF、`/odometry/filtered`、costmap/ESDF、人群状态  
- 输出：`cmd_vel`  
- 输出调试：最优轨迹、最小距离、代价分解、求解耗时（P50/P95/P99）

#### 4.2.7 参考轨迹构建：从 Nav2 Path 到 MPC 参考（常被忽略但决定效果上限）
> MPC 的效果很大程度取决于“参考怎么喂”。如果参考曲率/航向不连续，控制会天然抖。

**建议做法（工程版）**
- 输入：Nav2 给的离散 `nav_msgs/Path`（通常在 `map`）  
- 步骤：
  1) 变换到 `base_link` 或 `odom`（局部化，减小数值尺度）
  2) 按弧长重采样（固定间距，例如 0.05–0.10m）
  3) 计算切向航向 `theta_ref` 与曲率 `kappa_ref`（可平滑）
  4) 做时间参数化（简单起步：`v_ref = min(v_max, v_profile(s))`；后续可引入曲率限速：`v_ref <= w_max / |kappa|`）
- 输出：长度为 `N` 的参考序列（`x_ref[i], y_ref[i], theta_ref[i], v_ref[i]`）

#### 4.2.8 障碍处理：costmap vs ESDF（两种都能做，但工程体验差很多）
**用 costmap（离散）**
- 优点：接入快，和 Nav2 对齐  
- 缺点：不可导、栅格效应强，优化容易“抖”
- 工程建议：用 costmap 时更像“采样型 MPC/MPPI”而不是梯度型 NMPC

**用 ESDF（连续且可导，推荐给 acados/NMPC）**
- 定义距离函数 `d(x,y)`：到最近障碍的距离（>0 安全）  
- 安全约束：`d(x,y) >= d_min`  
- 软约束（常用）：`penalty = w * max(0, d_min - d)^2`（或 barrier/log 形式）  
- 关键：ESDF 查询必须快（见 4.3 的工程建议）

#### 4.2.9 社会代价（推荐形式）：非对称椭圆势场 + 速度方向拉伸
> 目标是“可解释、可调参、可回放”。建议先用简单形式跑通，再逐步增强（群体、过道规则等）。

**单人势场（示意）**
- 设行人位置 `p_h`、速度方向单位向量 `e_h`，机器人位置 `p`  
- 在行人坐标系中定义前/侧/后不同的尺度（椭圆）：
  - 前方尺度 `sigma_front`，侧向 `sigma_side`，后方 `sigma_back`
- 代价可用高斯/指数：
  - `J_social(p) = w * exp(-0.5 * q(p))`
  - `q(p)` 是椭圆二次型（前向轴沿 `e_h`）
- 速度拉伸：`sigma_front = sigma_front0 + k * |v_h|`（人走得快，前向更敏感）

**MPC 中的用法**
- 将 `J_social(p_i)` 加入每个预测步的 stage cost  
- 同时保留硬安全约束（例如 ESDF 距离），避免把“舒适”当成“安全”

#### 4.2.10 数值稳定与调参顺序（实战经验：先保证可行，再谈优雅）
建议你按以下顺序加复杂度（每一步都能回归）：
1) 仅路径跟踪 + 控制平滑（无障碍、无社会）  
2) 加速度/角速度/角加速度约束（与你车体一致）  
3) 加障碍距离（先 costmap，后 ESDF）  
4) 加社会代价（先真值人群，再接跟踪输出）  
5) 加不可行处理（松弛变量 + 降级策略）

每一步都输出：
- `solve_ms` 分位数、最小距离、人机最小距离、jerk 与急停次数、失败/降级计数

#### 4.2.11 学习路线建议（CasADi → acados，面向“能落地到 50Hz”）
- CasADi：先把模型、约束、代价建对（允许慢）  
- 迁移到 acados：固定 `dt,N` 与缩放（state/control scaling），加入 warm start 与超时策略  
- 最后做插件化：把“参考构建/ESDF查询/人群编码/求解器”拆成独立模块（SRP），便于性能剖析与替换

---

### 4.3 TSDF/ESDF：用于连续距离查询与可导代价（尤其利于 MPC）

#### 4.3.1 你在地面差速导航中“正确使用 TSDF/ESDF”的三种路径
**路径 A（最稳）：全局仍 2D，局部用 ESDF**
- 全局规划：Nav2 2D planner（稳定、工程成熟）  
- 局部控制：MPC/MPPI/RL 使用 ESDF 做距离/梯度，避障更平滑  
- 推荐你优先采用（ROI 最高）

**路径 B（进阶）：从 TSDF 提取 traversability 投影到 2D**
- TSDF/点云提取坡度/粗糙度 → 2D traversability grid  
- Nav2 在该 grid 上规划，能避开不可通行区域

**路径 C（重度）：直接 3D/2.5D 全局规划**
- 通常需要自研 planner 插件与数据结构，成本高，建议作为研究方向而非早期工程主线

#### 4.3.2 ESDF 与社会导航的结合建议
社会代价（人机距离）与 ESDF（障碍距离）是两个不同距离场：
- ESDF：对静态/半静态障碍的几何安全  
- Social field：对行人/群体的舒适与交互  
在 MPC/RL 中建议两者都进代价函数，但权重要可解释、可调参。

#### 4.3.3 TSDF/ESDF 实现选型（GPU/CPU 两条线，按你“训练GPU、推理多端”思路）
> 你要同时考虑：实时性、部署环境、与现有传感器/定位链路的耦合成本。

**GPU 路线（吞吐高，适合稠密点云与频繁更新）**
- 典型：nvblox/自研 CUDA TSDF（思路：体素融合 + ESDF 派生）  
- 优点：更新快，可用于更高频局部距离场  
- 风险：依赖 CUDA/驱动栈，版本更新要做好容器化与锁定

**CPU 路线（部署广，易维护）**
- 典型：voxblox/openvdb（思路类似，但频率一般更低）  
- 优点：部署轻，回归更容易  
- 风险：点云密度高/地图大时 CPU 压力明显，需要更激进的降采样与局部窗口

**工程建议**
- 初期（进阶前半段）：先用 2D costmap + 点云体素层跑通社会导航与评测  
- 资深阶段：再把 ESDF 引入 MPC/RL 的“局部优化”，先局部窗口，再考虑全局 TSDF

#### 4.3.4 ESDF 查询的工程优化（否则 50Hz NMPC 会被查询拖死）
**核心目标**：把“每周期的 ESDF 查询量”控制在可预测范围内。

常用策略：
- 只维护机器人附近局部窗口（例如 10×10m×2m），随机器人滚动  
- 预取：每周期先把局部 ESDF 栅格裁剪到连续数组，后续查询 O(1) 插值  
- 预测点数控制：MPC 只在预测轨迹的关键点查询距离（例如每步 1 次或每两步 1 次）  
- 对社会代价与 ESDF 代价分层：ESDF 用于硬安全与近场，社会代价可用更低频更新

**建议输出的 debug**
- `esdf_query_count` / `esdf_query_ms`  
- `min_esdf`（预测轨迹最小距离）  
- `esdf_window_age`（窗口更新时间，过旧要降级）

---

### 4.4 MINCO：用于“平滑与时间参数化”，而非替代全局规划

#### 4.4.1 你在差速地面车上的最佳用法
把 MINCO 当作：
- 全局路径离散点 → 连续平滑曲线（降低曲率突变）  
- 输出可跟踪的参考（曲线 + 速度时间参数化）  
然后由 MPC/MPPI/RL 跟踪并处理动态障碍与社会约束。

#### 4.4.2 差速关键约束（否则平滑会害你“擦边/甩头”）
差速关系：`w = v * kappa`  
在 `v_max=1.0`、`w_max=1.0` 时，曲率上限约 `kappa_max≈1.0 1/m`。
- 平滑器必须考虑曲率上限，否则局部控制会被迫饱和（导致抖动与贴障）  
- 在窄门/贴边区域，平滑应结合障碍距离（最好用 ESDF 做约束/惩罚）

#### 4.4.3 MINCO 在地面车上的“正确角色”：做参考轨迹，而不是做避障决策
建议你把职责拆开：
- **规划**：负责“从 A 到 B 走哪条路”（Nav2 planner）  
- **平滑/时间参数化（MINCO）**：负责“把离散路变成可跟踪的连续参考”（限制曲率/jerk）  
- **局部控制（MPPI/MPC/RL）**：负责“在动态障碍与社会约束下跟踪参考并实时修正”

这样做的好处：
- MINCO 不需要直接面对动态障碍（避免复杂约束导致优化不稳定）  
- 局部控制可以统一处理静态/动态/社会约束（SRP 更清晰）

#### 4.4.4 如果你想把 MINCO 做到“贴近最优”：引入 ESDF 生成安全走廊（选做）
> 这是资深玩法：先从 ESDF 生成沿路的“可行走廊”，再在走廊内做多项式优化，能显著减少擦边与不必要的保守。

工程化思路（概念）：
1) 将离散路径分段  
2) 对每段在 ESDF 上扩张出局部凸区域（2D 可用矩形/多边形近似）  
3) 在走廊约束内做 MINCO 优化（位置连续、速度/加速度/jerk 平滑）  
4) 加曲率约束（或等效地限制 `w`）  

风险与建议：
- 走廊生成与优化计算可能变重，建议降低频率（例如 1–5Hz）  
- 走廊失败要有退路（直接用 Nav2 smoother 或原始 path）

---

### 4.5 全局规划与轨迹生成进阶（Smac/TSDF投影/时间参数化）
> 你的系统里“全局规划”负责产出拓扑可行路径，“局部控制/优化”负责在动态与社会约束下执行。别把局部问题塞进全局规划，否则会难以实时与回归。

#### 4.5.1 Nav2 Planner 插件化点（你未来要扩展的接口）
Nav2 全局规划在 `planner_server` 中，以 `nav2_core::GlobalPlanner` 插件形式运行：
- 输入：起点/终点（通常在 `map`）、2D costmap  
- 输出：`nav_msgs/Path`

工程建议：
- 先用 Smac/Theta* 建立 baseline  
- 若要引入 TSDF/地形信息，优先做“2D traversability 投影”而不是直接 3D 规划

#### 4.5.2 Smac vs Theta*（差速地面车的常用取舍）
- `SmacPlanner2D`：稳定、工程成熟，适合栅格地图与成本代价  
- `Theta*`：路径更直、更少折线，对后续平滑与局部控制更友好  
- 经验：走廊窄/障碍密时 Smac 更稳；开阔区 Theta* 更自然

#### 4.5.3 TSDF/地形信息如何进入全局规划（推荐“投影+融合”）
如果你有 TSDF/点云地形信息（坡度/台阶/粗糙度），建议生成 2D traversability 代价图并融合进 costmap：
- 对每个 (x,y) 栅格，统计高度方差/最大坡度/可通行概率  
- 转为“额外 costmap layer”的 cost（traversability layer）  
- 与 static/obstacle/inflation 融合后交给 planner

这样 planner 仍工作在 2D，但能避开“物理不可通行区域”，也更利于回归评测。

#### 4.5.4 时间参数化（把“路径”变成“可执行参考”）
不论你是否用 MINCO，都建议做时间参数化（尤其你要评测舒适与社会行为）：
- 曲率限速：`v <= w_max / |kappa|`  
- 障碍/人群限速：近场距离小则降低 `v_ref`  
- 输出给 MPC/RL：参考 `v_ref` 能显著减少抖动与不必要急停

---

## 5. 50Hz 实时与稳定性：性能预算与防抖

### 5.1 频率建议（可作为默认起点）
- `controller_frequency`：50Hz（你的目标）  
- costmap 更新：10–20Hz（过高会浪费算力，过低会迟钝）  
- LIO：尽量与 LiDAR 频率匹配（常见 10–20Hz 或更高）  
- `robot_localization`：50–100Hz（输出给控制器更平滑）

### 5.2 延迟预算（你应该用数据说话）
对 50Hz 控制（20ms 周期），建议把“感知→控制”端到端延迟控制在：
- 目标：<40ms（2 个控制周期内）  
并记录 P50/P95/P99：
- 推理耗时（TRT/OV/CPU）  
- MPC 求解耗时（acados）  
- costmap 更新耗时  
- TF 查询耗时

### 5.3 防抖与降级（工程必需）
任何一个子系统偶发卡顿都不能把整车带崩：
- 推理/求解超时：降级到保守控制（RPP/低速 MPPI）或直接刹停  
- costmap 未更新：限速 + 触发 BT 恢复/等待  
- TF 过期：刹停并报警（日志 + 统计）

### 5.4 性能剖析与验收清单（你应该拿到“数字”，而不是感觉）
**频率验收**
```bash
ros2 topic hz /cmd_vel
ros2 topic hz /odometry/filtered
ros2 topic hz /points_raw
```

**时延/滞后排查（回放也能做）**
- 观察：控制输出是否落后于障碍更新（常见表现：看见障碍但仍向前冲一小段）
- 做法：录包后用 RViz2 对齐 `/points_raw` 与 `/cmd_vel` 时间戳，或在你的控制器 debug 中输出 `now - last_costmap_update_stamp`

**TF 健康检查**
```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
```
期望：
- echo 输出连续稳定；不应频繁出现 lookup timeout / extrapolation

**控制器/求解器耗时（必须输出分位数）**
- RL 推理：`infer_ms` 的 P95 应显著小于 20ms（建议 <5ms 作为目标）  
- NMPC 求解：`solve_ms` 的 P95 必须小于 20ms（建议留 30–50% 余量给特征/TF/costmap）

---

## 6. 评测体系：场景矩阵、指标、录包/回放、报告

### 6.1 场景矩阵（建议至少覆盖）
**静态结构**
- 窄门/窄走廊、S 弯、回字走廊、死胡同、开阔区长距离  

**动态人群**
- 迎面会车/会人、横穿、近距离切入、拥挤随机游走、成组行走/队列

**退化与异常**
- 点云丢帧/延迟抖动、IMU bias 增大、wheel slip 增大、定位偶发跳变  

### 6.2 指标（建议做成自动统计）
**导航能力**
- 到达率、平均耗时、路径长度、重规划次数、恢复触发次数

**安全**
- 碰撞次数、最小障碍距离、最小人机距离、TTC 分布

**舒适**
- 加速度峰值、角加速度峰值、jerk（线/角）、急停次数

**实时**
- 控制回路耗时（P50/P95/P99）、推理/求解耗时、costmap 更新时间

**定位**
- 位姿误差 RMSE/P95、航向误差 RMSE/P95、跳变次数、断链次数

### 6.3 录包/回放（强制执行，支撑“同包对比”）
建议分数据集版本：
- v1：静态基础场景（入门）  
- v2：动态人群（进阶）  
- v3：退化注入（资深）

录包应包含：
- 时钟与 TF：`/clock /tf /tf_static`  
- 传感器：`/points_raw /imu /scan(如有)`  
- 里程计/定位：`/odom /odometry/filtered`、LIO 输出、全局 pose 输出  
- 导航：`/cmd_vel`、全局/局部路径、costmap（或 debug 输出）、BT 状态（如可得）  
- 人群：感知检测、跟踪输出、人群真值（用于离线对齐评测）

### 6.4 同包对比（A/B）标准流程（建议你把它当成团队规范）
**原则**：一次只改变一个变量（控制器插件/参数集/定位方案），其它全部固定。

1) 固定数据集（例如 `bags/v2_social/run_001`）  
2) 回放同一包，分别运行：
   - baseline：MPPI（teacher/对照）
   - 候选 1：你的 RL 控制器（TRT）
   - 候选 2：你的 MPC 控制器（acados）
   - 候选 3：你的 RL 控制器（OpenVINO/CPU fallback）
3) 统一输出报告（CSV/JSON + Markdown 摘要）：
   - 结果指标 + 失败原因统计 + 实时耗时分位数
4) 把“失败原因”可视化（至少包含：tf_timeout/costmap_stale/infer_timeout/solver_timeout/progress_stuck）

### 6.5 社会导航专项评测（你要“同时验证整链”的关键）
建议你把 social nav 的指标分成三类：
- **安全底线**：最小人机距离、TTC、碰撞/擦碰次数  
- **社会舒适**：侵入个人空间次数（距离阈值统计）、急停次数、绕行是否过激  
- **效率与礼让**：到达时间、让行等待总时长、是否出现“互相让到死”的僵局

---

## 7. 风险清单与排雷手册（按出现频率排序）
1) **TF 冲突**：多个节点同时发布 `map->odom` 或 `odom->base_link`，导致偶发跳变。  
2) **时间不同步**：漏配 `use_sim_time`，导致 TF extrapolation、规划控制偶发失败。  
3) **时间戳抖动/倒退**：点云 stamp 不单调，LIO/融合会爆炸。  
4) **QoS 不匹配**：点云/TF 收不到或丢包（尤其 bridge 默认 QoS）。  
5) **costmap 更新太慢**：控制 50Hz 但障碍层 1–2Hz，避障迟钝、撞人撞墙。  
6) **训练分布过于理想**：无延迟/无噪声训练，部署就抖动与贴障。  
7) **控制饱和**：平滑/轨迹曲率不考虑 `w_max`，导致局控长期饱和而抖动。  
8) **人群“鬼影”**：跟踪丢失后社会势场不衰减，机器人被虚假障碍卡死。

---

## 8. 推荐的里程碑交付物（每阶段可验收）

### M0（入门）
- Isaac Sim 场景 + ROS 2 bridge 闭环：`/clock /tf /points_raw /odom /cmd_vel` 稳定  
- Nav2 baseline（MPPI/RPP）在静态场景稳定到点  
- 数据集 v1（rosbag2）+ 回放复现说明

### M1（进阶）
- LIO + robot_localization 融合链路稳定输出（`/odometry/filtered`）  
- 定位评测报告（对照 AMCL vs LIO vs 融合）  
- 社会导航整链跑通：感知→跟踪→social layer→BT 策略（动态场景）  
- 数据集 v2（动态人群）+ 指标脚本

### M2（资深）
- 自训局控插件（ONNX + TRT/OV/CPU），可与 MPPI/RPP 同包对比  
- MPC（acados）插件化，满足 50Hz 并有超时降级  
- ESDF 接入 MPC/学习控制器（更平滑的距离约束）  
- MINCO/平滑模块接入（曲率约束正确）  
- 数据集 v3（退化注入）+ 全量回归报告

---

## 附：推荐你下一步要补齐的“工程骨架”（不涉及具体代码）
> 你已确认：LIO+融合、整链社会导航验证、50Hz。下一步建议你把工程骨架按以下三件事落地：

1) **统一接口规范（obs/spec + people msg + debug msg）**：所有控制器共享同一套输入抽象与日志输出。  
2) **统一评测流水线（bag→回放→指标→报告）**：让每次改动都有量化结果。  
3) **插件化替换策略（BT 层切换控制器/参数集）**：让“先进算法”可控上线，不是一次性赌博。

---

## 9. MuJoCo 补充路线：与 Isaac Sim 的分工与融合
> 你现在的主线是 Isaac Sim（高保真传感器+RTX+场景资产+并行训练）+ Nav2（工程导航框架）+ RL/MPC（先进局控）。  
> MuJoCo 的价值不是替代 Isaac Sim，而是补齐“控制/动力学/接触”方向的快速迭代与算法验证，并把结果再迁移回 Nav2 插件体系。

### 9.1 为什么在你这个项目里还要引入 MuJoCo
**适合 MuJoCo 的内容（建议你用它来做“控制研究的快环”）**
- 控制器验证：MPC/RL 的动作约束、延迟鲁棒、极端工况（轮滑、摩擦变化、瞬时扰动）  
- System ID / 域随机化：质量、摩擦、质心等参数扫频，得到稳定性边界  
- 高速/高频闭环实验：在不需要复杂渲染与传感器建模时，用 MuJoCo 快速跑大量实验

**不适合 MuJoCo 的内容（仍建议留给 Isaac Sim）**
- 逼真传感器（相机噪声/曝光/运动模糊、RTX LiDAR 细节）  
- 大规模资产场景、程序化人群与行为  
- 需要“感知→跟踪→社会导航”整链验证的部分

### 9.2 推荐的“双仿真”分工（强烈建议写成工程规范）
**Isaac Sim：感知与系统集成主场**
- 3D LiDAR、相机、IMU、噪声/延迟、动态人群、RT 渲染  
- 输出可回放数据集（bags/v2_social、bags/v3_degraded）

**MuJoCo：控制与动力学主场**
- 相同 kinematic/dynamic 约束（`v_max/w_max/a_max`）  
- 重点评估：控制稳定性、jerk、限幅策略、延迟/丢帧下的稳态与恢复

### 9.3 接入方式（建议统一成 ROS 2 接口，避免分叉两套代码）
**统一 ROS 2 话题契约（两端共用）**
- 输入：`/cmd_vel`（来自 Nav2 或你的 controller 插件）  
- 输出：`/odom`、`/imu`、（可选）`/joint_states`  
- TF：`odom->base_link`（由仿真或 `robot_localization` 输出，但必须避免双发布）

**建议的工程策略**
- 控制器插件不依赖具体仿真（只依赖 TF/odom/costmap/people msg 抽象）  
- MuJoCo 场景用于“控制器内核”的回归与性能压测；Isaac Sim 用于“系统整链”的回归

### 9.4 MuJoCo 专项评测（你应该额外记录的指标）
- 稳态误差：速度跟踪误差、转向误差  
- 鲁棒性：外力扰动/摩擦突变下的恢复时间  
- 控制品质：jerk、饱和占比、振荡次数  
- 延迟敏感性：动作延迟（1–3 帧）+ 观测延迟（20–80ms）下的退化曲线

---

## 10. 视觉全链路：识别/追踪/定位/动态SLAM/安全走廊
> 这一章给你一个可落地的“视觉导航子系统蓝图”，并明确它与 LIO+融合、Nav2 costmap、社会导航、MPC/RL 的接口关系。  
> 你要求“同时验证整链”，因此这里既包含“真值快速验证路径”，也包含“真实可部署路径”。

### 10.1 视觉子系统在你项目中的定位（先把边界画清楚）
视觉在导航里常见承担四类职责（你不必一次全上，按收益排序）：
1) **检测/追踪人群与动态障碍**：社会导航与安全绕行（ROI 最高）  
2) **视觉里程计/VIO**：在 LiDAR 退化（玻璃、雨雾、稀疏纹理）时补位  
3) **动态 SLAM / 语义建图**：把“动态”从地图里剔除，并提供语义可通行信息  
4) **安全走廊/可通行区域（free-space）**：用于 MPC/MINCO 的约束与参考生成

建议你按顺序推进：先做 1，再做 2，然后视场景决定是否需要 3/4。

### 10.2 视觉感知：识别与追踪（Detection → Tracking → People Msg）
#### 10.2.1 最小可用链路（先跑通，再增强）
**输入（相机）**
- `/camera/image`、`/camera/camera_info`  
- （可选）`/camera/depth/image` 或 `/camera/depth/points`

**输出（建议统一）**
- `/people`：`people_msgs/People`（位置+速度+协方差）  
- `/tracks/debug`：可视化 marker 或 overlay image（便于回放）

**处理链（推荐骨架）**
1) 2D 检测：输出 bbox + class + score（人/车/自行车等）  
2) 多目标跟踪（MOT）：输出 track id + 速度估计（或由滤波器估计）  
3) 3D 定位：
   - 有深度：bbox 内取深度统计 → 得到 3D 点  
   - 无深度：用地平面假设/单目深度（难度更高，建议后置）
4) 滤波与统一：Kalman/IMM 估计速度与协方差，发布 `People`

#### 10.2.2 GPU/多端部署建议（与你 TRT/OpenVINO 目标对齐）
建议把视觉模型也统一到 ONNX，并准备：
- GPU：TensorRT（高吞吐、低延迟）  
- Intel：OpenVINO（便于 CPU/iGPU 部署）  
- CPU fallback：ONNX Runtime

工程要点：
- 固定输入分辨率与 batch=1（实时流更稳定）  
- 明确推理超时与降级：超时则只发布空 tracks（或延迟标记），social layer 做衰减而非卡死

#### 10.2.3 与社会导航的接口（必须统一）
social costmap layer 与 MPC/RL 都应订阅同一个 `/people`：
- social layer：把人群变成代价场（舒适/礼让）  
- MPC/RL：把人群作为代价/约束（避免碰撞与提升自然性）  
- 评测：用 `people/ground_truth` 对齐误差，统计误检/漏检/延迟对导航指标的影响

### 10.3 视觉定位：VIO/VSLAM 与 LIO+融合的组合策略
> 你主定位为 LIO+融合，但视觉可以做“补位”与“退化场景增强”。关键是：别让多个里程计在 TF 上互相打架。

#### 10.3.1 推荐融合思路（工程优先）
- wheel odom：高频、低延迟（给控制器稳态速度）  
- IMU：航向与角速度稳定  
- LIO：结构强、对光照不敏感（主定位）  
- VIO/VSLAM：在 LiDAR 退化/稀疏/动态遮挡时补位（或作为对照评测）

**融合输出建议**
- `robot_localization` local ekf：输出 `/odometry/filtered`（给控制器）  
- global 约束：由闭环/建图模块输出 `map->odom`（或 global ekf）  

#### 10.3.2 你必须做的“时间与外参一致性”（否则动态场景会爆）
- 相机-IMU 外参：`tf_static` 固定发布  
- 相机时间戳：与 `/clock` 同步，且与 IMU 误差 < 5ms（越小越好）  
- 如果桥接带来固定延迟：显式记录 offset，并纳入回归与训练随机化

### 10.4 动态 SLAM（Dynamic SLAM）：把“动态”从定位与地图里剔除
> 动态 SLAM 的核心不是“更花哨”，而是解决两类工程痛点：  
> 1) 动态人群导致定位漂移/跳变；2) 动态物体被写进地图导致规划误判。

#### 10.4.1 你可以采用的三种工程路径（按复杂度递增）
**路径 A：动态过滤（推荐起步，收益极高）**
- 在点云/图像侧做动态目标检测（人/车），把动态点/特征剔除  
- LIO/VSLAM 只用“静态子集”做匹配  
- 地图更新也只融合静态部分（减少“鬼影”）

**路径 B：语义辅助 SLAM**
- 使用语义分割 mask（人、车等）对特征点加权/屏蔽  
- 优势：对拥挤环境更稳  
- 风险：语义模型误检会伤害定位，需要协方差/置信度管理

**路径 C：显式建模动态对象（研究向）**
- 同时估计静态地图与动态对象轨迹（Multi-body SLAM）  
- 工程复杂度高，建议在你的主线稳定后再探索

#### 10.4.2 动态 SLAM 的评测指标（务必加入你的报告）
- 定位：RMSE/P95、跳变次数（动态场景前后对比）  
- 地图：动态鬼影占比（可用“与真值静态地图差异”评估）  
- 导航：在相同人群场景下的到达率/碰撞率/舒适性提升

### 10.5 安全走廊（Safety Corridor）：把“可行区域”变成控制/优化的硬约束
> 你提到“安全走廊”，它是把路径执行从“软惩罚”提升到“几何约束”的关键技术，尤其适合 NMPC/MINCO。

#### 10.5.1 2D 地面车的安全走廊（推荐工程实现）
输入：
- 全局路径 `nav_msgs/Path`
- 局部 costmap（或 ESDF）

输出（两类都可，建议都提供）：
- 走廊多边形序列（每段一个凸多边形/矩形/圆柱近似）  
- 或走廊中心线 + 左右边界距离（便于 MPC 约束化）

生成思路（工程可落地版本）：
1) 路径分段（按弧长每 0.5–1.0m 一段）  
2) 对每段在 costmap/ESDF 上做“最大可扩张区域”近似（矩形/多边形）  
3) 加入机器人半径与安全裕度（含定位误差裕度）  
4) 输出给：
   - MPC：作为硬/软约束（保持在走廊内）  
   - MINCO：作为优化约束（见 4.4.4）  
   - 可视化：RViz marker，便于回放分析

#### 10.5.2 动态人群下的走廊策略（避免“走廊抖动”）
动态障碍会导致走廊随时变化，工程上建议：
- 走廊生成频率低于控制频率（例如 1–5Hz）  
- 走廊更新做滞回：只有变化超过阈值才更新  
- 人群进入走廊时：优先交给控制器（MPPI/MPC/RL）处理，走廊仅提供“静态安全边界”

#### 10.5.3 走廊与定位误差的耦合（你既然要评测定位，这一条很关键）
走廊膨胀应显式考虑定位误差：
- 走廊有效半径 = 几何安全半径 + 定位 P95 误差 + 速度相关裕度  
否则会出现：定位略漂移 → 控制器看似在走廊内但实际擦边/碰撞。

---

## 11. 分层学习路径（入门→进阶→资深）与时间规划
> 这一节是把你给的参考方案“工程化落地”，并和本文件的章节对应起来，便于你按月推进与自检。

### 11.1 技术栈总览（Level 1/2/3）
**Level 1：入门（0–3 个月）**
- ROS 2 基础：Node/Topic/Service/Action、TF2、Launch、Param、Lifecycle、QoS  
- Nav2 核心概念：Servers、BT、costmap、常用 Action（`/navigate_to_pose` 等）  
- Gazebo（或轻量仿真）打底：URDF、传感器插件、建图/定位/导航  
- 本文件对应章节：第 2 章（入门闭环）+ 第 6 章（录包回放规范）

**Level 2：进阶（3–6 个月）**
- Isaac Sim：USD/Action Graph/ROS2 Bridge，3D LiDAR + 动态场景  
- Nav2 高级：MPPI、Smac、性能剖析与 Benchmark（同包对比）  
- 定位主线：LIO + robot_localization 融合 + 定量评测  
- 社会导航整链：感知→跟踪→social layer→BT 策略  
- 本文件对应章节：第 3 章 + 第 5 章 + 第 6 章

**Level 3：资深（6–12 个月+）**
- RL 导航：Isaac Lab 并行训练、域随机化、Sim2Real  
- Nav2 C++ 插件：controller/planner/smoother layer 插件化与生命周期工程  
- 模型部署：ONNX + TensorRT/OpenVINO/CPU fallback + 超时降级  
- MPC：CasADi→acados 50Hz 实时化，ESDF/社会代价/安全走廊约束  
- TSDF/ESDF、MINCO：连续距离场与平滑/走廊优化  
- MuJoCo：控制/动力学快环与鲁棒性边界验证  
- 本文件对应章节：第 4 章 + 第 9–10 章

### 11.2 时间投入估算（可作为你的 roadmap）
| 阶段 | 预计时间 | 主要产出（可验收） |
|---|---:|---|
| Level 1 入门 | 1–3 月 | Gazebo/基础仿真 + Nav2 稳定建图/导航；能录包回放与调参 |
| Level 2 进阶 | 3–6 月 | Isaac Sim + MPPI/Smac 调优；LIO+融合定位评测；社会导航整链跑通 |
| Level 3 资深 | 6–12 月+ | RL/MPC 控制器作为 Nav2 插件上线（TRT/OV/CPU）；ESDF+安全走廊；实机/量产策略 |

### 11.3 Level Checkpoint（每阶段你该能回答的问题）
**Level 1**
- 我能解释并画出 `map->odom->base_link->sensor` TF 链吗？  
- 我能通过修改 `inflation_radius/update_frequency` 定向改变行为吗？  

**Level 2**
- 我能给出定位误差（RMSE/P95）并解释它如何影响到达率/碰撞率吗？  
- 我能在同一 rosbag 上对比 MPPI vs 自己参数/策略的差异吗？  
- 我能解释 social layer/BT/控制器三者的职责边界吗？

**Level 3**
- 我的 RL/MPC 插件在 50Hz 下 `P95 total_ms` 有数据吗？超时如何降级？  
- 我的模型工件（ONNX + meta）能在 TRT/OV/CPU 三端跑一致的输入输出吗？  
- 我能在退化注入（延迟/误检/定位漂移）下给出性能退化曲线吗？

---

## 12. 自研“感知-定位-规划-控制（归控）”方案（Batch-LIWO × 哨兵2025）
> 本章把 `doc/Batch-LIWO.pdf` 与 `doc/中科大哨兵2025技术报告.pdf` 的关键思想抽取成**可工程落地**的“自研归控”方案，并明确：  
> - **实机为主**：算法设计以真实机器人可部署、可回归为首要目标；  
> - **仿真为辅**：Isaac Sim / MuJoCo 用作预研与验证环节（造数据、做退化、做回归），不让仿真绑架工程。

### 12.1 两份参考材料的“可复用技术要点”提炼（只取能直接落地的部分）
#### 12.1.1 Batch-LIWO / Point-LIWO 的要点（定位链路）
从 `Batch-LIWO.pdf` 可以抽象出三条“工程级关键结论”：
1) **Batch-Wise 更新替代 Point-Wise**：把短时间窗口内点云打包成 batch，在 batch 内完成运动传播/去畸变/残差构建，再做一次滤波更新。  
   - 价值：减少更新次数、易并行、对噪点/退化更稳、吞吐更高。  
2) **轮速作为紧耦合观测源 + 自适应协方差**：把 wheel speed 作为额外观测输入到紧耦合滤波框架，并用“残差 RMS”度量轮速置信度（打滑/离地时自动降权）。  
   - 价值：LiDAR/IMU 退化时仍有稳定速度约束；轮速退化时又不会把系统带崩。  
3) **嵌入式观测器/底盘侧鲁棒速度估计**：在资源受限平台上提供可实时运行的速度观测与退化度量（尤其轮滑）。  
   - 价值：为后端规划/控制提供稳定输入，并为“安全限速/功率控制/牵引控制”提供量化依据。

#### 12.1.2 “哨兵2025技术报告”的要点（感知-规划-控制链路）
从 `中科大哨兵2025技术报告.pdf` 可抽象出一条“平坦、模块少、强实时”的链路范式：
1) **以机器人为中心的 Occupancy Grid（ROG-Map 思路）**：本地栅格，支持 fading（遗忘），并行化实现。  
2) **以 2D ESDF 作为规划/控制前端**：用 ESDF 提供可调的避障梯度与距离信息，且对动态障碍可高频更新。  
3) **MINCO + L-BFGS 做轨迹表示与优化**：高效、可重规划，并通过二次插值/两阶段优化应对狭窄地形梯度震荡，提高稳定性与速度。  
4) **SE2 MPC 做控制器**：目标是“牢牢贴合规划轨迹”，并通过“沿轨迹/横向”两种行为模式应对不同跟踪需求；控制器可消费“控制点附近占据情况”决定是否强制跟随航向。  

> 你要的“现代+传统路线”，核心就是：  
> - 传统：Batch-LIWO（紧耦合滤波）+ ESDF + MINCO + SE2 MPC（可解释、可约束、强实时）  
> - 现代：视觉检测/追踪/动态过滤、学习式预测/社交代价、RL 作为可插拔局控候选或 teacher、TRT/OpenVINO 多端部署

---

### 12.2 自研归控总体目标与工程约束（先把边界写死）
#### 12.2.1 目标（可验收）
- 50Hz 控制闭环稳定运行（`/cmd_vel` 输出），P95 控制周期总耗时显著小于 20ms  
- 3D LiDAR + IMU + Wheel 的鲁棒里程计，在退化场景（点云缺失/IMU饱和/轮滑）下稳定不崩  
- 动态人群场景下：到达率、最小人机距离、急停次数、TTC 分布可量化优于 baseline（或至少不差）  
- 能在 Isaac Sim 复现实机 bug（通过 rosbag + 退化注入），形成可回归数据集 v2/v3

#### 12.2.2 约束（按你已给定 + 必要补充）
- 机器人：差速，`v_max=1.0m/s`，`w_max=1.0rad/s`，`a_max≈1.0m/s²`  
- 传感器：
  - 3D LiDAR：10Hz，非重复扫描（固态/扇形 FoV 类），以“连续点流”的方式参与里程计与建图  
  - wheel：轮速可提供时间戳（可用于与 IMU/LiDAR 对齐、以及退化度量输出）  
  - 视觉：RGBD，硬件同步（用于检测/追踪、近场占据/动态过滤，必要时补位 VIO/动态SLAM）  
- 定位主线：LIO + `robot_localization` 融合  
- 社会导航：必须验证“感知→跟踪→social cost→规划/控制”整链  
- 计算平台：Intel 为主（推理优先 OpenVINO；保留 CPU fallback；GPU/TRT 仅作为可选加速路径）  
- 地图表示：以地面车为主，**局部 3D ESDF**（近场可导距离场）+ 2D 投影/切片（供 Nav2 costmap/可视化）  
- 安全走廊定位：作为 **BT 的窄通道保底策略**（低频构建，失败可退回 ESDF/MPC/MPPI）  
- 实机落地：需要上实机（因此必须有：诊断与日志、参数锁定、版本/模型工件管理、可回归数据集）

---

### 12.3 分层系统架构（自研模块 + Nav2 编排）
> 这里给你两种集成方式。建议先做 A（最小改动、回归最强），成熟后再探索 B。

#### 12.3.1 方案 A（推荐）：Nav2 做“编排与接口”，自研做“核心算法插件/节点”
**Nav2 保留**
- `bt_navigator`：任务编排、恢复、切换控制器/参数集  
- `planner_server`：全局规划（先 Smac/Theta*，后可替换 traversability）  
- `collision_monitor` + `velocity_smoother`：系统级安全兜底（永不下线）

**自研替换/新增**
1) 自研里程计：`batch_liwo_node`（ROS2 node）  
2) 自研感知前端：`rog_esdf_node`（ROS2 node）  
3) 自研轨迹规划：`minco_planner_node`（ROS2 node 或 planner plugin）  
4) 自研控制器：`se2_mpc_controller`（Nav2 controller plugin）  
5)（可选）RL 控制器：`rl_controller`（Nav2 controller plugin，可与 MPC/MPPI A/B）

**核心优点**
- 仍使用 Nav2 的 Action/BT/恢复体系（工程成熟）  
- 自研模块可逐个替换与回归（一次只动一块）

#### 12.3.2 方案 B（进阶）：Nav2 仅做 Action 门面，自研规划与控制在外部闭环
Nav2 只保留 `/navigate_to_pose` Action 接口与 BT 状态机；规划/控制主回路由自研节点完成。  
优点：自由度最高；缺点：你要自己承担更多“恢复/异常/状态机”工程成本（不建议初期就走）。

---

### 12.4 自研里程计：Batch-LIWO（LIO+Wheel）在你的系统里的落地方式
#### 12.4.1 ROS2 I/O 契约（建议你写成接口文档并锁版本）
**订阅**
- `/points_raw`（PointCloud2，`lidar_link`）  
- `/imu`（Imu，`imu_link`）  
- `/wheel/odom`（Odometry 或自定义，含轮速/编码器/时间戳）  

**发布**
- `/lio/odom`（nav_msgs/Odometry，建议在 `odom` frame）  
- `/liwo/quality`（自定义或 diagnostic）：包含退化度量（点云可用度、IMU 饱和、轮滑 RMS、观测权重等）  

**强制要求**
- 时间戳与 `/clock` 一致（仿真）或与硬件时钟一致（实机），且单调递增  
- 明确谁发布 `odom->base_link`、谁发布 `map->odom`，避免 TF 冲突（见 3.1.3/3.1.4）

#### 12.4.2 “轮速自适应协方差”在工程上的用法（不仅用于 LIO）
你应该把 wheel slip/退化度量输出出来，作为全系统的“安全信号”：
- 规划限速：轮滑严重 → 降低 `v_max/a_max`  
- 控制器模式：轮滑严重 → MPC 权重调整（更保守）或切换到更稳健控制器  
- 地图更新：姿态不可信时降低地图融合权重（避免把错误位姿写进地图）

#### 12.4.3 退化场景的验证用例（建议你做成固定回归数据集 v3）
- LiDAR 退化：点云缺失扇区/随机丢帧/强噪声  
- IMU 退化：饱和、bias 漂移、延迟抖动  
- wheel 退化：打滑、离地（仿真可人为设置摩擦突变/轮子悬空）  
- 验收：里程计不发散、TF 不跳变、控制不振荡、到达率不崩盘

---

### 12.5 自研感知：ROG Occupancy + 动态目标融合 + ESDF（规划/控制前端）
#### 12.5.1 核心思想（与 Nav2 costmap 的关系）
- Nav2 costmap 是“导航框架通用表示”；  
- 你的自研感知模块输出的 Occupancy/ESDF 是“规划/控制可直接消费的高频前端”。  

推荐策略：
- 继续维护 Nav2 local/global costmap（保证 BT/恢复/可视化与系统兜底）  
- 同时维护自研 `local_occupancy` 与 **局部 `esdf_3d`**（供 MINCO/MPC/RL 使用，并可切片生成 `esdf_2d`）

#### 12.5.2 感知输入与输出（把动态与静态分开，这是动态SLAM/社会导航的根）
**输入**
- 点云：`/points_filtered`（建议由点云预处理节点提供）  
- 视觉/雷达跟踪：`/people`（统一成 `people_msgs/People`）  
- （可选）静态先验地图：`/map`（2D occupancy）或离线 ESDF

**输出**
- `/local_occupancy`：机器人中心局部 Occupancy Grid（高频）  
- `/esdf_3d`：局部 3D ESDF（高频，含距离与梯度查询接口，优先给 MPC/MINCO）  
- `/esdf_2d`：由 `esdf_3d` 投影/切片得到的 2D ESDF（给 2D 规划/可视化/兼容组件）  
- `/dynamic_occupancy`：动态层（来自 people/tracks，带 fading）  
- `/static_occupancy`：静态层（来自点云/先验地图，慢变化）

#### 12.5.3 fading（遗忘）机制：工程上必须有
动态场景里如果不做 fading，会出现两类致命问题：
- “鬼影”长期占据，导致规划莫名其妙卡死  
- 动态物体被写进静态地图，导致全局规划长期偏航

建议参数化：
- `decay_time_ms`：多久没观测到就衰减  
- `min_hit_count`：需要累计多少次观测才认为是稳定占据  
- `dynamic_priority`：动态层优先级（动态优先清除/覆盖静态）

#### 12.5.4 ESDF 构建与查询：给 MINCO/MPC 的“数值友好”接口
**关键工程点**
- ESDF 的梯度插值要稳定（双线性/二次插值），否则狭窄地形会出现梯度震荡  
- ESDF 更新频率与控制频率解耦：ESDF 可 10–30Hz，控制 50Hz（控制侧用最近一次 ESDF）

建议提供两种 API（即使内部实现不是服务，也要语义一致）：
- `distance(x,y)`：返回最近障碍距离  
- `grad(x,y)`：返回距离梯度（用于优化）

#### 12.5.5 局部 3D ESDF 的推荐实现形态（面向室内+狭窄地形+Intel算力）
> 你的主场景是室内（含狭窄走廊/隧道类难点），且以 Intel 平台为主。工程上建议“局部窗口 + 体素网格 + 可控频率”，避免全局 3D 地图带来的算力与维护成本。

**局部窗口（建议默认值，可按机器人尺寸与速度调整）**
- 空间范围：以 `base_link` 为中心的局部立方体/长方体窗口  
  - `x,y`: 10m × 10m（或 8m × 8m 起步）  
  - `z`: 2m（例如 `[-0.5m, +1.5m]` 覆盖地面到高障碍）  
- 分辨率：0.05–0.10m（狭窄地形建议更细，但 CPU 压力上升）  
- 更新频率：10–20Hz（控制 50Hz 使用最近一次 ESDF）

**传感器融合建议（LiDAR 远场 + RGBD 近场互补）**
- 3D LiDAR（10Hz 非重复扫描）：更适合 3–10m 的结构与墙面约束  
- RGBD：更适合 0.3–3m 的近场细节（桌椅腿、人腿、玻璃边缘等），也更适合“动态过滤/语义”  
- 融合方式：统一落到同一个局部 occupancy（静态/动态分层），再派生 ESDF

**切片/投影生成 `esdf_2d`（供 2D 组件使用）**
- 方式 A（常用）：对 `z∈[z_min, z_max]` 做 min-reduction（取最小距离/最大占据）  
- 方式 B（狭窄地形更稳）：按机器人高度/体积做“膨胀切片”，把侧壁/上方结构投影成更保守的 2D 代价

**动态层处理（与社会导航一致）**
- people/tracks 进入 dynamic occupancy，设置较短衰减时间  
- 静态点云进入 static occupancy，衰减更慢或不衰减  
- ESDF 派生时两层都参与，但动态层可设置“更大安全距离/更高代价权重”

---

#### 12.5.6 RGBD（硬件同步）在“感知→ESDF→社会导航”中的具体用法（给你可直接照做的落地模板）
> 你已确认 RGBD 且硬件同步。工程上建议把 RGBD 的价值聚焦在两件事：  
> 1) 近场占据补全（LiDAR 10Hz + 非重复扫描在近场细节上不一定够密）；2) 动态目标（人）3D 定位与追踪稳定性。

**A. 近场占据补全（推荐优先落地）**
- 输入：`/camera/depth/image` + `camera_info`（或直接 `/camera/depth/points`）  
- 处理：深度点云降采样 + 地面/桌面剔除 + ROI（近场 0.3–3m）  
- 输出：以 `base_link` 为中心的近场 `static_occupancy` 增量（只补近场，不和 LiDAR 抢远场）
- 典型收益：桌椅腿、玻璃边缘、低矮障碍更容易被 ESDF 感知，狭窄地形贴边更稳

**B. 人体 3D 定位（用于 `/people` 的深度支撑）**
- 输入：RGB 检测 bbox + 深度图（硬件同步降低时差）  
- 处理：bbox 内深度做鲁棒统计（中位数/分位数），得到人中心 3D 点；再做跟踪滤波（Kalman/IMM）  
- 输出：`/people`（位置+速度+协方差），并带上 `header.stamp`（用于 social layer 的时间衰减）

**C. 动态过滤（Dynamic SLAM 的最小形态，强烈建议做）**
- 用 `/people` 的 3D 位置在点云/深度点中剔除动态区域（或降低权重）  
- 目的：减少动态点被写进静态 occupancy/ESDF，降低“鬼影”与定位漂移

**D. Intel + OpenVINO 的落地建议（让你在无GPU也能跑得动）**
- 检测模型：优先选“输入固定、batch=1、INT8/FP16 友好”的结构（工程上比追求 SOTA 更重要）  
- 推理策略：
  - 固定输入分辨率（例如 640×384 或 640×480），减少运行时重新编译/重排带来的延迟尖峰  
  - 推理线程与 ROS 回调线程隔离（避免 callback 阻塞导致 TF/控制抖动）  
  - 输出超时：检测超时不阻塞整链，允许短时无 `/people`，由 social layer 的 fading 补偿

**E. 你可以直接做的“最小验证案例”（RGBD 真值 → 人体跟踪 → social cost）**
1) 在 Isaac Sim 放 5–20 个行人随机走动（提供 ground truth）  
2) 用 RGBD 检测/跟踪输出 `/people`（加入误检/漏检/延迟）  
3) social layer + MPC/MPPI 对比：最小人机距离、急停次数、TTC、到达时间  
4) 录包回放：同一 bag 复现并对比不同模型/阈值/衰减参数

### 12.6 规划：ESDF + MINCO + L-BFGS（含狭窄地形稳定策略）
#### 12.6.1 为什么用 MINCO（在你的系统里它扮演什么角色）
MINCO 在这里扮演“高效、可重规划的轨迹表示与优化器”：
- 输入：起点状态、目标状态、ESDF（距离/梯度）、（可选）先验路径（Nav2 path/JPS path）  
- 输出：带时间参数化的连续轨迹（可供 MPC 跟踪）

#### 12.6.2 前端路径与时间参数化（哨兵报告的工程经验可直接复用）
建议你把“前端路径搜索”和“后端轨迹优化”解耦：
- 前端：Nav2 planner（Smac/Theta*）或 JPS（如果你自己实现）  
- 时间参数化：梯形加减速模型 + 曲率限速（`v<=w_max/|kappa|`）  
- 宽松优化：先做一次放松碰撞/时间的 MINCO 预优化，再做严格优化（减少“扭成麻花/时间极长”的失败）

#### 12.6.3 狭窄地形梯度震荡：两条工程化对策（建议你都实现，按场景切换）
1) **二次插值/平滑梯度**：对 ESDF 梯度做更平滑的插值与滤波，避免离散栅格导致的符号翻转  
2) **两阶段优化**：先优化可行性（远离障碍、减少碰撞），再优化动力学/平滑（jerk/时间）

#### 12.6.4 动态障碍处理：建议“动态 ESDF”而不是“高频安全走廊”
结合哨兵报告的经验与 50Hz 约束：
- 动态障碍更适合用“动态 ESDF/动态代价”处理（更新快、实现简单、对控制器友好）  
- 安全走廊适合低频构建（1–5Hz）作为几何边界或窄通道保底，而不是每个预测步都重建

#### 12.6.5 安全走廊作为 BT 保底：触发条件、输出接口、与控制策略（按你“BT保底”要求固化）
> 你已明确：安全走廊用于 BT 的保底策略，而不是主规划约束。这里给你一套“可直接落地”的工程规格。

**走廊构建频率（建议）**
- 1–5Hz（低频），并带滞回（避免走廊抖动导致控制抖动）

**走廊输入（建议）**
- 当前参考路径（来自 Nav2 全局 path 或 MINCO 前端 path）  
- `esdf_3d`（或其 `esdf_2d` 切片）  
- 机器人 footprint 与定位误差裕度（把 P95 定位误差纳入走廊膨胀，见 10.5.3）

**走廊输出（建议你至少提供两种表达，便于不同模块消费）**
- `/safety_corridor/polygons`：分段凸多边形序列（可视化/BT 诊断友好）  
- `/safety_corridor/bounds`：沿路径采样点的左右边界距离（给控制器做“强约束/限速”更直接）  
- `/safety_corridor/status`：`corridor_ok`、`min_clearance`、`corridor_age_ms`（用于 BT 条件判断）

**BT 触发条件（建议作为 Condition 节点，必须日志化）**
- `is_narrow_passage`：`min_clearance < clearance_threshold` 持续 `T` 秒  
- `controller_stuck`：progress_checker 触发（卡住/抖动）  
- `localization_degraded`：`/liwo/quality` 指示低置信度（例如轮滑/点云退化）

**BT 保底行为（文字版逻辑）**
- if `is_narrow_passage`:
  - 请求构建走廊（若 `corridor_ok=false` → 退回常规 ESDF 模式 + 降速）
  - 切换到“窄通道参数集”（低速、航向锁定、更保守的障碍/社会权重）
  - 执行 `FollowCorridor`（本质仍是 FollowPath/FollowTrajectory，只是额外约束与限速更保守）
- else:
  - 走常规：ESDF+MINCO + SE2 MPC（或 MPPI）

**控制侧怎么用走廊（推荐最小实现）**
- 不要求 MPC 每步都做走廊硬约束（复杂且可能无解）  
- 推荐：走廊只用于：
  - 限速（走廊越窄 `v_ref` 越低）  
  - 航向锁定（减少横向摆动，提升通过率）  
  - 失败诊断（corridor_ok=false 或 age 过大时触发 BT 恢复）

---

### 12.7 控制：SE2 MPC（主）+ MPPI/RL（可插拔）+ 系统级安全兜底
#### 12.7.1 SE2 MPC 的工程定位
SE2 MPC 的目标不是“做规划”，而是“牢牢贴合规划轨迹”，并在约束内输出平滑稳定的 `cmd_vel`。

建议你把控制器行为拆成两个模式（与哨兵报告思路一致）：
- 沿轨迹模式：优先推进、减小 along-track 误差  
- 横向纠偏模式：优先纠偏、减小 cross-track 与航向误差  
模式切换条件应可解释、可日志化（例如误差阈值、障碍距离阈值、速度阈值）。

#### 12.7.2 控制器如何消费“占据情况”做航向跟随决策
推荐输入一个“控制点附近占据/狭窄度描述”：
- 来自 ESDF：预测轨迹点的最小距离 `min_d`  
- 或来自走廊：是否在走廊边界内、边界余量  

策略例子：
- 若 `min_d` 小于阈值（狭窄/贴边）：强制跟随轨迹航向（减少横向摆动）  
- 若环境开阔：允许更灵活的姿态（更舒适/更高效）

#### 12.7.3 兜底与降级：把“先进控制器”变成可上线组件
你至少需要三层兜底（从内到外）：
1) 控制器内部兜底：求解超时/数值异常 → 输出安全速度（或 0）  
2) BT 层兜底：切换到 MPPI/RPP baseline（保守）  
3) 系统级兜底：`collision_monitor` 强制刹停/限速

#### 12.7.4 RL 在这条自研归控路线中的“正确位置”
不建议 RL 直接取代整条链路；更建议三种用法：
1) **Teacher**：用 MPPI/MPC 生成数据，IL 训练一个轻量策略（加速部署）  
2) **候选控制器**：作为 Nav2 controller plugin 与 MPC/MPPI 同包 A/B  
3) **参数/权重自适应**：RL/学习模块输出 MPC 权重/模式切换阈值（比端到端更稳）

#### 12.7.5 Intel 平台上的 RL/MPC 推理与求解落地（50Hz 的现实约束）
> 你要上实机且以 Intel 为主，这意味着你需要提前把“无 GPU 时的可用策略”设计好，而不是临上线才发现算不动。

**RL 推理（OpenVINO 优先）**
- 目标：`infer_ms(P95) < 5ms`（给 20ms 控制周期留足余量）  
- 建议：
  - 输入特征固定 shape（避免运行期 reshape/compile 造成尖峰）  
  - 把归一化/特征拼接做成无动态内存的固定 buffer（见 4.1.7）  
  - 超时策略：一次超时就降级（切 MPPI 或 MPC 保守参数），不要连续“赌下一帧”

**MPC 求解（acados/SE2 MPC）**
- 目标：`solve_ms(P95) < 10–12ms`（其余时间留给 TF/ESDF/安全检查）  
- 建议：
  - `dt=0.02s`、`N=20~30` 起步（0.4–0.6s 时域），并做好 warm-start  
  - ESDF 查询次数严格受控（见 4.3.4、12.5.5）  
  - 不可行要用松弛变量兜底，否则狭窄地形很容易“无解→抖动”

**统一降级链（建议写入 BT 策略）**
- RL（高性能）→ MPC（强约束稳健）→ MPPI（保守 baseline）→ 刹停（collision_monitor）  
- 切换条件必须日志化（原因码 + 计数），否则无法做回归定位

---

### 12.8 仿真在本方案中的定位：预研与验证（不是主战场）
#### 12.8.1 Isaac Sim：用于系统整链与感知/动态验证
你应当用 Isaac Sim 做三件事：
1) 造动态人群与退化传感器场景（含噪声/延迟/丢帧）  
2) 生成可回归数据集（rosbag v2/v3）  
3) 在同一数据集上做 A/B：MPPI vs MPC vs RL；LIO vs LIO+wheel；ESDF 参数对比

#### 12.8.2 MuJoCo：用于控制/动力学鲁棒性边界与快回归
你应当用 MuJoCo 做两件事：
1) 轮滑/摩擦突变/外力扰动下 MPC/RL 稳定性与 jerk 的快回归  
2) 限幅与降级策略的边界测试（超时、延迟、丢测量）

#### 12.8.3 推荐的仿真验证案例（你可以直接照这个做回归集）
**案例 A：狭窄隧道 + 点云缺失（验证 Batch-LIWO + ESDF + MINCO + MPC）**
- 目的：复现“狭窄地形 + LiDAR 退化”组合下的稳定性  
- 做法：在 Isaac Sim 构建长隧道带转角；注入点云缺失扇区与随机丢帧；记录里程计稳定性与控制抖动  
- 验收：无 TF 跳变；控制不振荡；到达率显著高于无 wheel 融合的 LIO baseline

**案例 B：动态人群交互（验证 视觉/点云追踪→social cost→规划/控制）**
- 目的：验证整链在误检/漏检/延迟下的退化曲线  
- 做法：同时运行三路 people 输入：真值 / LiDAR track / 视觉 track；用同一 bag 回放对比  
- 验收：人机最小距离、TTC、急停次数可量化；并能解释“哪类误差导致哪类失败”

**案例 C：轮滑/离地（验证 wheel 退化度量 + 安全限速/控制模式）**
- 目的：验证“轮速退化不把系统带崩”的闭环策略  
- 做法：MuJoCo 或 Isaac 中设置摩擦突变与轮子悬空片段，观察 wheel RMS/quality 信号触发限速与权重调整  
- 验收：里程计不发散；控制输出自动保守；恢复后能回到正常模式

**案例 D：窄通道保底（验证“安全走廊 + BT 保底 + 降级链”可用性）**
- 目的：验证你设定的“安全走廊只做 BT 保底”的策略在狭窄地形确实提高通过率且不引入抖动  
- 做法：
  1) 构建带转角的窄走廊/隧道（宽度接近机器人 footprint + 裕度）  
  2) 注入定位轻微漂移（P95 级别）与人群短暂遮挡（模拟误检/丢 track）  
  3) 对比：关闭走廊保底 vs 开启走廊保底（其它参数不变）  
  4) 统计：到达率、擦碰次数、急停次数、`corridor_ok`/`min_clearance` 触发频次、降级链切换原因  
- 验收：开启走廊保底后通过率提升/擦碰下降；且控制 jerk 不显著恶化；日志能解释每次触发原因

#### 12.8.4 每个案例的“可复现操作模板”（你把它做成脚本就能长期回归）
> 目标是：每次算法/参数改动，都能用同一套命令产出相同结构的产物（bag + 指标 + 报告），避免“凭感觉调参”。

**通用录包（建议你按数据集版本管理）**
```bash
mkdir -p bags/v2_social bags/v3_degraded

# v2：动态人群（示例）
ros2 bag record -o bags/v2_social/run_001 \
  /clock /tf /tf_static \
  /points_raw /points_filtered /imu \
  /wheel/odom /lio/odom /odometry/filtered /liwo/quality \
  /people /people/ground_truth \
  /cmd_vel /cmd_vel_raw \
  /plan /local_plan /goal_pose

# v3：退化注入（示例：增加一些 debug/诊断）
ros2 bag record -o bags/v3_degraded/run_001 \
  /clock /tf /tf_static \
  /points_raw /imu /wheel/odom \
  /lio/odom /odometry/filtered /liwo/quality \
  /local_occupancy /esdf_2d /dynamic_occupancy \
  /people /people/ground_truth \
  /cmd_vel /cmd_vel_raw
```

**同包回放做 A/B（示例：MPPI vs MPC vs RL）**
```bash
# 1) 回放同一包（统一 use_sim_time）
ros2 bag play bags/v2_social/run_001 --clock

# 2) 分别启动不同控制器/参数集（建议你用不同 launch 或不同 params）
# - baseline: MPPI
# - candidate: SE2 MPC
# - candidate: RL Controller (TRT/OV/CPU)
```

**期望产物（每次回归都要有）**
- `bags/...`：原始数据（可回放）  
- `reports/.../metrics.json`：指标（到达率/碰撞/人机距离/jerk/实时分位数/定位误差）  
- `reports/.../summary.md`：人类可读总结（包含失败原因 Top-K 与复现命令）

#### 12.8.5 “仿真预研”与“实机回归”的衔接方式（建议你用两条线并行）
**线 1：仿真预研（快速试错）**
- 目的：验证算法思想、快速定位 bug、构建退化场景  
- 产物：仿真 bag（v2/v3）+ 退化注入参数（噪声/延迟/丢帧/摩擦）

**线 2：实机回归（以实机指标为准）**
- 目的：验证仿真结论是否成立、发现 sim2real 缺口  
- 做法：在实机录包形成“golden dataset”，所有算法更新必须在该数据集上回归  
- 建议：对齐仿真与实机的传感器频率/QoS/延迟模型，把差异写进 domain randomization 配置

---

### 12.9 这条“现代 + 传统融合路线”的推荐推进顺序（避免一次性堆模块）
**阶段 1（传统骨架打牢）**
- Batch-LIWO（含 wheel 自适应协方差）+ EKF 融合输出  
- ROG occupancy + fading + 2D ESDF  
- MINCO + L-BFGS（含狭窄地形两阶段优化）  
- SE2 MPC（50Hz）+ collision_monitor 兜底

**阶段 2（现代感知与动态增强）**
- 视觉检测/追踪输出 `/people`（TRT/OV/CPU 三端）  
- 动态过滤（动态SLAM的最小版本：剔除动态点/特征）  
- social cost 融合到 ESDF/代价，BT 增加让行/等待策略

**阶段 3（学习增强，逐步替换而不是推倒重来）**
- IL：用 MPPI/MPC 生成 teacher 数据训练轻量策略（ONNX 部署）  
- RL：在退化注入与域随机化下做鲁棒性提升  
- 学习做权重自适应/模式切换（优先于端到端）

---

### 12.9.1 模块级“传统 vs 现代”组合表（建议你按场景选配）
| 子系统 | 传统优先（工程稳） | 现代增强（提升上限） | 产出/接口（建议固定） |
|---|---|---|---|
| 里程计/定位 | Batch-LIWO（LIO+wheel）+ EKF/UKF | 动态过滤特征、学习式退化检测（仅做权重/置信度） | `/lio/odom` `/odometry/filtered` `/liwo/quality` |
| 静态建图 | 2D map / 体素地图（工程成熟） | TSDF/ESDF（GPU）增强可导距离场 | `/map` `/esdf_2d`（或查询服务） |
| 动态感知 | LiDAR 聚类 + Kalman 跟踪 | 视觉检测+MOT + 多模态融合/预测 | `/people`（带速度/协方差） |
| 规划（全局） | Smac/Theta*（Nav2） | traversability layer/语义代价 | `nav_msgs/Path` |
| 规划（局部） | ESDF + MINCO + L-BFGS | 预测式动态代价/社交规则学习 | `trajectory`（连续参考） |
| 控制 | SE2 MPC（强约束可解释） | RL 做候选控制器/权重自适应 | `/cmd_vel_raw` → `collision_monitor` → `/cmd_vel` |
| 安全 | collision_monitor + 限幅 | CBF/学习安全过滤（慎用，先离线验证） | 安全统计/告警 |

---

### 12.9.2 “自研归控”的推荐包/节点拆分（DRY + SRP，便于团队协作）
> 下面是建议的工程拆分（不是强制），核心目标是：每个包职责单一、接口清晰、可独立回归。

**建议的 ROS2 package（示例命名）**
- `my_odom_batch_liwo`：Batch-LIWO 节点 + 质量度量输出  
- `my_perception_rog_esdf`：occupancy + fading + ESDF 构建与查询  
- `my_people_perception`：视觉/雷达检测与跟踪，统一输出 `/people`  
- `my_planner_minco`：MINCO 轨迹生成（可作为 Nav2 planner/smoother 插件或独立 node）  
- `my_controller_se2_mpc`：Nav2 controller plugin（SE2 MPC）  
- `my_controller_rl`：Nav2 controller plugin（ONNX/TRT/OV/CPU）  
- `my_nav_eval`：评测脚本（bag → 指标 → 报告），包含失败原因归类

**强制建议：统一调试输出**
- 所有关键模块都输出 `diagnostic` 与“耗时分位数”指标  
- 所有模块都接受 `use_sim_time`，并能在 bag 回放下稳定运行

### 12.10 已确认的系统假设与落地决策（根据你的回复固化）
你已确认并纳入本方案的关键决策如下（后续建议把这些写进“项目配置基线”，避免频繁改动导致回归失效）：
1) wheel：轮速可带时间戳（可用于对齐与退化度量）  
2) 3D LiDAR：10Hz，非重复扫描（按“连续点流”处理，Batch-Wise 对这种雷达更友好）  
3) 视觉：RGBD，具备硬件同步（用于近场占据、动态过滤、检测/追踪）  
4) 场景：室内为主，且存在狭窄地形难点（长走廊/隧道/转角窄通道类）  
5) 平台：Intel 为主（推理优先 OpenVINO；CPU fallback 必须可用）  
6) 距离场：采用局部 3D ESDF（并派生 2D 切片给 Nav2/可视化）  
7) 安全走廊：作为 BT 的保底策略（低频构建，失败可降级回 ESDF/MPC/MPPI）  
8) 目标：需要上实机（因此必须建设：诊断、日志、参数锁定、模型工件管理、golden dataset 回归）

### 12.11 建议你再补充确认的“上实机关键细节”（不影响现在开干，但会影响最终稳定性）
为了把第 12 章进一步“落成可直接开工的工程规格”，建议你补充以下信息（可简答）：
1) wheel odom 发布频率与时间戳来源：例如 100Hz/200Hz？stamp 来自同一时钟域吗？  
2) IMU 频率与饱和范围：例如 200Hz？陀螺/加计满量程（是否容易饱和）？  
3) RGBD 型号与分辨率/帧率：例如 640×480@30？深度噪声/空洞是否明显？  
4) LiDAR 点数规模与 FoV：每秒点数/视场角（决定 Batch 大小、地图结构、CPU 压力）  
5) 机器人几何与 footprint：外形尺寸、雷达到 base_link 外参位置（影响狭窄通道裕度与走廊）  
6) 控制接口与延迟：底盘从 `/cmd_vel` 到实际速度的滞后大约多少（10–50ms？）  
7) 实机安全策略：是否需要硬件急停/区域限速/人机最小距离硬阈值（合规要求）？

### 12.12 上实机交付与量产化清单（建议你从一开始就按这个做）
> 你明确要上实机。这里给你一份“工程落地 checklist”，按优先级排序。  
> 原则：任何一个模块都可能偶发失败，但系统必须“可诊断、可降级、可复现、可回滚”。

#### 12.12.1 标定与时钟（上实机最常见隐形杀手）
**必做标定**
- LiDAR↔IMU 外参（`tf_static` 固化）：影响 LIO 稳定性与去畸变  
- RGBD 内参/畸变：影响 3D 定位与深度可靠性  
- RGBD↔base_link 外参：影响 people 3D 定位与近场占据

**时间同步**
- 既然你有 RGBD 硬件同步，建议把“相机-IMU”的同步链路写清楚，并在日志中记录时间偏差统计  
- wheel 时间戳要明确来源：MCU 时钟/主机时钟/同步后的统一时钟域  

验收标准（建议量化）：
- IMU 与 RGBD 时间差统计（均值/P95）  
- LiDAR 与 IMU 对齐后的 deskew 残差是否稳定（用 `/liwo/quality` 辅助）

#### 12.12.2 部署与版本管理（模型/参数/代码必须可追溯）
**强制建议**
- 每次发布都生成一个版本号：`YYYYMMDD_hhmm_gitsha`  
- 统一工件目录（建议）：
  - `configs/`：Nav2 params、ESDF params、MPC params  
  - `models/`：ONNX + `model_meta.yaml`（输入输出 schema、归一化、约束）  
  - `launch/`：一键 bringup（仿真/实机各一套）  
  - `reports/`：回归报告（引用 bag 与版本号）

**OpenVINO 部署建议**
- 记录 OpenVINO 版本、device（CPU/iGPU）、precision（FP16/INT8）  
- 模型一旦上线，输入 shape 与前处理必须锁定，否则线上延迟尖峰很难定位

#### 12.12.3 运行时诊断（必须有“原因码”，否则无法排障）
建议全系统统一输出：
- `diagnostic_msgs/DiagnosticArray`（每个模块一个 status：OK/WARN/ERROR）  
- 关键指标：`infer_ms/solve_ms/feature_ms/tf_ms` 分位数  
- 关键计数：`tf_timeout`、`costmap_stale`、`esdf_stale`、`infer_timeout`、`solver_timeout`、`progress_stuck`

#### 12.12.4 安全与降级（把危险关在笼子里）
**建议的安全链**
- 控制输出：`/cmd_vel_raw` → `velocity_smoother` → `collision_monitor` → `/cmd_vel`  
- 任何模块异常都不允许绕过 `collision_monitor`

**建议的降级层级（与 12.7.5 对齐）**
- RL → MPC → MPPI → 刹停  
- 切换必须可回放（写入 bag/日志），否则无法做回归分析

#### 12.12.5 实机 golden dataset（你最终能否稳定迭代，取决于它）
建议你尽早建立：
- `bags/real_golden/`：典型室内狭窄地形 + 动态人群 + 退化片段（轮滑/遮挡/深度空洞）  
- 规则：任何控制器/感知/定位改动，都必须在 `real_golden` 上回归并生成 `reports/`  
- 仿真数据集用于预研筛选；实机 golden 用于最终放行
