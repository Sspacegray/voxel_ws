# Localization Stack Comparison

## 概览
- **当前方案**：`robot_navigation` 自带 Fast-LIO（未启用先验图） + `icp_registration` + Nav2 读取 `diff_drive_controller/odom`。
- **候选方案**：独立的 `FAST_LIO` 包（启用 `locate_in_prior_map`）+ 新增 `icp_relocalization`（含 transform publisher）+ Nav2 直接消费 `/Odometry`。

## 关键维度对比
| 维度 | 现有方案 | 新方案 |
| --- | --- | --- |
| 初始定位 | 依赖 `icp_registration` 多阶段匹配，精度受 `rough/refine` 体素栈影响；需要在 RViz 手动给初始位姿 | `icp_relocalization` 支持 Livox 自定义点类型、连续发布 `icp_result` 与 `prior_map`，并由 Fast-LIO 自动消费 |
| 运行里程计 | Fast-LIO 输出 `/Odometry` 但 Nav2 仍读取轮速里程，姿态融合滞后 | Nav2/控制器直接使用 Fast-LIO `/Odometry`，IMU+点云融合更稳 |
| 先验地图 | `FAST_LIO/config/mid360.yaml` 中 `locate_in_prior_map:false`，无法加载 PCD | 参考 `fast_lio_relocalization_param.yaml` 启用 `locate_in_prior_map`，ICP 输出可直接对齐先验 |
| TF/Frames | 保留 `camera_init`、`body` 静态 TF，树混乱 | Fast-LIO 直接广播 `odom -> base_link`，`icp_relocalization` 提供 `map -> odom`，树更简单 |
| 点云管线 | Fast-LIO → `/cloud_registered_body` → linefit → `/scan`，逻辑可复用 | 相同链路，只需确保话题与 Fast-LIO 参数一致 |
| 定制空间 | `icp_registration` 参数散落在 launch 中，修改成本高 | 新方案可拆到 YAML，ICP/Fast-LIO 均支持独立配置 |
| 依赖成熟度 | `icp_registration` 代码写在 `robot_relocalization`，缺少点类型优化 | 新包直接来自 SCURM 工程，针对 Livox/MID360 优化，含 `transform_publisher` |
| 调试可视化 | `icp_registration` 只输出 TF，缺少 `prior_map` / `transformed_cloud` | `icp_relocalization` 内置 `prior_map`、`transformed_cloud` topic，便于评估匹配质量 |
| 资源占用 | `icp_registration` 与 Fast-LIO 并行，同步策略简单 | `icp_relocalization` 收敛后即可退出或待机，可节省算力 |

## AMCL 偏差原因
- `/scan` 的 `frame_id` 来自 `pointcloud_to_laserscan` 的 `target_frame=livox_frame`（`robot_navigation/src/launch/bringup.launch.py:91-113`），而该 frame 在 URDF 中绕 Y 轴依旧固定了 `-45°`（`robot_base/urdf/four_diff.xacro:55-60`），Livox 驱动又在 extrinsic 中设置了 `pitch=45°`（`robot_hardware_driver/livox_ros_driver2/config/MID360_config.json:31-38`），导致激光数据在转换回 `livox_frame` 时被再次仰起，AMCL 的 2D 激光模型无法正确匹配地图；
- AMCL 的运动模型仍依赖 `diff_drive_controller/odom`（`robot_navigation/src/params/nav2_params.yaml:45-154`），而 TF 树上的 `odom->base_link` 由 Fast-LIO 发布（`FAST_LIO/config/mid360.yaml:14-21` 与 `FAST_LIO/src/laserMapping.cpp:980-1020`），两套里程数据不一致，导致 AMCL 推理的里程与测量残差相互矛盾；
- 因此机器人运动后几乎不会被激光测量纠正，表现为“偏差很大且无法自动回正”。

## 建议
1. **修正激光姿态链路**：根据 `doc/fix.md` 所述，只保留 Livox 驱动或 URDF 中的一个俯仰补偿，并把 `pointcloud_to_laserscan` 的 `target_frame` 设为水平坐标系（`base_link` 或新的 `livox_scan`）；
2. **统一里程计来源**：让 Nav2/AMCL 与控制器都消费 Fast-LIO 的 `/Odometry`，同步修改 `robot_navigation/src/params/nav2_params.yaml` 中的 `odom_topic` 并在 `robot_navigation` Launch 中删除/禁用 `diff_drive_controller/odom` 依赖；
3. **实现可切换的定位模式**：在 `robot_navigation` Launch 与参数里直接提供 `localization_mode`，同时打包 Fast-LIO/ICP 所需 YAML，便于现场快速切换/回退。
