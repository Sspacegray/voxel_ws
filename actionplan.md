# FAST-LIO 倾斜问题行动计划 (actionplan.md)

## 目标
- 消除 `odom` 作为 Fix Frame 时的 90° 俯仰问题，确保 FAST-LIO 与底盘 TF 链一致。
- 统一 LiDAR/IMU 外参与 TF 配置，保证建图、导航、录包回放流程稳定。

## 阶段 1：信息确认与清理
1. **梳理现有配置**
   - [x] 核对 `src/Fast-LIO2-Localization/FAST_LIO/config/mid360.yaml` 与 `fast_lio_relocalization_param.yaml` 中 `common.*` 参数，重点是 `sensor_frame_id`、`base_frame_id`、`send_odom_base_tf`、`mapping.extrinsic_T/R`。
   - [x] 核对 `src/robot_base/urdf/four_diff.xacro` 里 `livox_joint` 的物理姿态；**已修正**：将 pitch 从 -0.785 改为 0.785 (Nose Down)，解决 90° 倾斜问题。
   - 收集最近的运行日志 `/home/suja/.ros/log/...`，观察报错与 TF 警告。
2. **清理多余 TF 发布者**
   - [x] 确保 `robot_base/config/controllers.yaml` 中 `enable_odom_tf=false`（轮速不发 TF）。
   - 检查是否还有其他节点 (`static_transform_publisher`、导航栈) 在广播 `odom->base_*`，必要时暂时关闭。

## 阶段 2：参数调整
1. **统一 FAST-LIO 帧设定**
   - 将 `sensor_frame_id` 与 `base_frame_id` 统一为 `base_footprint`，`send_odom_base_tf=false`，确保 `laserMapping` 直接输出 `odom->base_footprint`。
   - 保持 `mapping.extrinsic_T/R` 为真实 LiDAR→IMU 外参；**已修改**：设置 `extrinsic_est_en=false`，强制使用 Identity 旋转，避免内参估计漂移导致地图倾斜。
2. **静态 TF（可视化用）**
   - 在 `fast_lio/launch/mapping.launch.py` 中保留 `livox_frame -> lio_sensor` 的 `static_transform_publisher`，但仅用于 RViz 展示，确定 pitch 方向使用 `PythonExpression(['-', lio_level_pitch])`。
   - 若不需要 `lio_sensor`，可以直接移除该静态 TF，避免混淆。

## 阶段 3：代码修复与编译
1. **确认 `laserMapping.cpp` 修补**
   - 确保 `send_odom_base_tf` 分支使用 `lookupTransform(base_frame, sensor_frame, ...)` 并用 `sensor_to_base` 矩阵（避免反向乘法导致 90° 倾斜）。
   - 若未来需要再次启用该分支，考虑添加调试日志记录实时 TF。
2. **重新编译与安装**
   - 执行 `colcon build --packages-select fast_lio`，并 `source install/setup.bash`。
   - 记录构建输出，确保无 `allow-overriding` 相关警告；如保留自定义 `livox_ros_driver2`，添加 `--allow-overriding livox_ros_driver2`。

## 阶段 4：验证流程
1. **在线验证**
   - 启动顺序：`livox_ros_driver2` → `robot_base bringup` → `fast_lio mapping`。
   - 在 RViz 中设置 `Fixed Frame=odom`，观察点云是否水平；同时查看 TF 树 `ros2 run tf2_ros tf2_monitor odom base_footprint`，确保只有 FAST-LIO 一条边、姿态稳定。
2. **录包与回放**
   - 录制话题：`/tf /tf_static /state_estimation /livox/lidar /livox/imu`。
   - 离线 `ros2 bag play`，再运行 FAST-LIO 回放，验证建图与 TF 输出一致，满足实时性能检查要求。

## 阶段 5：文档与后续
1. **文档记录**
   - 在 `doc/` 目录新增一条记录，描述本次调整的参数、外参、验证结果。
   - 保存 `frames_*.pdf` 供团队查阅。
2. **后续工作**
   - 如需集成导航栈 (Nav2)，在 `nav2_params.yaml` 中明确 `odom_topic: /state_estimation`；若引入回环/重定位，定义 `map->odom` 的发布策略。
   - 根据文章建议，若雷达位置再次调整，优先更新外参文件，再复跑上述验证流程。
