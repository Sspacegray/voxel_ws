# 导航本体改造计划

## 目标
直接在 `robot_navigation` 代码库内完成定位链路升级：以 Fast-LIO + `icp_relocalization` 取代 AMCL + `icp_registration`，并确保激光/里程计/TF/参数在同一 launch 体系内维护，无需依赖额外脚本。

## 拆分步骤

### 1. Launch 框架调整（直接改动代码）
1. 编辑 `robot_navigation/src/launch/bringup.launch.py`：
   - 新增 `pcd_file`、`fast_lio_config`、`icp_mode` 等 LaunchArgument。
   - 依据 `icp_mode` 选择 `FAST_LIO/config/mid360.yaml`（建图）或新建的 `FAST_LIO/config/fast_lio_relocalization.yaml`（定位）。
   - 当 `icp_mode` 为定位时，向 Fast-LIO 注入 `locate_in_prior_map:=true`、`prior_map_path:=pcd_file` 覆盖项，并移除 `camera_init`、`body` 静态 TF。
2. 编辑 `robot_navigation/src/launch/localization.launch.py`：
   - 保留 Bringup 引用，但把 `icp_registration` 节点替换为 `icp_relocalization` 的 `icp_node` + `transform_publisher`，参数引用 `robot_navigation/src/params/icp_relocalization.yaml`。
   - 将 `pcd_file`、初始位姿、阈值从 LaunchArgument 直接透传到 ICP 节点。
3. `robot_navigation/src/launch/navigation.launch.py`：
   - 仅保留 Nav2 节点与 RViz，所有传入参数（map、pcd、params、mode）原封不动传给 Localization Launch，保证一个入口即可切换方案。

### 2. 参数与话题统一
1. `FAST_LIO/config/fast_lio_relocalization.yaml`：复制 SCURM 模板，确认 lidar/imu 话题、frame 与机器人一致。
2. `robot_navigation/src/params/nav2_params.yaml`：将所有 `odom_topic` 更新为 `/Odometry`，并确认 `global_frame`=`map`、`robot_base_frame`=`base_link`，配合 `transform_publisher` 提供的 `map->odom`。
3. `robot_navigation/src/rviz/nav2_default_view.rviz`：更新 TF/odom 显示项，减少调试歧义。

### 3. 代码验证
1. `colcon build` 确认依赖完整；
2. `ros2 launch robot_navigation navigation.launch.py localization_mode:=icp pcd_file:=... map:=...` 验证 TF、激光、Nav2；
3. 监控 `/icp_result`、`/Odometry`、`/scan`，确保链路闭环。

## 注意
- 所有改动都集中在 `robot_navigation`、`FAST_LIO` 参数与 `robot_navigation/src/params`，不再依赖外部脚本；
- Launch 结构保持单入口，便于后续集成测试与部署。
