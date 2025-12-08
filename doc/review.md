# 修改说明

## 代码变更
1. `robot_navigation/src/launch/bringup.launch.py`
   - 新增 `pcd_file`、`fast_lio_config`、`fast_lio_localization_config` 参数，基于 `localization_mode` 在建图与定位之间切换不同的 Fast-LIO 配置，并在定位模式下注入 `prior_map_path`。
   - 调整 `pointcloud_to_laserscan` 的 `target_frame` 为 `base_link`，避免将点云再次旋转回旧的 `livox_frame`，配合固定外参后可生成水平激光。
   - 移除多余的静态 TF 发布器，改由 Fast-LIO 输出的 TF 链主导。

2. `robot_navigation/src/launch/localization.launch.py`
   - Bringup 现在接收新的 Fast-LIO 参数。
   - 将 `icp_registration` 替换为 `icp_relocalization`，并额外启动 `transform_publisher`，配套新的 `icp_params_file`。

3. `robot_navigation/src/launch/navigation.launch.py`
   - 顶层 Launch 可以统一设置 Fast-LIO/ICP 的参数文件，并向下传递，保持单一入口切换定位模式。

4. `robot_navigation/src/params/nav2_params.yaml`
   - Nav2、控制器和速度平滑模块全部改为订阅 Fast-LIO 的 `/Odometry`，避免轮速里程与点云里程冲突。

5. 新增配置
   - `FAST_LIO/config/fast_lio_relocalization.yaml`：定位模式专用配置，默认启用 `locate_in_prior_map`。
   - `robot_navigation/src/params/icp_relocalization.yaml`：集中 ICP 参数，Launch 中可以按需覆盖 `map_path`。

## 预期效果
- 切换到 `localization_mode:=icp` 时，Fast-LIO 自动加载先验 PCD，ICP 负责发布 `map->odom`，Nav2 使用 `/Odometry`，导航坐标链更加一致。
- `pointcloud_to_laserscan` 使用水平坐标系，解决了 /scan 反向倾斜导致 AMCL 与 costmap 不可信的问题。
- 新增的参数文件使得定位/建图参数明确区隔，后续只需在 Launch 层修改路径即可完成部署。
