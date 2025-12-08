# 导航定位改造说明

> **当前默认模式：Fast-LIO + ICP。** 如需回退 AMCL，请显式传入 `localization_mode:=amcl` 并忽略 PCD/ICP 参数，下文的命令示例已经按照该约定更新。

## 背景
- 机器人原先依赖 `icp_registration + AMCL + diff_drive_controller/odom` 完成定位；
- Lidar 实际安装向下 45°，但 URDF 和 Livox 驱动都在补偿俯仰，导致 `/scan` 重新仰起，AMCL 很难与 2D 地图匹配；
- Fast-LIO 虽已集成在 Bringup 中，但未启用 `locate_in_prior_map`，Nav2 也没有消费 `/Odometry`，导致点云里程计的优势无法发挥。

## 目标
1. 在 `robot_navigation` 的 Launch/参数内部完成定位链升级，提供 AMCL 和 Fast-LIO+ICP 两套模式；
2. 统一 Lidar→LaserScan 的坐标系，使 `/scan` 在水平平面内；
3. 让 Nav2 全栈使用 Fast-LIO 的 `/Odometry`，避免机械轮速与点云里程混用。

## 关键改动
### Launch 与参数
- `bringup.launch.py`
  - 引入 `pcd_file`、`fast_lio_config`、`fast_lio_localization_config` 参数；
  - 当 `localization_mode==icp` 时加载 `FAST_LIO/config/fast_lio_relocalization.yaml` 并注入 `prior_map_path`；
  - `pointcloud_to_laserscan` 的 `target_frame` 改为 `base_link`，同时移除 `camera_init/body` 静态 TF。
- `localization.launch.py`
  - 将 `icp_registration` 替换为 `icp_relocalization` + `transform_publisher`；
  - 向 Bringup 透传新的 Fast-LIO 配置。
- `navigation.launch.py`
  - 顶层 Launch 增加 Fast-LIO/ICP 参数，保持单入口切换定位模式。
- `nav2_params.yaml`
  - BT Navigator、Controller Server、Velocity Smoother 改订阅 `/Odometry`。

### 新增配置与文档
- `FAST_LIO/config/fast_lio_relocalization.yaml`：定位专用、默认 `locate_in_prior_map:true`；
- `robot_navigation/src/params/icp_relocalization.yaml`：集中 ICP 参数，便于部署；
- `doc/fix.md`：记录 Lidar 45° 倾角与 `/scan` 倾斜问题；
- `doc/plan.md`：更新后的实施步骤；
- `doc/compare.md`：旧方案与新方案对比；
- `doc/review.md`：提交说明；
- `doc/readme.md`（本文件）：总体解释。

## 使用方式
```bash
# 构建工程
colcon build --packages-select robot_navigation fast_lio icp_relocalization

# AMCL 模式（需要手动指定）
ros2 launch robot_navigation navigation.launch.py \
  localization_mode:=amcl \
  map:=/path/to/map.yaml

# Fast-LIO + ICP 模式（默认，必须提供 PCD/配置文件）
ros2 launch robot_navigation navigation.launch.py \
  localization_mode:=icp \
  map:=/path/to/map.yaml \
  pcd_file:=/path/to/pcd.pcd \
  fast_lio_localization_config:=/path/to/fast_lio_relocalization.yaml \
  icp_params_file:=/path/to/icp_relocalization.yaml
```
> 若 `localization_mode==icp` 但未提供 `pcd_file`，Launch 会直接报错以提示补全参数。

## 验证建议
1. 使用 `rviz` 检查 `/scan` 是否水平、`/Odometry` 是否连通 `map->odom->base_link`；
2. 在 Fast-LIO + ICP 模式下，确认 `icp_result`、`prior_map`、`transformed_cloud` 正常发布；
3. 用 `ros2 topic echo /Odometry`、`/scan`、`/cmd_vel_nav` 等话题确认 Nav2 接入链路正常；
4. 实车验证时，先在空旷环境走“8”字路径观察定位漂移，再进入复杂场景。
