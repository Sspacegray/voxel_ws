# 激光姿态问题排查记录

## 现象
- MID360 实际安装在底盘前方，向下俯仰约 45°；
- Livox 驱动配置中已经把 `pitch` 设置为 `45.0`（`robot_hardware_driver/livox_ros_driver2/config/MID360_config.json:31-38`），点云在 RViz 显示正常；
- 经 `linefit_ground_segmentation_ros` 与 `pointcloud_to_laserscan` 处理后的 `/scan` 却出现反向（向上）倾斜 45°。

## 原因
1. URDF 里 `base_link -> livox_frame` 的静态关节仍然保留了 `rpy="0 -0.785398 0"`（`robot_base/urdf/four_diff.xacro:55-60`），相当于再次把点云视作向下 45° 的传感器；
2. `pointcloud_to_laserscan` 的 `target_frame` 设置成 `livox_frame`（`robot_navigation/src/launch/bringup.launch.py:91-109`），因此在生成激光前会把点云从 Fast-LIO 输出（近似水平）的 `base_link` 变换回那个仍然倾斜的 `livox_frame`；
3. 变换后的点云重新被投影到 2D，导致 /scan 相对于地图仰起 45°，AMCL/Costmap 也会收到同样的扭曲数据。

## 修改建议
1. **只保留一个补偿步骤**：要么把 Livox 驱动中的 `pitch` 还原为 `0`，继续依赖 URDF 的 `rpy="-0.785398"`；要么保留驱动补偿，并把 URDF 中 `livox_joint` 的 RPY 设为 `0 0 0`；两者只能选其一；
2. **LaserScan 使用水平坐标系**：将 `pointcloud_to_laserscan` 的 `target_frame` 改为 `base_link` 或新增的 `livox_scan`（姿态水平的子 frame），确保转换到 2D 之前的坐标已经去掉俯仰；
3. `linefit_ground_segmentation_ros` 目前订阅 `/cloud_registered_body` 并假设其 Z 轴与重力对齐（`segmentation_params.yaml:1-19`），若保留 Fast-LIO 的 body frame，不需要额外修改；若未来直接使用 Livox 原始点云，则需要同步更新 `gravity_aligned_frame`。

采用上述修复后，可同时解决 /scan 反向倾斜问题，并为 AMCL/Costmap 提供符合 2D 假设的激光数据。

## 硬编码路径修复
为了提高代码的可移植性，修复了 Launch 文件和配置文件中的硬编码绝对路径：

1. **Launch 文件修改**：
   - `robot_navigation/src/launch/bringup.launch.py`:
     - 移除了 `pcd_file` 的默认绝对路径。
     - 移除了 `livox_driver_node` 中测试用的 `lvx_file_path`。
   - `robot_navigation/src/launch/navigation.launch.py`:
     - 使用 `PathJoinSubstitution` 动态获取 `map` 参数的默认路径（指向包内的 `map/1126.yaml`）。
     - 移除了 `pcd_file` 的默认绝对路径。

2. **配置文件修改**：
   - `FAST_LIO/config/fast_lio_relocalization.yaml`: 将 `prior_map_path` 和 `map_file_path` 修改为占位符 `/tmp/test.pcd`。
   - `robot_navigation/src/params/icp_relocalization.yaml`: 将 `map_path` 修改为占位符 `/tmp/test.pcd`。

**注意**：在使用 ICP 定位模式时，请务必在 Launch 命令中通过 `pcd_file:=/path/to/your.pcd` 显式指定 PCD 文件路径。
