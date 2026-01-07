# Visual Perception Package

D435 深度相机视觉感知功能包，用于障碍物检测和可行道路分割。

## 功能

1. **深度图处理** (`depth_processor_node`)
   - 深度图滤波（中值滤波 + 双边滤波）
   - 生成障碍物点云
   - 高度过滤（去除地面和高处障碍物）

2. **深度转激光扫描** (`depth_to_laser_node`)
   - 将深度图投影为虚拟 2D 激光扫描
   - 输出 `/visual_scan` 话题
   - 与 Nav2 代价地图无缝集成

3. **道路分割** (`road_segmentation_node`)
   - 基于 RANSAC 的地面平面拟合
   - 输出可通行区域掩码 `/traversable_mask`
   - 分离地面点云和障碍物点云

## 安装

```bash
# 确保 D435 相机驱动已安装
sudo apt install ros-humble-realsense2-camera

# 编译
cd ~/voxel_ws
colcon build --packages-select visual_perception --symlink-install
source install/setup.bash
```

## 使用方法

### 1. 仅视觉感知（测试用）

```bash
# 启动相机和视觉处理节点
ros2 launch visual_perception navigation_with_vision.launch.py
```

### 2. 与现有导航系统集成

修改现有导航系统的 launch 文件，添加视觉感知启动：

```python
# 在 navigation.launch.py 中添加
visual_perception_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(
            get_package_share_directory('visual_perception'),
            'launch', 'visual_perception.launch.py'
        )
    ]),
    launch_arguments={
        'use_sim_time': 'true',  # 仿真时设为 true
        'camera_namespace': 'camera',
    }.items(),
)
```

### 3. 修改 Nav2 参数

在 `nav2_params.yaml` 的 `local_costmap` 中添加视觉障碍物层：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "visual_obstacle_layer", "inflation_layer"]
      
      visual_obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: visual_scan
        visual_scan:
          topic: "/visual_scan"
          data_type: "LaserScan"
          marking: True
          clearing: True
          max_obstacle_height: 2.0
```

## 话题

### 输入话题
| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera/depth/image_rect_raw` | sensor_msgs/Image | D435 深度图 |
| `/camera/color/image_raw` | sensor_msgs/Image | D435 彩色图像 |
| `/camera/depth/camera_info` | sensor_msgs/CameraInfo | 相机内参 |

### 输出话题
| 话题 | 类型 | 说明 |
|------|------|------|
| `/visual_scan` | sensor_msgs/LaserScan | 虚拟激光扫描（用于代价地图） |
| `/traversable_mask` | sensor_msgs/Image | 可通行区域掩码 |
| `/ground_cloud` | sensor_msgs/PointCloud2 | 地面点云 |
| `/visual_obstacle_cloud` | sensor_msgs/PointCloud2 | 障碍物点云 |
| `/depth_obstacle_cloud` | sensor_msgs/PointCloud2 | 深度处理后的障碍物点云 |

## 参数配置

参数文件: `config/visual_perception_params.yaml`

### 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `min_depth` | 0.2 | 最小有效深度 (m) |
| `max_depth` | 5.0 | 最大有效深度 (m) |
| `scan_height_min` | -0.3 | 扫描最小高度 (m) |
| `scan_height_max` | 0.5 | 扫描最大高度 (m) |
| `plane_distance_threshold` | 0.03 | RANSAC 平面距离阈值 |
| `max_slope` | 0.3 | 最大可通行坡度 (tan值, ~17°) |

## 与激光雷达融合

本功能包设计为与现有激光雷达避障系统协同工作：

```
激光雷达 (/scan) ──┐
                  ├──► Nav2 Costmap ──► 路径规划
深度相机 (/visual_scan) ─┘
```

- 激光雷达：水平面 360° 检测
- 深度相机：前方 87° 视野，可检测低矮障碍物和悬空物体

## 坐标系配置

请根据实际相机安装位置修改 `navigation_with_vision.launch.py` 中的静态 TF：

```python
# 修改这些值以匹配实际相机安装位置
arguments=[
    '--x', '0.1',      # 相机在 base_link 前方距离
    '--y', '0.0',      # 左右偏移
    '--z', '0.3',      # 相机高度
    '--pitch', '0.0',  # 俯仰角（向下为正）
    ...
]
```

## RViz 可视化

使用提供的 RViz 配置文件：

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix visual_perception)/share/visual_perception/rviz/visual_perception.rviz
```

显示内容：
- 激光扫描（红色）和视觉扫描（青色）
- 深度图像、彩色图像
- 可通行区域掩码
- 地面点云（绿色）和障碍物点云（橙色）
- 局部/全局代价地图
