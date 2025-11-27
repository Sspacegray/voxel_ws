# linefit_ground_segmentation (地面分割算法)

实现了以下论文中提出的地面分割算法：
```
@inproceedings{himmelsbach2010fast,
  title={Fast segmentation of 3d point clouds for ground vehicles},
  author={Himmelsbach, Michael and Hundelshausen, Felix V and Wuensche, H-J},
  booktitle={Intelligent Vehicles Symposium (IV), 2010 IEEE},
  pages={560--565},
  year={2010},
  organization={IEEE}
}
```
`linefit_ground_segmentation` 包包含地面分割库。
ROS接口在 `linefit_ground_segmentation_ros` 中提供。

如果你不使用ROS，该库可以独立于ROS接口进行编译。

## 安装

```bash
colcon build --symlink-install
```

## 启动说明

可以通过执行 `ros2 launch linefit_ground_segmentation_ros segmentation.launch` 来启动地面分割ROS节点。
输入和输出话题名称可以在同一文件中指定。

使用你自己的点云源进行启动和运行非常简单：

1. 将 `segmentation.launch` 中的 `input_topic` 参数修改为你的话题名称。
2. 调整 `segmentation_params.yaml` 中的 `sensor_height` 参数为传感器在机器人上的安装高度（例如，KITTI Velodyne: 1.8m）

## 参数说明

参数设置在 `linefit_ground_segmentation_ros/launch/segmentation_params.yaml` 中

该算法基于已知传感器相对地面高度的假设工作。
因此，**你必须根据机器人规格调整 `sensor_height`**，否则算法将无法正常工作。

默认参数适用于KITTI数据集。

### 地面条件参数
- **sensor_height**  传感器相对地面的高度。
- **max_dist_to_line**  点到线的最大垂直距离，超过此值将不被视为地面点。
- **max_slope**  直线的最大斜率。
- **min_slope**  直线的最小斜率。
- **max_fit_error**  点在直线拟合中允许的最大误差。
- **max_start_height**  新点与估计地面高度之间的最大高度差，用于开始新的直线。
- **long_threshold**  应用max_height条件的距离阈值。
- **max_height**  当直线点之间的距离超过 *long_threshold* 时，允许的最大高度差。
- **line_search_angle**  在角度方向上搜索直线的范围。更大的角度有助于填补地面分割中的"空洞"。
- **gravity_aligned_frame**  与重力对齐的坐标系名称（其z轴与重力对齐）。如果指定，传入的点云将被旋转（但不平移）到该坐标系。如果留空，将使用传感器坐标系。

### 分割参数

- **r_min**  开始分割的距离。
- **r_max**  结束分割的距离。
- **n_bins**  径向分区数量。
- **n_segments**  角度分段数量。

### 其他参数

- **n_threads**  使用的线程数量。
- **latch**  在ROS节点中锁存输出点云。
- **visualize**  可视化分割结果。**仅用于调试。** 在线运行时请勿设置为true。
