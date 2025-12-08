# Voxel Robot Navigation System

这是一个基于 ROS 2 (Humble) 的移动机器人自主导航与定位系统。本项目集成了 Livox Mid360 激光雷达、Fast-LIO 里程计、Nav2 导航栈以及相关的硬件驱动和感知算法，提供了一套完整的建图、定位和导航解决方案。

## 目录

- [项目简介](#项目简介)
- [整体技术架构](#整体技术架构)
- [项目架构](#项目架构)
- [感知与预处理模块](#感知与预处理模块)
  - [IMU 互补滤波器](#imu-互补滤波器-imu_complementary_filter)
  - [地面分割](#地面分割-linefit_ground_segmentation)
  - [点云转激光扫描](#点云转激光扫描-pointcloud_to_laserscan)
- [定位方案](#定位方案)
- [导航方案](#导航方案)
- [工具模块](#工具模块)
- [硬件依赖](#硬件依赖)
- [软件依赖](#软件依赖)
- [安装与编译](#安装与编译)
- [核心功能与使用](#核心功能与使用)
  - [1. 硬件驱动启动](#1-硬件驱动启动)
  - [2. 建图模式 (Mapping)](#2-建图模式-mapping)
  - [3. 导航与定位 (Navigation & Localization)](#3-导航与定位-navigation--localization)
- [参数配置说明](#参数配置说明)
- [常见问题与故障排除](#常见问题与故障排除)

---

## 项目简介

- **传感器侧**：Livox Mid360 激光雷达 + 外置高精度 IMU，支持雷达内置 IMU 兜底。
- **定位侧**：Fast-LIO 作为主里程计与定位方案，可切换 Point-LIO/LIO-SAM 等实现。
- **导航侧**：Nav2 + MPPI 控制器 + 行为树导航（行为树文件随导航配置发布）。
- **控制侧**：基于 `ros2_control` 的底盘接口，输出 `/cmd_vel` 至差速/全向底盘驱动。
- **应用侧**：留有 `robot_app` 框架用于任务调度或上层业务逻辑接入。

## 整体技术架构

**数据流/分层**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           应用层 (robot_app)                                 │
│                     任务调度 / 业务逻辑 / 状态机                              │
└─────────────────────────────────────────────────────────────────────────────┘
                                     ↓ 目标点
┌─────────────────────────────────────────────────────────────────────────────┐
│                        导航与决策层 (Nav2)                                   │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐                    │
│  │Planner   │  │Controller│  │Behavior  │  │Costmap   │                    │
│  │(Theta*)  │  │(MPPI)    │  │(BT Nav)  │  │(2D Grid) │                    │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘                    │
└─────────────────────────────────────────────────────────────────────────────┘
                    ↑ /scan, /map, /tf                ↓ /cmd_vel
┌─────────────────────────────────────────────────────────────────────────────┐
│                        感知与预处理层                                        │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐          │
│  │pointcloud_to_    │  │linefit_ground_   │  │imu_complementary │          │
│  │laserscan         │  │segmentation      │  │_filter           │          │
│  │(3D→2D投影)       │  │(地面/障碍物分离) │  │(IMU姿态融合)     │          │
│  └──────────────────┘  └──────────────────┘  └──────────────────┘          │
└─────────────────────────────────────────────────────────────────────────────┘
                    ↑ /livox/lidar, /imu/data
┌─────────────────────────────────────────────────────────────────────────────┐
│                           定位层                                             │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐          │
│  │Fast-LIO          │  │Point-LIO         │  │LIO-SAM           │          │
│  │(主方案)          │  │(备选)            │  │(备选)            │          │
│  └──────────────────┘  └──────────────────┘  └──────────────────┘          │
│                    输出: /Odometry, /tf (map→odom→base_link)               │
└─────────────────────────────────────────────────────────────────────────────┘
                    ↑ /livox/lidar, /imu/data
┌─────────────────────────────────────────────────────────────────────────────┐
│                      传感器与驱动层                                          │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐          │
│  │livox_ros_driver2 │  │hipnuc_imu        │  │robot_base        │          │
│  │(雷达驱动)        │  │(IMU驱动)         │  │(底盘ros2_control)│          │
│  └──────────────────┘  └──────────────────┘  └──────────────────┘          │
└─────────────────────────────────────────────────────────────────────────────┘
```

**关键接口速览**
- 话题：`/livox/lidar`(点云)、`/imu/data`、`/Odometry`、`/scan`、`/cmd_vel`、`/tf`。
- 核心配置：导航参数 `src/robot_navigation/src/params/nav2_params.yaml`；LIO 外参与话题 `src/robot_localization/fast_lio/config/mid360.yaml`。

## 项目架构

本项目采用模块化设计，代码组织在 `src` 目录下，主要模块功能如下：

| 模块名称 | 功能描述 | 关键技术栈 |
| :--- | :--- | :--- |
| **robot_base** | 机器人底盘控制、URDF 模型定义、硬件接口 | `ros2_control`, `xacro`, `serial` |
| **robot_hardware_driver** | 传感器驱动（激光雷达、IMU 等） | `livox_ros_driver2`, `hipnuc_imu` |
| **robot_localization** | 激光雷达里程计与定位算法 | `fast_lio`, `point_lio`, `lio-sam` |
| **robot_perception** | 感知处理模块 | `imu_complementary_filter`, `linefit_ground_segmentation`, `pointcloud_to_laserscan` |
| **robot_navigation** | Nav2 导航配置、Launch 启动文件 | `nav2`, `behavior_trees`, `mppi_controller` |
| **robot_app** | 上层应用逻辑（如任务调度） | Python/C++ |
| **robot_calibration** | 传感器标定工具 | 自动标定脚本 |
| **tools** | 辅助工具 | `pcd2pgm`, `pcd2elevation` |

---

## 感知与预处理模块

### IMU 互补滤波器 (`imu_complementary_filter`)

**位置**: `src/robot_perception/imu_complementary_filter/`

**功能**: 使用互补滤波算法融合 IMU 的加速度计和陀螺仪数据，输出稳定的姿态估计（Roll, Pitch, Yaw）。互补滤波是一种轻量级的姿态估计方法，适用于低漂移场景。

**原理**:
- 陀螺仪积分提供短期精确角速度，但长期存在漂移。
- 加速度计提供绝对重力方向参考，但受到高频噪声和运动加速度干扰。
- 互补滤波通过高通滤波陀螺仪数据 + 低通滤波加速度计数据，融合两者优点。

**关键参数**:
- `gain_acc`: 加速度计增益 (默认 0.01)，越大越信任加速度计。
- `do_bias_estimation`: 是否估计陀螺仪偏置。
- `do_adaptive_gain`: 是否使用自适应增益（运动时降低加速度计权重）。

**启动方式**:
```bash
ros2 launch imu_complementary_filter complementary_filter.launch.py
```

**替代方案对比**:

| 方案 | 优点 | 缺点 | 适用场景 |
| :--- | :--- | :--- | :--- |
| **互补滤波 (本项目)** | 计算量小、实时性好、易调参 | 无法处理磁力计、精度一般 | 移动机器人、无人机姿态估计 |
| **Madgwick Filter** | 支持磁力计、无需调参、精度高 | 计算量稍大 | 需要航向校正的应用 |
| **Mahony Filter** | 实现简单、支持磁力计 | 参数敏感 | 资源受限平台 |
| **EKF (robot_localization)** | 融合多源传感器、最优估计 | 计算复杂、需协方差调参 | 高精度定位、多传感器融合 |

---

### 地面分割 (`linefit_ground_segmentation`)

**位置**: `src/robot_perception/linefit_ground_segementation_ros2/`

**功能**: 将 3D 点云分割为「地面点」和「障碍物点」。地面点可用于构建可通行区域，障碍物点用于避障。

**原理**:
- 将点云按极坐标划分为若干扇区 (`n_segments`) 和距离区间 (`n_bins`)。
- 在每个扇区内拟合地面直线 (Line Fitting)。
- 根据点到地面线的距离判断是否为地面点。

**关键参数** (见 `ground_segmentation.h`):
- `sensor_height`: 传感器离地高度 (m)
- `max_slope`: 最大地面坡度 (tan值)
- `max_dist_to_line`: 判定为地面点的最大距离阈值 (m)
- `n_segments`: 角度分区数量 (默认 180)
- `n_bins`: 距离分区数量 (默认 30)

**输出话题**:
- `/segmentation/ground`: 地面点云
- `/segmentation/obstacle`: 障碍物点云

**替代方案对比**:

| 方案 | 优点 | 缺点 | 适用场景 |
| :--- | :--- | :--- | :--- |
| **Linefit (本项目)** | 速度快、适合平坦地面 | 复杂地形表现一般 | 室内/结构化室外环境 |
| **RANSAC** | 简单、鲁棒性好 | 速度慢、无法处理坡度 | 简单场景验证 |
| **Patchwork/Patchwork++** | 支持复杂地形、精度高 | 参数多、计算量大 | 野外自动驾驶 |
| **Ground Plane Fitting (GPF)** | 实时性好 | 假设单一地面 | 结构化环境 |
| **深度学习 (PointNet++)** | 语义分割能力强 | 需要训练、GPU依赖 | 复杂城市场景 |

---

### 点云转激光扫描 (`pointcloud_to_laserscan`)

**位置**: `src/robot_perception/pointcloud_to_laserscan/`

**功能**: 将 3D 点云 (`sensor_msgs/PointCloud2`) 投影为 2D 激光扫描 (`sensor_msgs/LaserScan`)。主要用于为 Nav2 代价地图提供 2D 障碍物数据。

**原理**:
- 定义一个高度范围 `[min_height, max_height]`，筛选该范围内的点。
- 将筛选后的点投影到 XY 平面。
- 按角度分 bin，取每个角度 bin 内的最近距离作为激光测量值。

**关键参数** (在 `launch/pointcloud_to_laserscan_launch.py` 中配置):
- `min_height`: 最小高度 (m)，过滤地面
- `max_height`: 最大高度 (m)，过滤高处障碍
- `angle_min`, `angle_max`: 激光扫描角度范围 (rad)
- `range_min`, `range_max`: 有效距离范围 (m)
- `target_frame`: 输出激光扫描的坐标系

**替代方案对比**:

| 方案 | 优点 | 缺点 | 适用场景 |
| :--- | :--- | :--- | :--- |
| **pointcloud_to_laserscan (本项目)** | 简单、通用、与 Nav2 兼容 | 丢失 3D 信息 | 2D 导航、成本敏感 |
| **octomap_server** | 保留 3D 信息、支持多层 | 内存占用大、更新慢 | 3D 避障、无人机 |
| **3D Costmap (voxel_layer)** | 原生 3D 障碍检测 | 配置复杂、计算量大 | 高精度 3D 导航 |
| **深度图投影** | 保留遮挡信息 | 需要深度相机 | RGB-D 导航 |

---

## 定位方案

本项目支持多种激光雷达惯性里程计 (LIO) 方案：

| 方案 | 路径 | 特点 | 适用场景 |
| :--- | :--- | :--- | :--- |
| **Fast-LIO (默认)** | `robot_localization/fast_lio` | 高效 ikd-tree、紧耦合 | 通用、实时性好 |
| **Point-LIO** | `robot_localization/point_lio` | 点级别紧耦合、精度极高 | 高精度建图 |
| **LIO-SAM** | `robot_localization/LIO-SAM` | 因子图优化、回环检测 | 大范围建图 |
| **Faster-LIO** | `robot_localization/faster-lio` | 增量式体素更新、更快 | 资源受限平台 |

**其他可选方案**:
- **KISS-ICP**: 纯激光里程计，无需 IMU
- **CT-ICP**: 连续时间 ICP，处理运动畸变
- **hdl_localization**: 基于 NDT/GICP 的定位

---

## 导航方案

### 路径规划器 (Planner)

| 方案 | 配置关键字 | 特点 |
| :--- | :--- | :--- |
| **Theta* (本项目)** | `ThetaStarPlanner` | 任意角度路径、平滑 |
| **NavFn (A*/Dijkstra)** | `NavfnPlanner` | 经典、可靠 |
| **Smac 2D** | `SmacPlanner2D` | 成本感知 A* |
| **Smac Hybrid-A*** | `SmacPlannerHybrid` | 支持非完整约束 |

### 路径控制器 (Controller)

| 方案 | 配置关键字 | 特点 |
| :--- | :--- | :--- |
| **MPPI (本项目)** | `MPPIController` | 采样优化、动态避障强 |
| **DWB** | `DWBLocalPlanner` | 动态窗口法、经典 |
| **Regulated Pure Pursuit** | `RegulatedPurePursuitController` | 简单、稳定 |
| **TEB** | `TEBLocalPlanner` (需额外安装) | 时间弹性带、支持倒车 |

### 路径平滑器 (Smoother)

| 方案 | 配置关键字 | 特点 |
| :--- | :--- | :--- |
| **Constrained Smoother (本项目)** | `ConstrainedSmoother` | 优化平滑、支持约束 |
| **Simple Smoother** | `SimpleSmoother` | 简单迭代平滑 |
| **Savitzky-Golay** | `SavitzkyGolaySmoother` | 滤波平滑 |

---

## 工具模块

| 工具 | 路径 | 功能 |
| :--- | :--- | :--- |
| **pcd2pgm** | `src/tools/pcd2pgm` | 将 PCD 点云地图转换为 2D 栅格地图 (PGM) |
| **pcd2elevation** | `src/tools/pcd2elevation` | 将 PCD 转换为高程图 |
| **performance_analysis** | `src/tools/performance_analysis` | 性能分析工具集 |

---

## 性能分析指南

本项目提供了一套完整的性能分析工具，位于 `src/tools/performance_analysis/`。以下是各类性能分析的详细使用方法。

### 1. 系统资源监控 (CPU/内存)

使用 `ana_cost.py` 脚本分析各节点的 CPU 和内存占用。

**步骤 1: 收集系统数据**
```bash
# 在运行导航时，开启另一个终端执行：
top -d 5 -n 60 -i -b > ~/top.txt
# -d 5: 每5秒采样一次
# -n 60: 采样60次 (共5分钟)
# -i: 忽略空闲进程
# -b: 批处理模式，输出到文件
```

**步骤 2: 分析数据**
```bash
cd ~/voxel_ws/src/tools/performance_analysis
# 修改 ana_cost.py 中的 file_path 和 proccess_name
python3 ana_cost.py
```

**输出**: 生成 CPU 和内存使用率的时序图，可识别哪个节点占用资源过高。

**典型问题诊断**:
| 现象 | 可能原因 | 解决方案 |
| :--- | :--- | :--- |
| `controller_server` CPU 过高 | MPPI batch_size 过大 | 降低 `batch_size` 到 1000 |
| `pointlio` 内存持续增长 | 地图累积过大 | 增加降采样或限制地图大小 |
| `planner_server` 峰值高 | 路径规划频繁 | 降低 `expected_planner_frequency` |

---

### 2. SLAM 轨迹精度评估 (EVO)

使用 [EVO](https://github.com/MichaelGrupp/evo) 工具评估定位算法的轨迹精度。

**安装 EVO**:
```bash
pip3 install evo --upgrade
```

**步骤 1: 录制里程计数据**
```bash
ros2 bag record /Odometry -o slam_test
```

**步骤 2: 转换为 TUM 格式**
```bash
evo_traj bag2 ./slam_test/ /Odometry --save_as_tum
# 生成 Odometry.tum 文件
```

**步骤 3: 轨迹可视化**
```bash
# 单轨迹可视化
evo_traj tum Odometry.tum -p --plot_mode=xy

# 多算法轨迹对比 (Fast-LIO vs Point-LIO)
evo_traj tum fast-lio.tum point-lio.tum -p --plot_mode=xy
```

**步骤 4: 相对位姿误差 (RPE) 分析**
```bash
# 需要真值轨迹 (ground_truth.tum)
evo_rpe tum ground_truth.tum Odometry.tum -v -p --plot_mode=xy
```

**步骤 5: 绝对轨迹误差 (ATE) 分析**
```bash
evo_ape tum ground_truth.tum Odometry.tum -v -p --align --correct_scale
```

**已有对比数据** (位于 `SLAM/` 目录):
- `fast-lio.tum`: Fast-LIO 轨迹
- `point-lio.tum`: Point-LIO 轨迹
- `fast-lio-sam.tum`: LIO-SAM 轨迹
- `relocalization.tum`: 重定位轨迹

---

### 3. 规划器性能对比

使用 `plansave.py` 脚本记录不同规划器的路径输出。

**步骤 1: 启动路径记录节点**
```bash
cd ~/voxel_ws/src/tools/performance_analysis/planning
python3 plansave.py
```

**步骤 2: 在 RViz 中发送目标点**

脚本会订阅 `/plan_smoothed` 话题并保存路径点到 TXT 文件。

**步骤 3: 切换规划器并重复测试**

修改 `nav2_params.yaml` 中的 `GridBased.plugin`：
```yaml
# 测试不同规划器
GridBased:
  plugin: "nav2_navfn_planner/NavfnPlanner"  # A*/Dijkstra
  # plugin: "nav2_smac_planner/SmacPlanner2D"  # Smac 2D
  # plugin: "nav2_theta_star_planner/ThetaStarPlanner"  # Theta*
```

**已有对比数据** (位于 `planning/` 目录):
| 文件 | 规划器 | 说明 |
| :--- | :--- | :--- |
| `AStar.txt` | NavFn (A*) | 经典 A* 路径 |
| `Dijkstra.txt` | NavFn (Dijkstra) | Dijkstra 路径 |
| `Smac.txt` | SmacPlanner2D | 成本感知 A* |
| `ThetaStar.txt` | ThetaStarPlanner | 任意角度路径 |

**分析指标**:
- 路径长度 (总点数)
- 路径平滑度 (角度变化)
- 规划时间 (通过 `ros2 topic echo /plan --qos-durability transient_local` 查看时间戳)

---

### 4. 路径平滑器对比

评估不同 Smoother 的平滑效果。

**已有对比数据** (位于 `smoother/` 目录):
| 文件 | Smoother | 说明 |
| :--- | :--- | :--- |
| `raw.txt` | 无平滑 | 原始规划路径 |
| `SimpleSmoother.txt` | SimpleSmoother | 简单迭代平滑 |
| `SavitzkyGolaySmoother.txt` | SavitzkyGolaySmoother | 滤波平滑 |
| `cons.txt` | ConstrainedSmoother | 约束优化平滑 |

**可视化对比**:
```python
import matplotlib.pyplot as plt
import numpy as np

def load_tum(file):
    data = np.loadtxt(file)
    return data[:, 1], data[:, 2]  # x, y

files = ['raw.txt', 'SimpleSmoother.txt', 'cons.txt']
for f in files:
    x, y = load_tum(f)
    plt.plot(x, y, label=f)
plt.legend()
plt.axis('equal')
plt.show()
```

---

### 5. 底盘标定

底盘标定用于校准里程计的线速度和角速度精度。标定脚本位于 `src/robot_base/scripts/`。

#### 5.1 线速度标定 (`linear_calibration.py`)

**功能**: 控制机器人直线行驶指定距离，比较里程计报告距离与实际测量距离。

**使用方法**:
```bash
# 默认前进 1 米
ros2 run robot_base linear_calibration.py

# 指定前进距离 (例如 2 米)
ros2 run robot_base linear_calibration.py 2.0
```

**标定流程**:
1. 在地面标记起点。
2. 运行脚本，机器人会自动前进到目标距离。
3. 使用卷尺测量实际行驶距离。
4. 计算校正系数：`correction = 实际距离 / 里程计报告距离`
5. 将校正系数更新到 `controllers.yaml` 或硬件接口中。

**关键参数** (可在脚本中修改):
- `self.speed`: 测试速度 (默认 0.4 m/s)
- `self.tolerance`: 停止容差 (默认 0.01 m)

#### 5.2 角速度标定 (`angular_calibration.py`)

**功能**: 控制机器人原地旋转指定角度 (默认 π 弧度 = 180°)，比较里程计和 IMU 的角度测量。

**使用方法**:
```bash
ros2 run robot_base angular_calibration.py
```

**标定流程**:
1. 在机器人上标记一个方向参考点。
2. 运行脚本，机器人会原地旋转 180°。
3. 使用量角器或视觉参考检查实际旋转角度。
4. 脚本会同时显示里程计 (`self.yaw`) 和 IMU (`self.imu_data`) 的角度读数。
5. 如果两者有偏差，调整轮距参数或 IMU 外参。

**关键参数** (可在脚本中修改):
- `self.target_yaw`: 目标旋转角度 (默认 π)
- `self.r_speed`: 旋转速度 (默认 1.0 rad/s)
- `self.r_tolerance`: 停止容差 (默认 0.1 rad)

**常见问题**:
| 现象 | 可能原因 | 解决方案 |
| :--- | :--- | :--- |
| 实际距离 > 里程计 | 轮径参数偏小 | 增大 `wheel_radius` |
| 实际距离 < 里程计 | 轮径参数偏大 | 减小 `wheel_radius` |
| 旋转角度偏差大 | 轮距参数不准 | 调整 `wheel_separation` |
| 里程计与 IMU 角度不一致 | IMU 外参错误 | 校准 IMU 安装角度 |

---

### 6. 实时性能监控 (ROS 2)

**话题频率检查**:
```bash
ros2 topic hz /scan
ros2 topic hz /Odometry
ros2 topic hz /cmd_vel
```

**节点延迟分析**:
```bash
ros2 topic delay /cmd_vel
```

**TF 树检查**:
```bash
ros2 run tf2_tools view_frames
# 生成 frames.pdf
```

**计算图可视化**:
```bash
ros2 run rqt_graph rqt_graph
```

---

### 7. 性能优化建议

| 问题 | 诊断方法 | 解决方案 |
| :--- | :--- | :--- |
| **导航卡顿** | `top` 查看 CPU | 降低 MPPI `batch_size`/`time_steps` |
| **定位漂移** | EVO RPE 分析 | 调整 IMU 外参、增加特征点 |
| **路径不平滑** | 可视化 `/plan` | 切换 ConstrainedSmoother |
| **TF 延迟** | `ros2 topic delay` | 检查传感器时间同步 |
| **内存泄漏** | `top` 监控内存 | 检查地图累积、降采样 |

---

## 硬件依赖

- **移动底盘**: 四轮差速或全向底盘 (支持串口通信)
- **激光雷达**: Livox Mid360 (360度混合固态激光雷达)
- **IMU**: 外部高精度 IMU (如 Hipnuc) 或雷达内置 IMU
- **计算平台**: x86 PC (推荐) 或 Nvidia Jetson (Orin/Xavier)

## 软件依赖

- **OS**: Ubuntu 22.04 LTS
- **ROS Distribution**: ROS 2 Humble Hawksbill
- **核心库**:
    - `pcl_ros`
    - `nav2-bringup`
    - `nav2-mppi-controller`
    - `xacro`
    - `robot_localization` (ROS 2 standard pkg)

## 安装与编译

1. **克隆代码库**
   ```bash
   cd ~/voxel_ws/src
   # clone your repo here
   ```

2. **安装依赖**
   使用 `rosdep` 安装所有依赖项：
   ```bash
   cd ~/voxel_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **编译项目**
   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

4. **配置环境变量**
   ```bash
   source install/setup.bash
   # 建议添加到 ~/.bashrc
   echo "source ~/voxel_ws/install/setup.bash" >> ~/.bashrc
   ```

## 核心功能与使用

### 1. 硬件驱动启动
如果不进行导航，仅需启动底层硬件（底盘、雷达、IMU）：

```bash
ros2 launch robot_base bringup.launch.py
```
*这将启动底盘通信、加载 URDF 模型、发布 `/tf` 和 `/tf_static`。*

### 2. 建图模式 (Mapping)
使用 Fast-LIO 进行 3D 点云建图：

```bash
ros2 launch fast_lio mapping.launch.py
```
**操作流程**:
1. 启动命令后，RViz 会自动打开。
2. 遥控机器人移动，覆盖所有区域。
3. 建图完成后，地图会保存为 PCD 文件或通过服务调用保存。

### 3. 导航与定位 (Navigation & Localization)
启动完整的导航栈（包含定位、路径规划、避障）：

```bash
ros2 launch robot_navigation bringup.launch.py
```
**包含模块**:
- `livox_ros_driver2`: 启动雷达
- `pointcloud_to_laserscan`: 将 3D 点云压平为 2D 扫描数据
- `fast_lio` (Localization Mode): 运行定位模式
- `nav2_bringup`: 启动规划器、控制器 (MPPI)、代价地图等

**使用方法**:
1. 在 RViz 中使用 "2D Pose Estimate" 给定初始位姿（如果未自动初始化）。
2. 使用 "2D Goal Pose" 发布目标点。
3. 机器人将自动规划路径并避障行驶。

## 参数配置说明

### 导航参数 (`nav2_params.yaml`)
路径：`src/robot_navigation/src/params/nav2_params.yaml`

关键参数：
- **Controller**: 使用 `MPPIController` 进行路径跟踪。
    - `vx_max`: 最大线速度 (目前设为 0.6 m/s)
    - `batch_size`: 采样批次大小 (影响计算负载与平滑度)
    - `time_steps`: 预测时间步长
- **Costmap**: 本地和全局代价地图配置
    - `inflation_radius`: 膨胀半径
    - `resolution`: 地图分辨率 (0.05m)

### Fast-LIO 参数 (`mid360.yaml`)
路径：`src/robot_localization/fast_lio/config/mid360.yaml`

关键参数：
- `common/lid_topic`: 雷达话题 (`/livox/lidar`)
- `common/imu_topic`: IMU 话题
- `mapping/extrinsic_T`: 雷达外参平移
- `mapping/extrinsic_R`: 雷达外参旋转

## 常见问题与故障排除

**Q: 机器人移动卡顿，一会快一会慢？**
A: 这通常是 MPPI 控制器计算负载过高导致的。
- **解决**: 检查 `nav2_params.yaml`，尝试降低 `batch_size` (如 1000) 和 `time_steps` (如 50)。

**Q: 无法提交代码 (Git commit 失败)？**
A: 检查是否误添加了 `.cache` 等大量临时文件。
- **解决**: 使用 `git restore --staged .cache` 撤销暂存，并确保 `.gitignore` 包含 `.cache/`。

**Q: 雷达数据在 RViz 中是歪的？**
A: 检查 URDF 中的雷达安装角度以及 `mid360.yaml` 中的外参配置 (`extrinsic_R`)。如果是倾斜安装，必须正确补偿安装角。

**Q: 地面分割效果不好？**
A: 调整 `linefit_ground_segmentation` 参数：
- 增大 `max_dist_to_line` 可放宽地面判定
- 调整 `sensor_height` 确保与实际安装高度一致

**Q: IMU 姿态漂移？**
A: 检查 `imu_complementary_filter` 的 `gain_acc` 参数，过小会导致漂移，过大会导致噪声。

---

## License

MIT License

---
