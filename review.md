# Ubuntu 24.04 + ROS 2 Jazzy 适配 Code Review（voxel_ws）

## 0. 你给的信息（简要复述）

- 你把系统从 Ubuntu 22.04 升级到 Ubuntu 24.04。
- 现有工作空间代码原本面向 Ubuntu 22.04 + ROS 2 Humble，需要“完全适配新系统”。

结合我在本机 workspace 的检查结果：当前环境已经是 **Ubuntu 24.04.3 + ROS 2 Jazzy**（`ROS_DISTRO=jazzy`），代码能编译，但仍存在较多“文档/配置/构建脚本仍停留在 Humble/22.04 时代”的耦合点，需要系统性清理。

## 1. 环境基线（本机实测）

- OS: Ubuntu 24.04.3 LTS（Noble）
- ROS: ROS 2 Jazzy（`/opt/ros/jazzy`）
- Toolchain: GCC 13.3.0、CMake 3.28.3
- Python: 3.12.3

## 2. 信息缺口 & 风险点（迁移时最容易踩坑）

- **目标 ROS 版本未显式确认**：Ubuntu 24.04 官方对应 ROS 2 Jazzy；若你坚持 Humble，需要容器/源码编译，成本和风险显著更高。
- **平台与实时性约束不明**：x86 / Jetson 算力差异会直接影响 MPPI / LIO 实时性；参数与线程策略需要按平台调。
- **传感器 topic/时戳模式不明**：IMU/LiDAR 是否硬件时间戳、是否同源时钟，直接影响 LIO 和 TF 同步；OS 升级后驱动/内核变化也可能改变时间行为。

> 下面建议默认按“Ubuntu 24.04 + ROS 2 Jazzy 原生适配”展开；如果你要继续用 Humble，请先告诉我你的选择。

## 3. 工作空间包清单（colcon 可见 22 个 ament_cmake 包）

| 包 | 路径 | 作用 | Jazzy 风险 |
|---|---|---|---|
| `robot_navigation` | `src/robot_navigation/src` | Nav2 参数/launch/map/rviz | 中高（参数/BT 变更） |
| `robot_route` | `src/robot_route` | Route Server 相关桥接脚本/配置 | 中（依赖声明/异步模型） |
| `robot_base` | `src/robot_base` | ros2_control 硬件接口 + 工具脚本 | 中（驱动/串口/实时） |
| `livox_ros_driver2` | `src/robot_hardware_driver/livox_ros_driver2` | Livox 驱动 | 中（外部 SDK/ABI） |
| `hipnuc_imu`/`hipnuc_gnss`/`hipnuc_lib_package` | `src/robot_hardware_driver/hipimu_driver/*` | Hipnuc IMU/GNSS | 低-中（设备权限/串口） |
| `fast_lio`/`point_lio` | `src/robot_localization/*` | LIO 里程计/建图 | 中（CMake/PythonLibs） |
| `faster_lio` | `src/robot_localization/faster-lio` | Faster-LIO | 中（CMake/rosidl 过时接口） |
| `lio_sam` | `src/robot_localization/LIO-SAM` | LIO-SAM | 中（rosidl 过时接口） |
| `icp_registration` | `src/robot_relocalization/icp_registration` | ICP 重定位 | 低（头文件安装告警） |
| `pointcloud_to_laserscan` | `src/robot_perception/pointcloud_to_laserscan` | 点云转激光 | 低（头文件安装告警） |
| `imu_complementary_filter` | `src/robot_perception/imu_complementary_filter` | IMU互补滤波 | 低 |
| `linefit_ground_segmentation*` | `src/robot_perception/linefit_ground_segementation_ros2/*` | 地面分割 | 低 |
| `pcd2pgm`/`pcd2elevation` | `src/tools/*` | 地图工具 | 低 |
| `waypoint_editor` | `src/robot_app/waypoint_editor` | 路点编辑器 | 低（文档需更新） |
| `costmap_converter*` | `src/robot_navigation/third_party/costmap_converter/*` | costmap 转换 | 低（遗留脚本 shebang） |
| `rviz_imu_plugin` | `src/robot_hardware_driver/hipimu_driver/rviz_imu_plugin` | RViz IMU 插件 | 低 |

## 4. 主要问题与修改建议（面向 Ubuntu24 + Jazzy）

### 4.1 文档/镜像仍写死 Humble/Ubuntu22（需要统一口径）

发现：
- `README.md` 仍声明 “Ubuntu 22.04 + ROS 2 Humble”。
- `src/robot_app/RVIZ-RQT-VISUA/RVIZ-RQT-VISUAL-main/Dockerfile` 使用 `ros:humble-perception-jammy` 且 `source /opt/ros/humble/setup.bash`。
- `src/robot_app/RVIZ-RQT-VISUA/RVIZ-RQT-VISUAL-main/frontend/.../SystemInfo.vue` 写死 “ROS2 Humble”。
- 多个第三方 README 仍以 Humble/22.04 为例（不致命，但会误导新环境部署）。

建议（KISS）：
- 把 **仓库根 `README.md`** 作为唯一“权威安装指南”，明确：
  - Ubuntu 24.04 → ROS 2 Jazzy（推荐/默认）
  - Ubuntu 22.04 → ROS 2 Humble（兼容/历史）
- Dockerfile 用 `ARG ROS_DISTRO=jazzy` + `FROM ros:${ROS_DISTRO}-...`，避免散落多份 Humble 固化信息（DRY）。

### 4.2 Nav2 Jazzy：BT 与参数文件是迁移主战场

#### 4.2.1 BT 插件库缺项（Route Server 相关）

发现：
- `src/robot_navigation/src/params/nav2_params.yaml` 的 `bt_navigator.plugin_lib_names` 列表里 **没有** route 相关 BT node（如 `nav2_compute_and_track_route_bt_node` / `nav2_compute_route_bt_node`）。
- `src/fix/action.md` 已指出：旧版 `navigate_on_route_graph_w_recovery.xml` 不适配 Jazzy，并给了 Arch(1) 新 BT 的落地方案。

建议（SRP + KISS）：
- **把 Route 专用 BT 单独成文件**（不要污染默认 NavToPose BT），例如按 `src/fix/action.md` 的方式新建 `navigate_on_route_graph_arch1_jazzy.xml`。
- 同步把 route BT node 动态库加入 `plugin_lib_names`，否则 BT 加载会失败。

#### 4.2.2 旧参数/无效参数：Jazzy 下会“静默失效”

发现：
- `nav2_params.yaml` 中 MPPI 段落存在 “Humble only” 注释参数；其中 `reset_period` 在当前 Jazzy 安装里未检索到（高度怀疑已移除/重命名），会导致你以为生效但实际被忽略。
- `FollowPath` 下的自定义参数（`adjustThre/minAngleDiff/...`）只有在你**自研/改过** `nav2_mppi_controller` 时才可能生效；否则同样会被忽略。

建议（工程可验证性优先）：
- 启动后用 `ros2 param list /controller_server` 和 `ros2 param get` 实测参数是否存在（避免“看 YAML 调参”的错觉）。
- 对“不被支持的参数”直接删掉（YAGNI），把有效参数收敛成一份 Jazzy 可验证的最小集合（KISS）。

#### 4.2.3 你现有的 nav2 参数还存在业务级风险（与 OS 升级叠加会放大）

这些在 `src/fix/fix.md` 已点到，我建议当作 Jazzy 迁移验收项：
- `yaw_goal_tolerance: 3.14` 会让“到点转正再前进”失效。
- `minAngleDiff: 365.0` 量纲明显不合理（MPPI 多数参数用弧度）。
- `odom_topic: "/Odometry"` 需要确认真实话题，否则控制器无里程计会表现为“卡住/不动”。

### 4.3 CMake 3.28 / GCC13 下的构建兼容性问题（建议尽早还债）

#### 4.3.1 `find_package(PythonLibs)` 已被 CMake 弃用（Ubuntu24 典型告警源）

发现：
- `src/robot_localization/fast_lio/CMakeLists.txt`、`src/robot_localization/point_lio/CMakeLists.txt` 使用 `find_package(PythonLibs REQUIRED)`，在 CMake 3.28 下触发 CMP0148 相关告警。

建议（可维护性 + 向前兼容）：
- 用 `find_package(Python3 COMPONENTS Interpreter Development REQUIRED)` 替换，并改用 `Python3::Python` / `Python3_INCLUDE_DIRS` / `Python3_LIBRARIES`。
- 如果仅用于 `matplotlibcpp.h` 可视化且运行时不需要，建议改成 **编译开关**（例如 `WITH_MATPLOTLIB=OFF` 默认关闭）（YAGNI + KISS）。

#### 4.3.2 全局乱改 `CMAKE_CXX_FLAGS` / 标准版本冲突（会放大工具链升级风险）

典型问题：
- `fast_lio` 同时设置 `CMAKE_CXX_STANDARD 14`、又手动塞 `-std=c++17`，还重复添加 `-std=c++0x` 等。
- `faster_lio` 里存在未 `find_package` 就引用 `${EIGEN3_INCLUDE_DIR}` / `${yaml-cpp_INCLUDE_DIRS}` / `${PYTHON_*}` 的情况（目前可能“恰好能编”，但非常脆）。

建议（KISS + DRY）：
- 统一用 `set(CMAKE_CXX_STANDARD 17)` + `target_compile_options`（按 target 粒度），不要全局覆盖 `CMAKE_CXX_FLAGS`。
- 把依赖显式 `find_package` 出来，避免“靠系统 include 路径碰运气”。

#### 4.3.3 `rosidl_target_interfaces()` 在 Jazzy 已提示弃用

发现：
- `src/robot_localization/LIO-SAM/CMakeLists.txt`、`src/robot_localization/faster-lio/CMakeLists.txt` 使用 `rosidl_target_interfaces()`，Jazzy 构建会提示弃用。

建议（OCP：为后续发行版保留扩展空间）：
- 按 CMake 提示替换为：
  - `rosidl_get_typesupport_target(cpp_typesupport_target ...)`
  - `target_link_libraries(target "${cpp_typesupport_target}")`

### 4.4 package.xml 依赖声明不完整/不规范（影响 rosdep 与可复现部署）

发现：
- `robot_navigation` 是纯配置包，但 `package.xml` 未声明 `nav2_*` 相关运行依赖；新机器用 `rosdep install` 很可能装不全。
- `robot_route` 安装了 Python 可执行脚本，但 `package.xml` 未声明 `rclpy`、`nav2_msgs`、`tf2_ros`、`tf2_geometry_msgs`、`geometry_msgs`、`nav_msgs` 等运行依赖。
- 多处直接把 apt 包名写进 `<depend>`（如 `libpcl-all-dev`/`libserial-dev`/`git`/`apr`），跨平台/跨发行版时 rosdep 易失败。

建议（DRY + 可复现）：
- 依赖声明按“能 rosdep 一键装全”为目标整理；对于系统包，尽量用 rosdep key（必要时在 README 里明确 apt 包名作为补充）。

### 4.5 Python 脚本 shebang / ROS1 遗留物（Ubuntu24 常见运行时炸点）

发现：
- `src/robot_localization/faster-lio/result/rpe_odom.py` 使用 `#!/usr/bin/env python`（Ubuntu24 默认可能没有 `python`）。
- `src/robot_localization/LIO-SAM/config/doc/kitti2bag/kitti2bag.py` 是 ROS1 脚本（`rospy/rosbag/tf`），且 shebang 写成 `#!env python`。
- `costmap_converter` 里存在 ROS1 dynamic_reconfigure 风格 `.cfg`，同样是 `#!/usr/bin/env python`（多半不在 Jazzy 运行路径上，但会误导维护者）。

建议（SRP + KISS）：
- 明确标注哪些是“仅文档/历史遗留/ROS1 工具”，必要时放到 `tools_ros1/` 或 README 标红；否则新同事会以为这是 Jazzy 运行链路的一部分。
- 需要继续使用 ROS1 工具时，用容器隔离（避免污染 Jazzy 环境）。

## 5. 建议的迁移执行顺序（3~8 个可落地子任务）

1) **口径统一**：更新根 `README.md`，明确 Jazzy/Noble 为默认；同步修正 Dockerfile 和前端显示版本（DRY）。
2) **Nav2 Jazzy 路网链路打通**：按 `src/fix/action.md` 落地新 BT + 补齐 `plugin_lib_names`，并清理无效参数（KISS）。
3) **补齐依赖声明**：修正 `robot_navigation/package.xml`、`robot_route/package.xml` 的 `exec_depend`（可复现部署）。
4) **修 CMake 弃用点**：替换 `PythonLibs`、替换 `rosidl_target_interfaces`、清理 `CMAKE_CXX_FLAGS` 乱改（向后兼容）。
5) **处理 Python shebang/ROS1 遗留**：将 `#!/usr/bin/env python` 统一为 python3，或隔离到 ROS1 工具目录（降低运行时踩坑）。

## 6. 验证建议（回放/实机都能做）

### 6.1 构建验证

```bash
cd ~/voxel_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.zsh
```

### 6.2 Nav2/Route 验证（建议先 bag 回放再上实机）

启动：
```bash
ros2 launch robot_navigation navigation.launch.py
ros2 lifecycle get /route_server
ros2 action list | grep route
```

录包（最小闭环链路 + 排障必备）：
```bash
ros2 bag record \
  /tf /tf_static \
  /scan /pointcloud \
  /imu /Odometry \
  /cmd_vel \
  /plan \
  /route_graph \
  /local_costmap/costmap_raw /global_costmap/costmap_raw
```

关键观测指标（建议写进你们的迁移验收）：
- TF 延迟：`ros2 run tf2_tools view_frames`，重点看 `map->odom->base_link` 是否稳定、延迟是否增大。
- 控制实时性：`controller_frequency` 实测是否达标（CPU 占用/超时日志）。
- 轨迹偏差：对 `/plan`（或 route dense path）计算横向偏差 RMS，建议 < 0.1 m（按场景调整）。

### 6.3 LIO 验证（时间同步是 Ubuntu 升级后第一大雷）

建议录包：
```bash
ros2 bag record \
  /tf /tf_static \
  /imu \
  /livox/lidar /points_raw /points \
  /Odometry
```

检查点：
- IMU/LiDAR 时间戳是否单调、是否同源（不同源就要做 time offset / deskew 配置）。
- LIO 输出频率、CPU 占用是否较 22.04 有明显变化（内核/编译器升级会改变性能特性）。

## 7. 下一步我需要你确认的两件事（决定迁移路线）

1) 你最终要 **Jazzy** 还是强行保留 **Humble**（容器/源码）？
2) 目标平台：x86 还是 Jetson（Orin/Xavier）？（决定 MPPI/LIO 的计算预算与参数收敛方式）

---

## 8. 近期实机问题：发点跑不起来 + 地面分割不干净（基于当前代码链路的结构性分析）

### 8.0 你给的新信息（简要复述）

- 目前“发点（点云/或最终 /scan）无法正常跑”，表现为链路某段没有输出或下游一直报 TF/时间相关错误。
- 雷达前置且有明显俯仰倾斜，导致地面分割总有残留点；你已经尝试调参但没有根本解决。

信息缺口 & 风险点（会直接决定修复方向）：
- 不清楚你说的“发点”具体指哪一段：`/livox/lidar`（原始驱动）、`/lidar/cloud`（tilt_compensator 后）、`/cloud_registered_body`（Fast-LIO 后）、还是 `/scan`（导航最终输入）。
- 不清楚雷达倾角是否固定（装配角）还是会随底盘姿态变化（上下坡/加减速俯仰）；后者需要“动态重力对齐”，单纯静态 pitch 参数必然会漏点。
- 不清楚你是否在用 namespace（当前 `robot_navigation/src/launch/bringup.launch.py` 声明了 `namespace` 但未真正应用），而多个节点使用了**绝对话题名**（前导 `/`），容易导致命名空间场景下订阅不到数据。

> 下面先按“倾角主要来自安装固定角 + 平地行驶为主”的假设给出改进方案；如果你实际经常上下坡/颠簸，需要把方案切到“IMU 动态对齐”。

### 8.1 当前链路（从 launch/代码还原的数据流与 I/O）

入口节点（硬件与预处理）：
- `src/robot_base/launch/bringup.launch.py` 启动：
  - `livox_ros_driver2_node`：发布 `/livox/lidar`（`livox_ros_driver2/msg/CustomMsg`）与 `/livox/imu`（`sensor_msgs/msg/Imu`，取决于驱动配置）。
  - `robot_base/tilt_compensator`：订阅 `/livox/lidar`、`/livox/imu`，发布 `/lidar/cloud`（CustomMsg）与 `/lidar/imu`（Imu）。

LIO（里程计与注册点云）：
- `src/robot_navigation/src/launch/bringup.launch.py` 启动 `fast_lio/fastlio_mapping`：
  - 订阅（来自 `src/robot_localization/fast_lio/config/mid360.yaml`）：
    - `common.lid_topic: /lidar/cloud`
    - `common.imu_topic: /lidar/imu`
  - 发布：
    - `/cloud_registered_body`（`sensor_msgs/msg/PointCloud2`，`frame_id=body`，见 `src/robot_localization/fast_lio/src/laserMapping.cpp`）

地面分割与 2D 化：
- `linefit_ground_segmentation_ros/ground_segmentation_node`：
  - 订阅：当前 launch 覆盖成 `input_topic=/cloud_registered_body`
  - 输出：`/segmentation/obstacle`（供下游转 `/scan`）
  - 关键点：该节点支持参数 `gravity_aligned_frame`，会用 TF 只取旋转（去平移）做“重力对齐”后再分割（见 `src/robot_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation_ros/src/ground_segmentation_node.cc`）。
- `pointcloud_to_laserscan`：
  - 输入：`/segmentation/obstacle`
  - 输出：`/scan`
  - 依赖：必须能在消息时间戳处查到 `cloud_in.frame_id -> target_frame` 的 TF。

### 8.2 “发点无法正常跑”的高概率断点（按排查优先级）

1) 消息类型误判（最常见）
- `/livox/lidar` 在当前配置下是 `CustomMsg`，RViz/很多工具默认按 `PointCloud2` 期待，容易误以为“没发点”。
- 先用这条确认类型与发布者：
  - `ros2 topic info -v /livox/lidar`

2) TF/时间戳导致下游丢数据（最容易表现为“/scan 不出”）
- `linefit` 在 `gravity_aligned_frame` 非空时会做 TF lookup；`pointcloud_to_laserscan` 也需要 TF。
- 只要**时间戳不单调**、**TF 缺失**、或**TF 与消息时间不同步**，就会出现：
  - TF extrapolation / lookupTransform failed
  - 下游不发布 `/scan`（或频率极低）

3) `tilt_compensator` 的“强行 now() 时间戳”带来的结构性副作用
- 见 `src/robot_base/src/tilt_compensator.cpp`：
  - LiDAR/IMU 的 `header.stamp` 被直接覆盖为 `now()`（两个回调各自 now，存在抖动与不一致）
  - 但 `CustomMsg.timebase` 与 `points[].offset_time` 仍是驱动侧时间语义
- 这会让“看起来时间同步了”，但在需要严格时间语义的模块里（deskew、TF、回放）变得更不可控：**调参很难救回来**。

4) TF 连接关系与几何外参不一致（会同时影响地面分割与 2D 化）
- 当前 `src/robot_navigation/src/launch/bringup.launch.py` 里给了 `body -> base_footprint` 的静态 TF，但只填了 z（`-0.55`），没有把雷达的 `x/y` 平移与真实俯仰角放进 TF。
- 如果你希望用 `linefit.gravity_aligned_frame` 来“把倾斜摆平”，那就必须让 TF 里包含真实旋转；否则 gravity alignment 等于没做。

### 8.3 为什么“调 linefit 参数”不一定能根治（从 SRP/数据流一致性角度）

核心矛盾是：**当前系统把“几何外参/姿态对齐”混在数据预处理里（tilt_compensator），但 TF/URDF 又假设传感器没倾斜**。

- SRP 被破坏：`tilt_compensator` 同时在做“坐标系定义改变 + 时间戳重写 + IMU 旋转”，这会影响 Fast-LIO、地面分割、以及任何依赖 TF 的节点；你调 `linefit` 参数只能治“表面症状”。
- 数据流输入输出语义不一致：点云已经被旋转，但 TF 里仍是“未旋转”的传感器关系，导致下游“以为自己在某个 frame”实际上不是。
- 结果就是：地面分割会出现“总有一些点漏掉”，而且参数怎么调都只能在“漏地面”和“吃掉障碍物”之间摇摆。

### 8.4 建设性改进路线（优先从最小改动、可回滚方案开始）

#### 方案 A（推荐，KISS + SRP）：把“倾斜矫正”从数据里拿出来，放回 TF；地面分割用 gravity_aligned_frame 做旋转对齐

目标：Fast-LIO 用真实传感器数据跑（保持时序/外参一致），地面分割只在需要时做“旋转摆平”，彼此解耦。

落地步骤（建议按顺序做，方便定位收益）：
1) 把“真实安装倾角”写回 TF/URDF
   - `src/robot_base/urdf/robotcar.xacro` 的 `lidar_pitch` 当前为 `0.0`，但你的雷达实际是倾斜安装。
   - 建议：让 `base_link -> livox_frame` 反映真实物理安装（rpy 里包含 pitch），不要再靠“旋转点云数据”来伪造水平。
2) 停止在基础链路里改写时间戳（避免把时间问题扩散到所有模块）
   - 原则：驱动产生的时间戳要么“原样保留”，要么“统一在一个地方做明确的 time sync/offset”，不要在中间节点随手 `now()`。
3) 让 linefit 只负责分割（SRP）
   - 设置 `gravity_aligned_frame: base_link`（或 `base_footprint`），让它用 TF 的旋转对齐后再分割。
   - 这样你只需要把 TF 做对，地面分割会立刻变得“可调且稳定”，而不是靠大 `max_dist_to_line/max_slope` 硬顶。
4) 再做参数微调（DRY：避免在多个地方重复“补救”同一个倾角）
   - 倾角问题交给 TF 解决后，再收紧 `max_dist_to_line/max_slope/max_start_height`，避免把真实障碍物也当作地面吞掉。

优点（为什么推荐）：
- KISS：不引入新算法，只是把系统语义摆正。
- SRP：Fast-LIO 负责状态估计；TF 负责几何外参；linefit 只做分割。
- DRY：倾角只在一个地方（TF/URDF）定义一次，不再在参数、预处理、URDF 三处“打补丁”。

#### 方案 B（次选，兼容现状）：保留 tilt_compensator，但把它“隔离”为导航专用分支，不影响 LIO 主链路

目标：Fast-LIO 仍吃“原始 + 时间语义正确”的 IMU/LiDAR；导航再单独用一个“摆平后的点云”去做地面分割/转 /scan。

建议做法（原则级，不展开到完整代码）：
- 让 `tilt_compensator` 输出一个单独的话题（例如 `/lidar/cloud_leveled`），只给 `linefit/pointcloud_to_laserscan` 用。
- Fast-LIO 继续订阅 `/livox/lidar`（或驱动转换后的原始）与 `/livox/imu`，避免“旋转 IMU + 改时间戳”影响状态估计。

> 这条路体现 ISP：不同模块只拿自己真正需要的数据，不要共享“被强行改写语义”的通道。

#### 方案 C（你经常上下坡/颠簸时才需要）：用 IMU 动态重力对齐替代固定 pitch

固定 pitch 只能修“装配角”，修不了“底盘姿态变化”。如果你的地面残留点在加减速、上坡、过坎时显著变多，就应该切到动态方案：
- 以 IMU 的姿态（roll/pitch）实时生成旋转，将点云对齐到重力方向后再做地面分割。
- 这可以继续沿用 linefit 的 `gravity_aligned_frame` 思路，只是 TF 必须能表达“随时间变化的重力对齐关系”（通常来自滤波后的 IMU 姿态）。

### 8.5 建议的验证与复现（rosbag/回放/指标）

#### 8.5.1 最小闭环录包（定位“发点卡在哪一段”）

```bash
ros2 bag record \
  /tf /tf_static \
  /livox/lidar /livox/imu \
  /lidar/cloud /lidar/imu \
  /cloud_registered_body \
  /segmentation/obstacle /segmentation/ground \
  /scan
```

你应该能通过 bag 回放确认：
- 哪个 topic 从某一段开始就没有数据（频率为 0 或突然中断）
- TF 是否能在每条点云 stamp 上查到（是否大量出现 extrapolation）

#### 8.5.2 回放建议（尽量复现问题而不依赖实机）

```bash
ros2 bag play <bag_dir> --clock
ros2 launch robot_navigation bringup.launch.py use_sim_time:=true
```

#### 8.5.3 关键观测指标（建议作为“修复是否有效”的客观标准）

- `/scan` 连续性：`ros2 topic hz /scan` 是否稳定接近预期（10Hz/20Hz）。
- TF 可用性：`ros2 run tf2_ros tf2_echo base_link body` 是否持续输出且没有时间跳变。
- 地面残留比例：在 RViz 里同时看 `/segmentation/ground` 和 `/segmentation/obstacle`，理想状态下近距离地面点大部分进入 ground（残留点主要来自轮子/底盘遮挡与近场噪声）。
- LIO 稳定性：Fast-LIO 是否还出现 “IMU and LiDAR not Synced”/loop back 等告警（如果有，先修时间语义，再谈分割参数）。

---

如果你愿意，我下一轮可以按你当前“发点”的具体含义（你告诉我是 `/livox/lidar` 还是 `/scan`）再把排查步骤收敛成一条“最短路径”，并给出对应的最小代码改动点（包含要改的 launch/URDF/参数项）。  
