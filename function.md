# 工业级路径追踪系统功能文档

## 系统概述

基于 ROS 2 的工业级 AGV 路径追踪系统，支持多控制器切换、路径插值、轨迹记录和效果评估。

---

## 1. 系统启动

### 基本启动（推荐）
```bash
ros2 launch wpr_simulation2 navigation.launch.py
```

### 指定路径文件启动
```bash
ros2 launch wpr_simulation2 navigation.launch.py \
  waypoint_file:=/home/suja/voxel_ws/src/robot_app/waypoint_editor/wpfile/xxx.json \
  auto_start:=true
```

### Launch 参数
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `waypoint_file` | `''` | 路点文件路径 (JSON) |
| `auto_start` | `true` | 是否自动开始执行路径 |

---

## 2. Waypoint Editor (RViz 路点编辑器)

### 功能
- 在 RViz 中绘制路点
- 密度选择 (0.1-0.5m)
- 保存时自动插值生成密集点
- 自动保存到 `wpfile/` 目录

### 文件保存位置
```
/home/suja/voxel_ws/src/robot_app/waypoint_editor/wpfile/YYYYMMDD_HHMMSS.json
```

---

## 3. PP Controller (独立 Pure Pursuit 控制器)

### 三态运动
```
IDLE → SPINNING → LINE_TRACKING → COMPLETED
         ↑ 障碍检测 → STOP
```

### 服务
```bash
# 开始执行路径
ros2 service call /pp/start std_srvs/srv/Trigger

# 停止并重置
ros2 service call /pp/stop std_srvs/srv/Trigger

# 暂停
ros2 service call /pp/pause std_srvs/srv/Trigger

# 恢复
ros2 service call /pp/resume std_srvs/srv/Trigger

# 重新加载路径文件
ros2 service call /pp/load_path std_srvs/srv/Trigger
```

### 动态加载新路径
```bash
ros2 param set /pp_controller path_file '/path/to/new/file.json'
ros2 service call /pp/load_path std_srvs/srv/Trigger
ros2 service call /pp/start std_srvs/srv/Trigger
```

### 状态话题
| 话题 | 类型 | 说明 |
|------|------|------|
| `/pp/state` | String | IDLE/SPINNING/LINE_TRACKING/COMPLETED |
| `/pp/progress` | Float32 | 执行进度 0.0~1.0 |
| `/pp/obstacle` | Bool | 障碍检测状态 |
| `/pp/lateral_deviation` | Float32 | 横向偏差 (m) |

---

## 4. 轨迹记录器

### 服务
```bash
# 开始记录
ros2 service call /trajectory/start std_srvs/srv/Trigger

# 停止记录
ros2 service call /trajectory/stop std_srvs/srv/Trigger

# 保存轨迹为 TUM 格式
ros2 service call /trajectory/save std_srvs/srv/Trigger

# 清除轨迹
ros2 service call /trajectory/clear std_srvs/srv/Trigger
```

### 话题
| 话题 | 类型 | 说明 |
|------|------|------|
| `/actual_trajectory` | nav_msgs/Path | 实际轨迹路径 (RViz 可视化) |
| `/actual_trajectory_marker` | Marker | 轨迹标记 |

---

## 5. 控制器切换

### 配置文件
```
/home/suja/voxel_ws/src/robot_simulation/wpr_simulation2/config/nav2_params.yaml
```

### 可用控制器
| 选项 | 控制器 | 插件名 |
|------|--------|--------|
| 1 | Nav2 RPP | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` |
| 2 | Custom PP | `pp_controller::Nav2PurePursuitController` |
| 3 | DWB | `dwb_core::DWBLocalPlanner` |
| 4 | TEB | `teb_local_planner::TebLocalPlannerROS` |

### 切换方法
1. 打开 `nav2_params.yaml`
2. 注释掉当前启用的 `FollowPath:` 块
3. 取消注释想使用的 `FollowPath:` 块
4. 重新启动导航

---

## 6. 轨迹评估

### 使用 evo 工具
```bash
# APE (绝对位姿误差)
evo_ape tum planned.tum actual.tum -p

# RPE (相对位姿误差)
evo_rpe tum planned.tum actual.tum -p
```

### 使用内置脚本
```bash
ros2 run robot_route path_comparison.py \
  -p planned.tum \
  -a actual.tum \
  --plot
```

---

## 7. 完整工作流程

### 方式 A: 手动绘制路径
```bash
# 1. 启动系统
ros2 launch wpr_simulation2 navigation.launch.py

# 2. 在 RViz 中绘制路点并保存

# 3. 加载并执行
ros2 param set /pp_controller path_file '/path/to/wpfile/xxx.json'
ros2 service call /pp/load_path std_srvs/srv/Trigger
ros2 service call /pp/start std_srvs/srv/Trigger
```

### 方式 B: 加载预设路径
```bash
# 修改 navigation.launch.py 第 99 行:
waypoint_file_default = '/path/to/wpfile/xxx.json'

# 启动 (自动执行)
ros2 launch wpr_simulation2 navigation.launch.py
```

---

## 8. 关键文件位置

| 文件 | 位置 |
|------|------|
| 导航启动文件 | `wpr_simulation2/launch/navigation.launch.py` |
| 导航参数 | `wpr_simulation2/config/nav2_params.yaml` |
| 路点文件目录 | `waypoint_editor/wpfile/` |
| RViz 配置 | `waypoint_editor/rviz/rviz_waypoint_editor.rviz` |
| PP Controller | `pp_controller/src/pp_node.cpp` |
| 轨迹记录器 | `robot_route/scripts/trajectory_recorder.py` |
| 轨迹比较 | `robot_route/scripts/path_comparison.py` |
