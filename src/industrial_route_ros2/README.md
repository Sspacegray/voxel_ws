# Industrial Route (ROS 2 Humble) — 半结构化路网导航（Nav2 解耦）

本模块把“半结构化路网导航（在路网/不在路网）”做成 **可在 ROS 2 Humble 工业落地**的独立组件：
- 路网以 **GeoJSON** 定义（拓扑 + 边 polyline + 交通属性）
- 支持 **在网规划** + **离网上网/下网**（投影到边/连接到近邻节点）
- 内置差速底盘执行器（Pure Pursuit + 原地转向对齐）
- 不依赖 `nav2_route`（也不要求 Nav2 生命周期）；可选启用 `/map` OccupancyGrid 做连线碰撞检查

## 包结构

- `industrial_route_core`：纯 C++ 核心库（无 ROS 依赖）
  - GeoJSON 加载、图结构、Dijkstra 规划、MapMatching（最近边/点投影）、稠密化路径构建
  - 可执行工具：`industrial_route_validate_graph`
- `industrial_route_interfaces`：ROS2 msg/srv/action 定义
- `industrial_route_ros2`：ROS2 节点与执行器
  - `industrial_route_server`：集成 Graph+Planner+Executor 的一体化服务节点

## 快速开始

### 1) 编译（Humble）

```bash
source /opt/ros/humble/setup.bash  # 或 setup.zsh
cd ~/voxel_ws
colcon build --packages-select industrial_route_interfaces industrial_route_core industrial_route_ros2
source install/setup.bash
```

### 2) 启动（自带 sample graph）

```bash
ros2 launch industrial_route_ros2 industrial_route.launch.py
```

默认会加载 `src/industrial_route_ros2/config/sample_graph.geojson`，并发布 RViz Marker：
- `/industrial_route/graph_markers`

### 3) 一键“规划+执行”（示例脚本）

```bash
ros2 run industrial_route_ros2 plan_and_execute.py
```

前提：你系统里已经有 TF `map -> base_link`（比如仿真/定位节点在跑）。没有 TF 时执行器会停止并报 `NO_TF`。

## ROS 2 接口

### Topics
- `/industrial_route/graph` (`industrial_route_interfaces/msg/RouteGraph`)：路网（latched）
- `/industrial_route/graph_markers` (`visualization_msgs/msg/MarkerArray`)：RViz 可视化（latched）
- `/industrial_route/path` (`nav_msgs/msg/Path`)：最近一次规划得到的执行路径（latched）
- `/industrial_route/status` (`industrial_route_interfaces/msg/RouteStatus`)：执行状态与进度
- `/cmd_vel` (`geometry_msgs/msg/Twist`)：执行器输出（可通过参数改 topic）

### Services
- `/industrial_route/plan` (`industrial_route_interfaces/srv/PlanRoute`)：规划（pose↔pose / node↔node）
- `/industrial_route/get_nearest_node` (`industrial_route_interfaces/srv/GetNearestNode`)：最近节点查询
- `/industrial_route/update_edge` (`industrial_route_interfaces/srv/UpdateEdge`)：动态启用/禁用边、限速、权重、单双向
- `/industrial_route/reload_graph` (`std_srvs/srv/Trigger`)：重新加载路网文件
- `/industrial_route/pause` (`std_srvs/srv/SetBool`)：暂停/恢复执行（`data=true` 暂停）
- `/industrial_route/cancel` (`std_srvs/srv/Trigger`)：取消当前执行

### Action
- `/industrial_route/execute` (`industrial_route_interfaces/action/ExecuteRoute`)：执行路径/路由
  - `use_path=true`：直接执行输入的 `nav_msgs/Path`
  - `use_node_sequence=true`：按 node 序列尝试拼接边并执行（best-effort）

## GeoJSON 路网格式（当前实现支持）

### Node（Point）
`geometry.type = "Point"`，`properties` 至少包含：
- `id`：节点 ID（int 或 string）
- `frame`：建议为 `"map"`（用于数据自描述，当前不强制）

可选：
- `theta`：节点朝向（rad）
- `type`：`NORMAL / STATION / CHARGING / WAIT`

### Edge（MultiLineString 或 LineString）
`geometry.type = "MultiLineString"`（取第一条 polyline）或 `"LineString"`，`properties` 至少包含：
- `id`：边 ID（int 或 string）
- `startid` / `endid`：端点 node id

可选：
- `weight`：边权重（默认 1.0）
- `max_speed`：该边建议最大速度（m/s，默认 0.6）
- `bidirectional`：是否双向（bool/string/int 均可）
- `enabled`：是否启用（默认 true）

## 在网 / 离网 行为

规划 pose→pose 时：
- 若起点/终点距离路网边小于 `planner.on_graph_max_dist`，会先投影到最近边作为“锚点”，保证路径能沿路网 edge 走
- 若不满足投影条件，会在 `planner.connect_max_dist` 内找近邻节点作为连接候选（可选结合 `/map` 做连线碰撞检查）

## 参数（industrial_route.yaml）

核心参数文件：`src/industrial_route_ros2/config/industrial_route.yaml`

常用项：
- `graph_geojson_path`：路网文件路径（建议通过 launch 传参）
- `frame_id`：规划/发布 frame（默认 `map`）
- `base_frame_id`：机器人底盘 frame（默认 `base_link`）
- `cmd_vel_topic`：输出速度 topic
- `use_map_collision_check`：是否使用 `/map` 做连接段碰撞检查（默认 false）

规划器：
- `planner.connect_max_dist`
- `planner.connect_k_nearest`
- `planner.on_graph_max_dist`
- `planner.densify_resolution`

控制器（差速）：
- `controller.max_linear_speed`
- `controller.max_angular_speed`
- `controller.lookahead_min`
- `controller.lookahead_gain`
- `controller.rotate_in_place_yaw_threshold`（角度大于阈值先原地转向）

## 工业落地建议（重要）

- 强烈建议在系统层面接入 **硬急停 / 软急停**；本包当前只提供 `pause/cancel` 软件接口。
- 执行器默认参数偏保守（0.5m/s、1.2rad/s），现场需按底盘能力与安全规范标定。
- 若环境障碍会变化，建议启用 `/map` 碰撞检查，并将离网段升级为栅格 A*（后续迭代点）。

## RViz

加载 `src/industrial_route_ros2/rviz/industrial_route.rviz`，订阅：
- `/industrial_route/graph_markers`

