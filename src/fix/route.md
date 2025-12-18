# 工业场景路网导航完整方案 (Route Navigation Guide)

本文档记录了如何在当前工作空间中实现基于 `nav2_route` 的工业路网导航，包括路网创建、配置和导航执行。

---

## 1. 概述

### 什么是路网导航？

与传统的自由空间规划不同，路网导航让机器人沿着**预定义的路径**（节点和边的图结构）进行移动，特别适合：

| 应用场景 | 优势 |
|----------|------|
| 工业/仓库 | 避免走入禁区，遵循安全通道 |
| 多机器人 | 预定义路径减少冲突 |
| 大规模室外 | 减少计算量，复用路径 |

### 架构概览

```
路网创建工具 --> GeoJSON --> Route Server --> 导航执行
   QGIS                        |
   VDA LIF Editor              +--> /route_graph (可视化)
   Nav2 Route Tool             +--> Waypoint Follower --> MPPI
```

---

## 2. 路网创建工具

### 2.1 QGIS 方法（当前使用）

详见：`src/robot_route/doc/route_drew.md`

**流程概要：**
1. 在 QGIS 中加载地图 (1126.pgm)
2. 创建 LineString 图层绘制路径
3. 设置 `id` 和 `bidirectional` 属性
4. 导出为 GeoJSON
5. 运行 `process_graph.py` 转换坐标

**启动命令：**
```bash
python3 src/robot_route/maps/process_graph.py
```

---

### 2.2 VDA LIF Editor（Web 工具）

**特点：**
- 无需安装，浏览器运行
- 支持导入地图图片
- 可视化编辑节点和边

**使用步骤：**
1. 访问 VDA LIF Editor 工具
2. 导入地图图片
3. 点击添加节点和边
4. 导出为 GeoJSON 或 LIF 格式

---

### 2.3 Nav2 Route Tool（RViz 面板）

> **注意**: Humble 默认不自带 Route Tool 面板，需要从 Nav2 主分支编译 `nav2_rviz_plugins` 或使用包含该插件的预编译版本。

**功能：**
- 在 RViz 中直接创建/编辑路网
- 加载现有 GeoJSON 文件
- 添加/编辑/删除节点和边
- 保存为 GeoJSON

**使用方式：**
- 启动 RViz：`ros2 run rviz2 rviz2`
- Panels -> Add Panel -> 选择 “Nav2 Route Tool”
- 在面板中加载/编辑/保存 GeoJSON

---

## 3. 当前配置

### 3.1 Route Server 配置

**配置文件**: `src/robot_route/config/route_server.yaml`

```yaml
route_server:
  ros__parameters:
    use_sim_time: False
    base_frame: "base_link"
    route_frame: "map"
    graph_file_loader: "GeoJsonGraphFileLoader"
    graph_filepath: ""  # overridden by launch argument `graph`

    edge_cost_functions: ["DistanceScorer"]
    DistanceScorer:
      plugin: "nav2_route::DistanceScorer"
      weight: 1.0

    operations: ["TimeMarker"]
    TimeMarker:
      plugin: "nav2_route::TimeMarker"
```

### 3.2 GeoJSON 路网文件

**文件路径**: `src/robot_route/maps/1126.geojson`

**结构说明：**
- `Point` 类型 Feature = 路网节点
- `MultiLineString` 类型 Feature = 路网边（带 startid/endid）

---

## 4. 路网导航执行

### ⭐ 实现 Nav2 Route Server Demo 效果

Nav2 官方 Demo 展示了机器人沿着预定义路网从节点 A 精确导航到节点 B 的效果。以下是在你的工作空间中实现相同效果的完整步骤：

**Demo 效果说明：**
- 机器人沿着路网边缘精确行走
- 到达节点时会调整朝向
- 支持动态重新规划
- 遇到障碍物会沿替代路径绕行

---

#### 步骤 1：确保 Route Server 已激活

```bash
# 启动 Route Server（含 lifecycle manager）
source install/setup.bash
ros2 launch robot_route route_demo.launch.py \
  params_file:=src/robot_route/config/route_server.yaml \
  use_sim_time:=false

# 检查 route_server 状态
ros2 lifecycle get /route_server
# 期望输出：active [3]
```

---

#### 步骤 2：创建路网导航脚本

创建一个 Python 脚本来执行路网导航：

**文件路径**: `src/robot_route/scripts/route_navigator.py`

```python
#!/usr/bin/env python3
"""
Nav2 Route Server 导航示例
实现 Demo 效果：从起始节点沿路网导航到目标节点
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputeAndTrackRoute
from nav2_msgs.msg import RouteNodes


class RouteNavigator(Node):
    def __init__(self):
        super().__init__('route_navigator')
        
        # 使用 Route Server 的 Action
        self.route_client = ActionClient(
            self, 
            ComputeAndTrackRoute, 
            'compute_and_track_route'
        )
        
        self.get_logger().info('等待 Route Server Action...')
        self.route_client.wait_for_server()
        self.get_logger().info('Route Server 已连接！')
    
    def navigate_by_node_ids(self, start_node_id: str, goal_node_id: str):
        """
        从起始节点导航到目标节点
        
        Args:
            start_node_id: 起始节点 ID（字符串）
            goal_node_id: 目标节点 ID（字符串）
        """
        goal = ComputeAndTrackRoute.Goal()
        
        # 设置起点和终点节点
        goal.start = RouteNodes()
        goal.start.node_ids = [start_node_id]
        
        goal.goal = RouteNodes()
        goal.goal.node_ids = [goal_node_id]
        
        self.get_logger().info(f'开始导航：节点 {start_node_id} → 节点 {goal_node_id}')
        
        # 发送目标，设置回调
        send_goal_future = self.route_client.send_goal_async(
            goal, 
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        """导航过程中的反馈"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'当前进度：距目标节点 {feedback.distance_to_goal:.2f}m'
        )
    
    def goal_response_callback(self, future):
        """目标接受/拒绝回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝')
            return
        
        self.get_logger().info('导航目标已接受，开始执行...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """导航完成回调"""
        result = future.result().result
        self.get_logger().info(f'导航完成！')


def main():
    rclpy.init()
    navigator = RouteNavigator()
    
    # 示例：从节点 0 导航到节点 5
    # 根据你的 1126.geojson 中的节点 ID 修改
    navigator.navigate_by_node_ids('0', '5')
    
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

#### 步骤 3：运行路网导航

```bash
# 1. 给脚本执行权限
chmod +x src/robot_route/scripts/route_navigator.py

# 2. 运行导航
python3 src/robot_route/scripts/route_navigator.py
```

---

#### 步骤 4：在 RViz 中观察效果

1. **路网可视化**：确保 `/route_graph` MarkerArray 已添加
2. **路径显示**：观察机器人沿着路网边缘移动
3. **到达节点**：机器人到达每个节点时会调整朝向

---

### 方案 A：通过 Route Server Action 执行

Route Server 提供以下服务和 Action：

| 接口 | 类型 | 用途 |
|------|------|------|
| `/compute_route` | Action | 计算两节点间路由（仅规划）|
| `/compute_and_track_route` | Action | 计算并跟踪路由（推荐）|
| `/follow_route` | Action | 跟踪已存在的路由 |

**示例调用：**
```bash
# 仅规划路由：节点 0 -> 节点 5
ros2 action send_goal /compute_route nav2_msgs/action/ComputeRoute \
"use_start: false
use_poses: false
start_id: 0
goal_id: 5"
```

---

### 方案 B：转换为 Waypoints 执行

将路网节点作为航点发送给 `waypoint_follower`。

**Python 示例代码：**

```python
#!/usr/bin/env python3
"""路网导航示例 - 将路网节点转换为航点执行"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
import json


class RouteNavigator(Node):
    def __init__(self):
        super().__init__('route_navigator')
        self.waypoint_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
    def load_route_nodes(self, geojson_path: str) -> list:
        """从 GeoJSON 加载路网节点"""
        with open(geojson_path, 'r') as f:
            data = json.load(f)
        
        nodes = []
        for feature in data['features']:
            if feature['geometry']['type'] == 'Point':
                coord = feature['geometry']['coordinates']
                node_id = feature['properties']['id']
                nodes.append({
                    'id': node_id,
                    'x': coord[0],
                    'y': coord[1]
                })
        return sorted(nodes, key=lambda n: n['id'])
    
    def navigate_route(self, node_ids: list, geojson_path: str):
        """按节点 ID 顺序导航"""
        all_nodes = self.load_route_nodes(geojson_path)
        node_map = {n['id']: n for n in all_nodes}
        
        # 构建航点序列
        waypoints = []
        for nid in node_ids:
            if nid in node_map:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = node_map[nid]['x']
                pose.pose.position.y = node_map[nid]['y']
                pose.pose.orientation.w = 1.0
                waypoints.append(pose)
        
        # 发送到 waypoint_follower
        self.waypoint_client.wait_for_server()
        goal = FollowWaypoints.Goal()
        goal.poses = waypoints
        
        self.get_logger().info(f'导航到 {len(waypoints)} 个节点...')
        self.waypoint_client.send_goal_async(goal)


def main():
    rclpy.init()
    navigator = RouteNavigator()
    
    geojson_path = 'src/robot_route/maps/1126.geojson'
    node_sequence = [0, 1, 2, 3, 4, 5, 6]  # 按此顺序导航
    
    navigator.navigate_route(node_sequence, geojson_path)
    rclpy.spin(navigator)


if __name__ == '__main__':
    main()
```

---

## 5. 与现有 Waypoint Editor 集成

你已有的 `waypoint_editor` RViz 插件可以与路网系统集成：

1. **加载路网节点为航点**：从 GeoJSON 读取节点坐标
2. **发送到 follow_waypoints**：使用现有的 Start Navigation 功能

---

## 6. 验证步骤

### 6.1 检查路网服务

```bash
# 检查 route_server 状态
ros2 lifecycle get /route_server

# 查看路网可视化话题
ros2 topic echo /route_graph --once
```

### 6.2 RViz 中查看路网

1. Add → By topic → `/route_graph` → MarkerArray
2. 设置 Durability Policy 为 **Transient Local**
3. 确保勾选 `edges`, `nodes`, `route_graph_ids`

### 6.3 测试路由计算

```bash
ros2 service call /route_server/compute_route nav2_msgs/srv/ComputeRoute \
    "{start: {node_id: '0'}, goal: {node_id: '5'}}"
```

---

---

## 7. 进阶：高级工业配置

为了满足更复杂的工业需求，建议启用以下 Route Server 高级插件：

### 7.1 自动重规划 (Rerouting)

当路网某条边被障碍物（如临时停放的货物）阻挡时，自动规划绕行路径。

**配置方法** (`route_server.yaml`):
```yaml
    operations: ["TimeMarker", "ReroutingService"]
    ReroutingService:
      plugin: "nav2_route::ReroutingService"
```

### 7.2 区域限速 (Speed Limits)

在 GeoJSON 中为特定边设置 `speed_limit` 属性（例如于人流密集区），机器人经过时会自动减速。

**配置方法** (`route_server.yaml`):
```yaml
    operations: ["TimeMarker", "ReroutingService", "AdjustSpeedLimit"]
    AdjustSpeedLimit:
      plugin: "nav2_route::AdjustSpeedLimit"
```

**QGIS 设置**: 在 Fields 中增加 `speed_limit` (Double) 字段。

---

## 8. 导航模式选择指南

针对你的需求 **"直到了路径点旋转到下一段的前进方向，优先旋转到位了再前进"**，有两种模式可选：

| 特性 | 方案 A: 连续跟踪 (Action) | 方案 B: 航点模式 (Waypoints) |
|------|---------------------------|------------------------------|
| **核心机制** | 发送完整路径给控制器 | 将每个路网节点作为独立目标发送 |
| **旋转行为** | **仅在起点旋转**，中间平滑过渡 | **在每个节点处停车并旋转** |
| **流畅度** | ⭐⭐⭐⭐⭐ (不停顿) | ⭐⭐ (每站必停) |
| **精确度** | 略有切角 (取决于控制参数) | ⭐⭐⭐⭐⭐ (点对点直角转弯) |
| **适用场景** | 宽敞主干道、追求效率 | **狭窄通道、从路口转弯、如果你要求"必须先转正再走"** |

### 🎯 推荐方案

根据你的描述，**方案 B (Waypoints 模式)** 结合我们之前配置的 **`RotationShimController`** 是最完美的组合：

1.  脚本将路网节点序列转换为一系列 Waypoints。
2.  Nav2 接收到第一个点 -> `RotationShim` 触发 -> **原地旋转对准** -> 直行到达。
3.  到达第一个点 -> 切换到第二个点 -> `RotationShim` 再次触发 -> **原地旋转对准下一段** -> 直行。

这完美复现了工业 AGV 的经典运行模式。

---

## 9. 参考资料

- [Nav2 Route Server 文档](https://docs.nav2.org/configuration/packages/configuring-route-server.html)
- [Nav2 Route Server Tools](https://docs.nav2.org/tutorials/docs/route_server_tools.html)
- 路网绘制指南：`src/robot_route/doc/route_drew.md`
