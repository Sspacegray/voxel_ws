严格路网导航实现分析
当前方案分析 ⚠️
waypoint_route_navigator.py做了什么?
1. 读取GeoJSON → 获取节点坐标
2. 节点 → PoseStamped列表 [节点0, 节点1, 节点2, ...]
3. 发送给waypoint_follower
waypoint_follower会怎么做?
对每个航点:
  当前位置 → [自由空间规划] → 航点
  
例如:
  节点0 → [Planner计算路径] → 节点1
  节点1 → [Planner计算路径] → 节点2
❌ 问题: 不是严格路网!
waypoint_follower使用自由空间规划,路径可能不沿着预定义的路网边!

预期路网 (严格):           实际路径 (waypoint_follower):
节点0 ────→ 节点1          节点0 ╱    节点1
  │                           │  ╲  ╱  
  │                           │   ✗   (走捷径!)
  ↓                           ↓  ╱  ╲
节点2                        节点2
实现真正严格路网的3种方案
方案1: Route Server Dense Path (最标准) ⭐⭐⭐⭐⭐
原理: Route Server输出沿路网边的密集点

# 使用Route Server的ComputeRoute
route_result = route_server.compute_route(start_node, goal_node)
dense_path = route_result.path  # 沿路网边每5cm一个点
# 发送给FollowPath (而非waypoint_follower)
follow_path_client.send_goal(dense_path)
架构:

Route Server
  ↓ (沿边生成密集路径)
  ↓ (path_density: 0.05m)
dense_path (nav_msgs/Path)
  ↓ (发送给FollowPath)
Controller直接跟踪
  ↓
严格沿路网边行走 ✅
优点:

✅ 真正严格路网
✅ 自动处理转角平滑
✅ 支持速度限制
✅ 支持重规划
需要:

Route Server启动
使用ComputeRoute或ComputeAndTrackRoute Action
方案2: 密集化Waypoints (折中方案) ⭐⭐⭐⭐
原理: 在路网边上生成大量密集的waypoints

def densify_edge(start_node, end_node, density=0.2):
    """在边上生成密集点"""
    waypoints = []
    dist = distance(start_node, end_node)
    num_points = int(dist / density)
    
    for i in range(num_points + 1):
        t = i / num_points
        x = start_node.x + t * (end_node.x - start_node.x)
        y = start_node.y + t * (end_node.y - start_node.y)
        waypoints.append(PoseStamped(x, y))
    
    return waypoints
# 使用
edge_waypoints = []
for i in range(len(nodes) - 1):
    edge_waypoints += densify_edge(nodes[i], nodes[i+1], density=0.2)
waypoint_follower.send_goal(edge_waypoints)
效果:

原来:
节点0 ────────────→ 节点1
(自由规划可能走捷径)
密集化后:
节点0 → wp1 → wp2 → wp3 → ... → 节点1
(强制沿边,每0.2m一个点)
优点:

✅ 不需要Route Server
✅ 较简单实现
✅ 基本沿路网边
缺点:

⚠️ 转角可能不平滑
⚠️ 密集点太多性能下降
⚠️ 仍有小偏离
方案3: 纯路径跟踪 (最精确) ⭐⭐⭐⭐⭐
原理: 直接使用FollowPath跟踪路网路径

# 从GeoJSON读取边的完整坐标
def load_route_path(geojson_path, node_sequence):
    # 读取边的coordinates (已包含所有点)
    path = Path()
    
    for i in range(len(node_sequence) - 1):
        edge = find_edge(node_sequence[i], node_sequence[i+1])
        for coord in edge.geometry.coordinates:
            pose = PoseStamped()
            pose.pose.position.x = coord[0]
            pose.pose.position.y = coord[1]
            path.poses.append(pose)
    
    return path
# 使用FollowPath
path = load_route_path(geojson, [0, 1, 2, 3])
follow_path_client.send_goal(path)
架构:

GeoJSON边的coordinates
  ↓ (已有路径点)
Path (nav_msgs/Path)
  ↓ (发送给FollowPath)
Controller直接跟踪
  ↓
严格沿路网边 ✅
优点:

✅ 最精确
✅ 不需要Route Server
✅ 利用GeoJSON已有的边坐标
缺点:

⚠️ 需要修改waypoint_route_navigator.py
⚠️ 不会在节点停车旋转
你的需求匹配度分析
你的需求:

✅ 严格路网 (沿边行走)
✅ 到点停车旋转
✅ 狭窄通道
推荐组合方案: 方案2改进版 ⭐⭐⭐⭐⭐
特点: 密集化边 + 在节点处停车

def generate_route_waypoints(node_sequence, geojson_path):
    waypoints = []
    
    for i in range(len(node_sequence) - 1):
        current = node_sequence[i]
        next_node = node_sequence[i + 1]
        
        # 在边上密集采样
        edge_points = densify_edge(current, next_node, density=0.3)
        waypoints += edge_points
        
        # 添加节点自身(确保到达并旋转)
        waypoints.append(create_pose(next_node))
    
    return waypoints
效果:

节点0 → p1 → p2 → p3 → 节点1(停车旋转)
                          ↓
                        节点1 → p1 → p2 → 节点2(停车旋转)
实现建议
立即可用方案: 修改waypoint_route_navigator.py
在
navigate_route()
中添加边密集化:

def navigate_route(self, node_ids: list, geojson_path: str):
    node_map = self.load_route_nodes(geojson_path)
    
    waypoints = []
    for i in range(len(node_ids) - 1):
        start_id = str(node_ids[i])
        end_id = str(node_ids[i + 1])
        
        if start_id in node_map and end_id in node_map:
            start = node_map[start_id]
            end = node_map[end_id]
            
            # 密集化这条边
            edge_waypoints = self.densify_edge(
                start['x'], start['y'],
                end['x'], end['y'],
                density=0.3  # 每30cm一个点
            )
            waypoints += edge_waypoints
    
    # 发送给waypoint_follower
    goal = FollowWaypoints.Goal()
    goal.poses = waypoints
    self.waypoint_client.send_goal_async(goal)
def densify_edge(self, x1, y1, x2, y2, density=0.3):
    """在两点间生成密集航点"""
    import math
    
    dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    num_points = max(2, int(dist / density))
    
    waypoints = []
    for i in range(num_points + 1):
        t = i / num_points
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x1 + t * (x2 - x1)
        pose.pose.position.y = y1 + t * (y2 - y1)
        pose.pose.orientation.w = 1.0
        
        waypoints.append(pose)
    
    return waypoints
对比总结
方案	严格程度	到点旋转	实现难度	推荐
当前waypoint	⚠️ 50%	✅	简单	⭐⭐
Route Server	✅ 95%	⚠️ 看配置	中等	⭐⭐⭐⭐
密集化waypoint	✅ 85%	✅	简单	⭐⭐⭐⭐⭐
纯路径跟踪	✅ 98%	❌	中等	⭐⭐⭐
回答你的问题
Q: 理论上就可以实现严格路网了吗?

A:

❌ 当前方案(仅节点waypoint): 不能严格路网
✅ 修改后(密集化边waypoint): 可以实现85%严格度
✅ Route Server方案: 可以实现95%严格度
最简单的修复: 在waypoint_route_navigator.py中添加densify_edge()函数,在每条边上生成密集点。

要我帮你实现吗?

