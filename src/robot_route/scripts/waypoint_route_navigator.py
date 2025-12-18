#!/usr/bin/env python3
"""
⚠️ [DEPRECATED] 此脚本已被官方行为树方案替代

推荐使用：
- 行为树: navigate_on_route_graph_arch1_jazzy.xml
- 启动方式: ros2 launch robot_navigation navigation.launch.py
- 导航方式: 在RViz点击 "2D Goal Pose"

官方方案优势：
✅ 自动密集路径生成（Route Server内置）
✅ 自动恢复机制
✅ 动态重规划（0.5Hz）
✅ Route Operations插件支持

保留原因：作为参考实现和备份方案

---
原始说明：
Nav2 Route Server - Waypoints模式导航
实现效果: 机器人到达每个路网节点时停车,旋转对齐下一段方向,然后前进
适用场景: 狭窄通道、需要精确转向的工业AGV导航
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
import json
import sys


class WaypointRouteNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_route_navigator')
        
        # 声明参数
        self.declare_parameter('geojson_path', 
            '/home/suja/voxel_ws/src/robot_route/maps/1126.geojson')
        self.declare_parameter('default_frame', 'map')
        self.declare_parameter('edge_density', 0.3)  # 边的密集化间距(m)
        
        # 创建waypoint_follower客户端
        self.waypoint_client = ActionClient(
            self, 
            FollowWaypoints, 
            'follow_waypoints'
        )
        
        self.get_logger().info('等待 Waypoint Follower...')
        self.waypoint_client.wait_for_server()
        self.get_logger().info('Waypoint Follower 已连接!')
    
    def load_route_nodes(self, geojson_path: str) -> dict:
        """从GeoJSON加载路网节点
        
        Args:
            geojson_path: GeoJSON文件路径
            
        Returns:
            {node_id: {'x': float, 'y': float}, ...}
        """
        try:
            with open(geojson_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except FileNotFoundError:
            self.get_logger().error(f'找不到文件: {geojson_path}')
            self.get_logger().error('请检查路径是否正确')
            return {}
        except json.JSONDecodeError as e:
            self.get_logger().error(f'GeoJSON格式错误: {e}')
            return {}
        except Exception as e:
            self.get_logger().error(f'加载路网失败: {e}')
            return {}
        
        # 验证GeoJSON结构
        if 'features' not in data:
            self.get_logger().error('GeoJSON缺少features字段')
            return {}
        
        nodes = {}
        for feature in data.get('features', []):
            # 安全访问geometry和properties
            geom = feature.get('geometry')
            props = feature.get('properties')
            
            if geom and geom.get('type') == 'Point' and props:
                coord = geom.get('coordinates')
                node_id = props.get('id')
                
                if coord and len(coord) >= 2 and node_id is not None:
                    node_id = str(node_id)
                    nodes[node_id] = {
                        'x': float(coord[0]),
                        'y': float(coord[1])
                    }
                    self.get_logger().info(f'加载节点 {node_id}: ({coord[0]:.2f}, {coord[1]:.2f})')
        
        self.get_logger().info(f'成功加载 {len(nodes)} 个路网节点')
        return nodes
    
    def densify_edge(self, x1, y1, x2, y2, density=0.3):
        """在两点间生成密集航点,实现严格路网跟随
        
        Args:
            x1, y1: 起点坐标
            x2, y2: 终点坐标
            density: 点间距(m)
            
        Returns:
            List[PoseStamped]: 密集航点列表
        """
        import math
        
        dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        if dist < 0.01:  # 距离太小,直接返回终点
            return []
        
        # 计算需要多少个中间点
        num_points = max(1, int(dist / density))
        
        waypoints = []
        frame = self.get_parameter('default_frame').value
        
        # 生成中间点(不包括起点,包括终点)
        for i in range(1, num_points + 1):
            t = i / num_points
            
            pose = PoseStamped()
            pose.header.frame_id = frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x1 + t * (x2 - x1)
            pose.pose.position.y = y1 + t * (y2 - y1)
            pose.pose.position.z = 0.0
            
            # 计算朝向(指向下一个点)
            angle = math.atan2(y2 - y1, x2 - x1)
            pose.pose.orientation.z = math.sin(angle / 2.0)
            pose.pose.orientation.w = math.cos(angle / 2.0)
            
            waypoints.append(pose)
        
        return waypoints
    
    def navigate_route(self, node_ids: list, geojson_path: str):
        """按节点ID顺序导航(严格路网模式)
        
        生成沿路网边的密集航点,确保机器人严格沿边行走
        """
        # 加载所有节点
        node_map = self.load_route_nodes(geojson_path)
        if not node_map:
            self.get_logger().error('无法加载路网节点')
            return
        
        # 获取密集化参数
        edge_density = self.get_parameter('edge_density').value
        
        # 构建严格路网航点序列
        waypoints = []
        frame = self.get_parameter('default_frame').value
        
        self.get_logger().info('=== 严格路网导航模式 ===')
        self.get_logger().info(f'边密集化间距: {edge_density}m')
        
        for i in range(len(node_ids) - 1):
            start_id = str(node_ids[i])
            end_id = str(node_ids[i + 1])
            
            if start_id not in node_map:
                self.get_logger().warn(f'节点 {start_id} 不在路网中,跳过')
                continue
            if end_id not in node_map:
                self.get_logger().warn(f'节点 {end_id} 不在路网中,跳过')
                continue
            
            start = node_map[start_id]
            end = node_map[end_id]
            
            # 添加起始节点(仅第一条边)
            if i == 0:
                start_pose = PoseStamped()
                start_pose.header.frame_id = frame
                start_pose.header.stamp = self.get_clock().now().to_msg()
                start_pose.pose.position.x = start['x']
                start_pose.pose.position.y = start['y']
                start_pose.pose.position.z = 0.0
                start_pose.pose.orientation.w = 1.0
                waypoints.append(start_pose)
                self.get_logger().info(f'起点: 节点{start_id}')
            
            # 在边上生成密集点
            edge_waypoints = self.densify_edge(
                start['x'], start['y'],
                end['x'], end['y'],
                density=edge_density
            )
            
            waypoints.extend(edge_waypoints)
            
            import math
            dist = math.sqrt((end['x']-start['x'])**2 + (end['y']-start['y'])**2)
            self.get_logger().info(
                f'边 {start_id}→{end_id}: 距离{dist:.2f}m, '
                f'生成{len(edge_waypoints)}个密集点'
            )
        
        if not waypoints:
            self.get_logger().error('没有有效的航点')
            return
        
        # 发送到waypoint_follower
        goal = FollowWaypoints.Goal()
        goal.poses = waypoints
        
        self.get_logger().info(f'开始导航: {len(waypoints)} 个航点')
        self.get_logger().info('机器人将在每个节点停车并旋转到下一段方向')
        
        # 发送目标
        send_future = self.waypoint_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        send_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        """导航反馈"""
        feedback = feedback_msg.feedback
        # 修复: FollowWaypoints.Feedback只有current_waypoint字段
        self.get_logger().info(
            f'正在导航到航点 {feedback.current_waypoint + 1}'
        )
    
    def goal_response_callback(self, future):
        """目标响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('航点导航被拒绝')
            return
        
        self.get_logger().info('航点导航已接受,开始执行...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        """导航完成回调"""
        result = future.result().result
        missed = len(result.missed_waypoints)
        
        if missed == 0:
            self.get_logger().info('✅ 路网导航完成!所有节点已到达')
        else:
            self.get_logger().warn(f'⚠️ 导航完成,但有 {missed} 个节点未到达')


def main():
    rclpy.init()
    navigator = WaypointRouteNavigator()
    
    # 从参数获取路网文件路径
    geojson_path = navigator.get_parameter('geojson_path').value
    
    # 示例: 从节点0导航到节点5
    # 修改这个列表来定义你的导航路线
    node_sequence = [0, 1, 2, 3, 4, 5]
    
    # 如果从命令行传入节点ID
    if len(sys.argv) > 1:
        try:
            node_sequence = [int(x) for x in sys.argv[1:]]
            navigator.get_logger().info(f'使用命令行参数: {node_sequence}')
        except ValueError:
            navigator.get_logger().error('命令行参数必须是整数节点ID')
            return
    
    # 开始导航
    navigator.navigate_route(node_sequence, geojson_path)
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('导航被用户中断')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
