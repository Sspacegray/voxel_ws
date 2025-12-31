#!/usr/bin/env python3
"""
路网目标点导航桥接脚本
RViz Goal Pose → Route Server (First-Mile + Route + Last-Mile)

使用方法:
    ros2 run robot_route route_goal_bridge.py

然后在 RViz 中使用 "2D Goal Pose" 点击目标，机器人将沿路网导航
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose, ComputeRoute, FollowPath
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped
import tf2_ros

import math
import time


class RouteGoalBridge(Node):
    def __init__(self):
        super().__init__('route_goal_bridge')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # TF for getting current pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Action clients
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.callback_group
        )
        self.compute_route_client = ActionClient(
            self, ComputeRoute, 'compute_route',
            callback_group=self.callback_group
        )
        self.follow_path_client = ActionClient(
            self, FollowPath, 'follow_path',
            callback_group=self.callback_group
        )
        
        # Subscribe to goal pose from RViz
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Parameters
        self.declare_parameter('first_mile_threshold', 0.5)  # m
        self.declare_parameter('last_mile_threshold', 0.5)   # m
        self.declare_parameter('fallback_to_nav', False)      # 默认禁用回退，强制走路网
        
        self.is_navigating = False
        
        self.get_logger().info('========================================')
        self.get_logger().info('路网目标点导航桥接已启动')
        self.get_logger().info('在 RViz 中使用 "2D Goal Pose" 点击目标')
        self.get_logger().info('========================================')
        
        # Wait for action servers
        self._wait_for_servers()
    
    def _wait_for_servers(self):
        """等待所有 Action Server"""
        self.get_logger().info('等待 Action Servers...')
        
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('NavigateToPose Action 不可用')
        else:
            self.get_logger().info('✓ NavigateToPose 已连接')
            
        if not self.compute_route_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('ComputeRoute Action 不可用')
        else:
            self.get_logger().info('✓ ComputeRoute 已连接')
            
        if not self.follow_path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('FollowPath Action 不可用')
        else:
            self.get_logger().info('✓ FollowPath 已连接')
    
    def get_current_pose(self) -> PoseStamped:
        """获取机器人当前位姿"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            
            return pose
        except Exception as e:
            self.get_logger().error(f'无法获取当前位姿: {e}')
            return None
    
    def goal_callback(self, goal_pose: PoseStamped):
        """收到目标点时的回调"""
        if self.is_navigating:
            self.get_logger().warn('正在导航中，忽略新目标')
            return
        
        self.is_navigating = True
        self.get_logger().info(f'收到目标: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})')
        
        # 获取当前位姿
        current_pose = self.get_current_pose()
        if current_pose is None:
            self.get_logger().error('无法获取当前位姿，取消导航')
            self.is_navigating = False
            return
        
        # 尝试路网导航
        self.navigate_via_route(current_pose, goal_pose)
    
    def navigate_via_route(self, start_pose: PoseStamped, goal_pose: PoseStamped):
        """通过路网导航"""
        self.get_logger().info('步骤 1: 计算路网路径...')
        
        # 1. 计算路网路径
        route_goal = ComputeRoute.Goal()
        route_goal.start = start_pose
        route_goal.goal = goal_pose
        route_goal.goal = goal_pose
        route_goal.use_poses = True
        
        future = self.compute_route_client.send_goal_async(route_goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('路网规划请求被拒绝，回退到普通导航')
            self.fallback_navigation(goal_pose)
            return
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        
        result = result_future.result()
        if result is None or result.result is None:
            self.get_logger().warn('路网规划失败，回退到普通导航')
            self.fallback_navigation(goal_pose)
            return
        
        route_path = result.result.path
        if len(route_path.poses) < 2:
            self.get_logger().warn('路网路径太短，回退到普通导航')
            self.fallback_navigation(goal_pose)
            return
        
        self.get_logger().info(f'路网路径包含 {len(route_path.poses)} 个点')
        
        # 2. First-Mile: 导航到路网起点
        route_start = route_path.poses[0]
        first_mile_threshold = self.get_parameter('first_mile_threshold').value
        
        dist_to_route_start = self._distance(start_pose.pose, route_start.pose)
        if dist_to_route_start > first_mile_threshold:
            self.get_logger().info(f'步骤 2: First-Mile 导航 ({dist_to_route_start:.2f}m)...')
            start_goal = PoseStamped()
            start_goal.header = route_path.header
            start_goal.pose = route_start.pose
            
            if not self._navigate_to_pose(start_goal):
                self.get_logger().error('First-Mile 导航失败')
                self.is_navigating = False
                return
        else:
            self.get_logger().info('已在路网起点附近，跳过 First-Mile')
        
        # 3. 沿路网导航
        self.get_logger().info('步骤 3: 沿路网导航...')
        if not self._follow_path(route_path):
            self.get_logger().error('路网导航失败')
            self.is_navigating = False
            return
        
        # 4. Last-Mile: 导航到最终目标
        route_end = route_path.poses[-1]
        last_mile_threshold = self.get_parameter('last_mile_threshold').value
        
        dist_to_goal = self._distance(route_end.pose, goal_pose.pose)
        if dist_to_goal > last_mile_threshold:
            self.get_logger().info(f'步骤 4: Last-Mile 导航 ({dist_to_goal:.2f}m)...')
            if not self._navigate_to_pose(goal_pose):
                self.get_logger().error('Last-Mile 导航失败')
                self.is_navigating = False
                return
        else:
            self.get_logger().info('已在目标附近，跳过 Last-Mile')
        
        self.get_logger().info('========================================')
        self.get_logger().info('导航完成！')
        self.get_logger().info('========================================')
        self.is_navigating = False
    
    def _navigate_to_pose(self, goal_pose: PoseStamped) -> bool:
        """使用 Nav2 NavigateToPose 导航"""
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        
        future = self.nav_to_pose_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        
        return result_future.result() is not None
    
    def _follow_path(self, path: Path) -> bool:
        """跟踪路径"""
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = ''  # 使用默认控制器
        
        future = self.follow_path_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=300.0)
        
        return result_future.result() is not None
    
    def _distance(self, pose1, pose2) -> float:
        """计算两点距离"""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def fallback_navigation(self, goal_pose: PoseStamped):
        """回退到普通 Nav2 导航"""
        if not self.get_parameter('fallback_to_nav').value:
            self.get_logger().error('路网导航失败，未启用回退')
            self.is_navigating = False
            return
        
        self.get_logger().info('回退到普通 Nav2 导航...')
        if self._navigate_to_pose(goal_pose):
            self.get_logger().info('导航完成！')
        else:
            self.get_logger().error('导航失败')
        
        self.is_navigating = False


def main():
    rclpy.init()
    node = RouteGoalBridge()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
