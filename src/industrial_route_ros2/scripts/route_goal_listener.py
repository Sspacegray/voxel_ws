#!/usr/bin/env python3
"""
route_goal_listener.py - 监听 RViz 目标点并通过工业路网导航

功能：
1. 监听 /goal_pose 话题（RViz 2D Goal Pose）
2. 调用 industrial_route_server 规划路径（包含进入/退出路网）
3. 将完整路径（approach + dense + departure）发送给 Nav2 执行

用法：
    ros2 run industrial_route_ros2 route_goal_listener.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from industrial_route_interfaces.srv import PlanRoute


class RouteGoalListener(Node):
    def __init__(self):
        super().__init__('route_goal_listener')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # 获取当前机器人位置的 TF
        self.current_pose = None
        
        # 订阅 RViz 目标点
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 订阅 AMCL pose 获取当前位置
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/amcl_pose',
            self.pose_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 也尝试订阅带协方差的 pose
        from geometry_msgs.msg import PoseWithCovarianceStamped
        self.pose_cov_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_cov_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 规划服务客户端
        self.plan_client = self.create_client(
            PlanRoute,
            '/industrial_route_server/plan_route',
            callback_group=self.callback_group
        )
        
        # Nav2 FollowPath action 客户端
        self.follow_path_client = ActionClient(
            self,
            FollowPath,
            '/follow_path',
            callback_group=self.callback_group
        )
        
        # 路径发布者（用于可视化）
        self.path_pub = self.create_publisher(Path, '/route_planned_path', 10)
        
        self.get_logger().info('Route Goal Listener started!')
        self.get_logger().info('Use "2D Goal Pose" in RViz to navigate through route network')
        self.get_logger().info('Waiting for robot pose...')
        
    def pose_callback(self, msg: PoseStamped):
        """更新当前位置"""
        self.current_pose = msg
        
    def pose_cov_callback(self, msg):
        """更新当前位置（带协方差）"""
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.current_pose = pose_stamped
        
    def goal_callback(self, msg: PoseStamped):
        """处理 RViz 目标点"""
        self.get_logger().info(
            f'Received goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )
        
        # 检查是否有当前位置
        if self.current_pose is None:
            self.get_logger().warn('No robot pose available yet, waiting...')
            return
        
        # 等待规划服务
        if not self.plan_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Plan route service not available!')
            return
            
        # 调用规划服务 - 使用当前位置作为起点，目标点作为终点
        request = PlanRoute.Request()
        request.start = self.current_pose
        request.goal = msg
        request.use_start_pose = True
        request.use_goal_pose = True
        
        self.get_logger().info(
            f'Planning route: ({self.current_pose.pose.position.x:.2f}, {self.current_pose.pose.position.y:.2f}) -> '
            f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )
        future = self.plan_client.call_async(request)
        future.add_done_callback(self.plan_response_callback)
        
    def plan_response_callback(self, future):
        """处理规划响应"""
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Plan service call failed: {e}')
            return
            
        if not response.success:
            self.get_logger().warn(f'Planning failed: {response.error_msg}')
            self.get_logger().info('Route network might not cover this area')
            return
        
        # 合并完整路径：approach_path + dense_path + departure_path
        full_path = Path()
        full_path.header.frame_id = 'map'
        full_path.header.stamp = self.get_clock().now().to_msg()
        
        # 1. 进入路网路径 (approach)
        approach_count = len(response.approach_path.poses)
        if approach_count > 0:
            self.get_logger().info(f'Approach path: {approach_count} poses')
            full_path.poses.extend(response.approach_path.poses)
        
        # 2. 路网主路径 (dense)
        dense_count = len(response.dense_path.poses)
        if dense_count > 0:
            self.get_logger().info(f'Dense path: {dense_count} poses')
            full_path.poses.extend(response.dense_path.poses)
        
        # 3. 退出路网路径 (departure)
        departure_count = len(response.departure_path.poses)
        if departure_count > 0:
            self.get_logger().info(f'Departure path: {departure_count} poses')
            full_path.poses.extend(response.departure_path.poses)
            
        total_poses = len(full_path.poses)
        if total_poses == 0:
            self.get_logger().warn('Empty path returned')
            return
            
        self.get_logger().info(
            f'Full route planned: {total_poses} poses '
            f'(approach: {approach_count}, route: {dense_count}, departure: {departure_count})'
        )
        
        # 发布路径可视化
        self.path_pub.publish(full_path)
        
        # 发送给 Nav2 执行
        self.send_path_to_nav2(full_path)
        
    def send_path_to_nav2(self, path: Path):
        """发送路径给 Nav2 FollowPath"""
        if not self.follow_path_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('FollowPath action server not available!')
            return
            
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = 'FollowPath'
        
        self.get_logger().info('Sending path to Nav2 controller...')
        
        send_goal_future = self.follow_path_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        """处理 action goal 响应"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            return
            
        self.get_logger().info('Goal accepted, robot is following route network path...')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
        
    def result_callback(self, future):
        """处理导航结果"""
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Route navigation succeeded!')
        else:
            self.get_logger().warn(f'Navigation finished with status: {result.status}')
            
    def feedback_callback(self, feedback_msg):
        """处理导航反馈"""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = RouteGoalListener()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
