#!/usr/bin/env python3
"""
Trajectory Recorder Node
Records robot's actual trajectory for comparison with planned path.

Features:
- TF-based pose sampling (default, most accurate)
- Subscribes to odom/amcl_pose as fallback
- Publishes Path and Marker for RViz visualization
- Services for start/stop/save/clear operations
- Saves trajectory in TUM format for evo analysis

Usage:
    ros2 run robot_route trajectory_recorder.py
    
Services:
    /trajectory/start - Start recording
    /trajectory/stop  - Stop recording  
    /trajectory/save  - Save to file
    /trajectory/clear - Clear recorded data
"""

import os
from datetime import datetime
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from std_msgs.msg import ColorRGBA

# TF2 imports
from tf2_ros import Buffer, TransformListener, TransformException


class TrajectoryRecorder(Node):
    """Records and visualizes robot trajectory."""
    
    def __init__(self):
        super().__init__('trajectory_recorder')
        
        # Parameters
        # P0-2 Fix: 默认使用 TF 采样，确保坐标系正确
        self.declare_parameter('pose_source', 'tf')  # 'tf' (recommended), 'odom', or 'amcl_pose'
        self.declare_parameter('pose_topic', '/odom')  # Used when pose_source != 'tf'
        self.declare_parameter('pose_type', 'Odometry')  # Odometry or PoseWithCovarianceStamped
        self.declare_parameter('min_distance', 0.05)  # Minimum distance between recorded points (m)
        self.declare_parameter('output_dir', os.path.expanduser('~/trajectory_data'))
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('tf_sample_rate', 10.0)  # Hz for TF sampling
        # self.declare_parameter('use_sim_time', True)
        
        self.pose_source = self.get_parameter('pose_source').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.pose_type = self.get_parameter('pose_type').value
        self.min_distance = self.get_parameter('min_distance').value
        self.output_dir = self.get_parameter('output_dir').value
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.tf_sample_rate = self.get_parameter('tf_sample_rate').value
        
        # State
        self.is_recording = False
        self.trajectory: List[PoseStamped] = []
        self.last_pose: Optional[PoseStamped] = None
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Publishers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.path_pub = self.create_publisher(Path, '/actual_trajectory', qos)
        self.marker_pub = self.create_publisher(Marker, '/actual_trajectory_marker', 10)
        
        # TF setup (for pose_source='tf')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Setup based on pose_source
        if self.pose_source == 'tf':
            # P0-2 Fix: TF 采样模式 - 直接从 TF 获取 map->base_link 变换
            self.create_timer(1.0 / self.tf_sample_rate, self.tf_sample_callback)
            self.get_logger().info(
                f'Pose source: TF ({self.global_frame} -> {self.robot_frame}) @ {self.tf_sample_rate} Hz')
        elif self.pose_source == 'amcl_pose':
            self.pose_sub = self.create_subscription(
                PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
            self.get_logger().info('Pose source: /amcl_pose (already in map frame)')
        else:
            # Legacy odom mode (not recommended for trajectory evaluation)
            if self.pose_type == 'Odometry':
                self.pose_sub = self.create_subscription(
                    Odometry, self.pose_topic, self.odom_callback, 10)
            else:
                self.pose_sub = self.create_subscription(
                    PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, 10)
            self.get_logger().warn(
                f'Pose source: {self.pose_topic} - WARNING: May have coordinate frame issues!')
        
        # Services
        self.start_srv = self.create_service(
            Trigger, '/trajectory/start', self.handle_start)
        self.stop_srv = self.create_service(
            Trigger, '/trajectory/stop', self.handle_stop)
        self.save_srv = self.create_service(
            Trigger, '/trajectory/save', self.handle_save)
        self.clear_srv = self.create_service(
            Trigger, '/trajectory/clear', self.handle_clear)
        
        # Timer for periodic publishing
        self.create_timer(0.5, self.publish_visualization)
        
        self.get_logger().info(f'Output directory: {self.output_dir}')
        self.get_logger().info('Call /trajectory/start to begin recording')
    
    def tf_sample_callback(self):
        """P0-2 Fix: Sample pose from TF (map->base_link) - most accurate method."""
        if not self.is_recording:
            return
        
        try:
            # Get latest transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time())
            
            pose = PoseStamped()
            pose.header.stamp = transform.header.stamp
            pose.header.frame_id = self.global_frame
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            
            self.record_pose(pose)
        except TransformException as e:
            self.get_logger().warn(
                f'TF lookup failed: {e}', throttle_duration_sec=5.0)
    
    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """Handle AMCL pose messages (already in map frame)."""
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.record_pose(pose)
    
    def odom_callback(self, msg: Odometry):
        """Handle Odometry messages (legacy mode - may have frame issues)."""
        pose = PoseStamped()
        pose.header = msg.header
        # WARNING: This may cause coordinate frame issues if odom != map
        pose.header.frame_id = self.global_frame
        pose.pose = msg.pose.pose
        self.record_pose(pose)
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Handle PoseWithCovarianceStamped messages (legacy mode)."""
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = self.global_frame
        pose.pose = msg.pose.pose
        self.record_pose(pose)
    
    def record_pose(self, pose: PoseStamped):
        """Record a pose if recording is enabled and distance threshold is met."""
        if not self.is_recording:
            return
        
        # Check distance threshold
        if self.last_pose is not None:
            dx = pose.pose.position.x - self.last_pose.pose.position.x
            dy = pose.pose.position.y - self.last_pose.pose.position.y
            dist = (dx**2 + dy**2) ** 0.5
            if dist < self.min_distance:
                return
        
        self.trajectory.append(pose)
        self.last_pose = pose
    
    def publish_visualization(self):
        """Publish trajectory as Path and Marker for RViz."""
        if not self.trajectory:
            return
        
        # Publish Path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.global_frame
        path_msg.poses = self.trajectory
        self.path_pub.publish(path_msg)
        
        # Publish Marker (line strip)
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.global_frame
        marker.ns = 'actual_trajectory'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Line width
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
        marker.pose.orientation.w = 1.0
        
        for pose in self.trajectory:
            marker.points.append(pose.pose.position)
        
        self.marker_pub.publish(marker)
    
    def handle_start(self, request, response):
        """Start recording trajectory."""
        self.is_recording = True
        self.get_logger().info('Started recording trajectory')
        response.success = True
        response.message = f'Recording started. Current points: {len(self.trajectory)}'
        return response
    
    def handle_stop(self, request, response):
        """Stop recording trajectory."""
        self.is_recording = False
        self.get_logger().info(f'Stopped recording. Total points: {len(self.trajectory)}')
        response.success = True
        response.message = f'Recording stopped. Total points: {len(self.trajectory)}'
        return response
    
    def handle_save(self, request, response):
        """Save trajectory to TUM format file."""
        if not self.trajectory:
            response.success = False
            response.message = 'No trajectory data to save'
            return response
        
        # Generate filename with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.output_dir, f'actual_trajectory_{timestamp}.tum')
        
        try:
            with open(filename, 'w') as f:
                for pose in self.trajectory:
                    t = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9
                    p = pose.pose.position
                    q = pose.pose.orientation
                    f.write(f'{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} '
                           f'{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n')
            
            self.get_logger().info(f'Saved trajectory to {filename}')
            response.success = True
            response.message = f'Saved {len(self.trajectory)} points to {filename}'
        except Exception as e:
            response.success = False
            response.message = f'Failed to save: {str(e)}'
        
        return response
    
    def handle_clear(self, request, response):
        """Clear recorded trajectory."""
        count = len(self.trajectory)
        self.trajectory.clear()
        self.last_pose = None
        self.get_logger().info('Cleared trajectory data')
        response.success = True
        response.message = f'Cleared {count} points'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
