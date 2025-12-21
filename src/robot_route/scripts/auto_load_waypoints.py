#!/usr/bin/env python3
"""
Auto Load Waypoints Script
Automatically loads a predefined waypoint file on startup.

Usage:
    ros2 run robot_route auto_load_waypoints.py --ros-args -p waypoint_file:=/path/to/waypoints.csv
"""

import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class AutoLoadWaypoints(Node):
    """Automatically load waypoints from a file and publish as path."""
    
    def __init__(self):
        super().__init__('auto_load_waypoints')
        
        # Parameters
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('auto_publish', True)  # Auto publish path after loading
        
        self.waypoint_file = self.get_parameter('waypoint_file').value
        self.auto_publish = self.get_parameter('auto_publish').value
        
        if not self.waypoint_file:
            self.get_logger().error('No waypoint_file specified! Use: --ros-args -p waypoint_file:=/path/to/file')
            return
        
        if not os.path.exists(self.waypoint_file):
            self.get_logger().error(f'Waypoint file not found: {self.waypoint_file}')
            return
        
        # Publishers
        self.path_pub = self.create_publisher(
            Path, '/waypoint_path', 
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1
            )
        )
        
        # Load and publish after a short delay to ensure RViz is ready
        self.create_timer(2.0, self.load_and_publish)
        self.get_logger().info(f'Will load waypoints from: {self.waypoint_file}')
    
    def load_and_publish(self):
        """Load waypoints from file and publish as Path."""
        try:
            waypoints = self.load_waypoints_from_file(self.waypoint_file)
            if not waypoints:
                self.get_logger().error('Failed to load waypoints')
                return
            
            # Create and publish Path message
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'
            path_msg.poses = waypoints
            
            self.path_pub.publish(path_msg)
            self.get_logger().info(f'Published path with {len(waypoints)} waypoints')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
    
    def load_waypoints_from_file(self, filepath: str):
        """Load waypoints from CSV or YAML file."""
        waypoints = []
        
        if filepath.endswith('.csv'):
            waypoints = self.load_csv(filepath)
        elif filepath.endswith('.yaml') or filepath.endswith('.yml'):
            waypoints = self.load_yaml(filepath)
        else:
            self.get_logger().error(f'Unsupported file format: {filepath}')
        
        return waypoints
    
    def load_csv(self, filepath: str):
        """Load waypoints from CSV format."""
        waypoints = []
        with open(filepath, 'r') as f:
            for line in f:
                parts = line.strip().split(',')
                if len(parts) >= 2:
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = float(parts[0])
                    pose.pose.position.y = float(parts[1])
                    pose.pose.position.z = 0.0
                    if len(parts) >= 4:  # Has orientation
                        pose.pose.orientation.z = float(parts[2]) if len(parts) > 2 else 0.0
                        pose.pose.orientation.w = float(parts[3]) if len(parts) > 3 else 1.0
                    else:
                        pose.pose.orientation.w = 1.0
                    waypoints.append(pose)
        return waypoints
    
    def load_yaml(self, filepath: str):
        """Load waypoints from YAML format."""
        import yaml
        waypoints = []
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        
        if 'waypoints' in data:
            for wp in data['waypoints']:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = float(wp.get('x', 0))
                pose.pose.position.y = float(wp.get('y', 0))
                pose.pose.position.z = float(wp.get('z', 0))
                pose.pose.orientation.x = float(wp.get('qx', 0))
                pose.pose.orientation.y = float(wp.get('qy', 0))
                pose.pose.orientation.z = float(wp.get('qz', 0))
                pose.pose.orientation.w = float(wp.get('qw', 1))
                waypoints.append(pose)
        
        return waypoints


def main(args=None):
    rclpy.init(args=args)
    node = AutoLoadWaypoints()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
