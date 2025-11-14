from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for the laser filter node.
    Filters outlier points from laser scan data.
    """
    # 声明启动参数
    use_filter = LaunchConfiguration('use_filter')
    source_topic = LaunchConfiguration('source_topic')
    pub_topic = LaunchConfiguration('pub_topic')
    outlier_threshold = LaunchConfiguration('outlier_threshold')
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'use_filter',
            default_value='true',
            description='Enable laser filter to remove outliers'
        ),
        DeclareLaunchArgument(
            'source_topic',
            default_value='/scan',
            description='Input laser scan topic'
        ),
        DeclareLaunchArgument(
            'pub_topic',
            default_value='/scan_filtered',
            description='Output filtered laser scan topic'
        ),
        DeclareLaunchArgument(
            'outlier_threshold',
            default_value='0.1',
            description='Threshold for outlier detection (meters)'
        ),
        
        # 激光雷达过滤器节点 - 仅在use_filter为true时启动
        Node(
            package='robotcar_base',
            executable='laser_filter_node_exe',
            name='lidar_filter',
            output='screen',
            condition=IfCondition(use_filter),
            parameters=[
                {'source_topic': source_topic},
                {'pub_topic': pub_topic},
                {'outlier_threshold': outlier_threshold}
            ]
        ),
    ]) 