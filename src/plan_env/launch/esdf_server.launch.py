from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import conditions
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('plan_env')
    config_file = os.path.join(pkg_share, 'config', 'esdf_config.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='ESDF配置文件路径'
        ),
        
        Node(
            package='plan_env',
            executable='esdf_server_node',
            name='esdf_server',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            remappings=[
                # 如果需要重映射话题，在这里添加
                # ('/scan', '/your_laser_topic'),
                # ('/odom', '/your_odom_topic'),
            ]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'esdf_view.rviz')],
            condition=conditions.IfCondition(
                LaunchConfiguration('use_rviz', default='true')
            )
        )
    ])
