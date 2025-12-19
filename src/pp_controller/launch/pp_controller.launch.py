import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('pp_controller')
    
    params_file = os.path.join(pkg_dir, 'config', 'pp_params.yaml')
    
    pp_node = Node(
        package='pp_controller',
        executable='pp_node',
        name='pp_controller',
        output='screen',
        parameters=[params_file],
    )
    
    return LaunchDescription([
        pp_node,
    ])
