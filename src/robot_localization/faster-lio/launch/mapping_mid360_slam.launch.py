from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    config_file = PathJoinSubstitution([get_package_share_directory('faster_lio'), 'config', 'mid360_slam.yaml'])

    # 可选：设置环境变量
    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    stdout_colorized_envvar = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')

    faster_lio_node = Node(
        package='faster_lio',
        executable='run_mapping_online',
        name='laserMapping',
        output='screen',
        parameters=[config_file]
    )
    lidar_tf = Node(
    name='lidar_tf',
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0','0','0.341989445','0','0','0','1','map','odom']
    #arguments=['0','0','0','0','0','0','1','map','odom']
    )
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # 参数顺序为：x y z roll pitch yaw parent_frame child_frame
        arguments=['0', '-0.12004', '0.00202', '0', '0', '0', 'livox_frame', 'base_link'],
    )

    return LaunchDescription([
        stdout_linebuf_envvar,
        stdout_colorized_envvar,
        faster_lio_node,
        lidar_tf,
        static_tf_node
    ])
