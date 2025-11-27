from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    rviz = LaunchConfiguration('rviz', default='true')
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RVIZ'
    )

    config_file = PathJoinSubstitution([get_package_share_directory('faster_lio'), 'config', 'mid360.yaml'])
    rviz_config = PathJoinSubstitution([get_package_share_directory('faster_lio'), 'rviz_cfg', 'loam_livox.rviz'])

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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(rviz)
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
        rviz_arg,
        stdout_linebuf_envvar,  # 设置行缓冲环境变量 用于实时查看日志
        stdout_colorized_envvar, # 设置彩色输出环境变量 用于区分日志级别
        faster_lio_node,
        lidar_tf,
        rviz_node,
        static_tf_node
    ])
