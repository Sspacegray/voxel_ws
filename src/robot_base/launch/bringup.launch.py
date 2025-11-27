from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use sim time if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "imu_port",
            default_value="/imu",
            description="The serial port for the IMU",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "imu_topic",
            default_value="imu",
            description="The topic name for the IMU",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="", 
            description="ROS2 namespace (optional, default: empty for root namespace)",
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    imu_port = LaunchConfiguration("imu_port")
    imu_topic = LaunchConfiguration("imu_topic")
    namespace = LaunchConfiguration("namespace")

    base_params = {
        'use_sim_time': use_sim_time,
    }

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("robot_base"), "urdf", "four_diff.xacro"]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
        "use_sim_time": use_sim_time
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("robot_base"),
            "config",
            "controllers.yaml",
        ]
    )
    # 构建节点列表
    nodes_list = [
        Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_controllers, base_params],  # {{ AURA-X: Fix - 移除robot_description参数，使用robot_state_publisher话题避免弃用警告. }}
                output="screen",
                remappings=[
                    ('~/robot_description', '/robot_description'),  # {{ AURA-X: Add - 重映射robot_description话题. }}
                ],
            ),

            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[
                    robot_description,
                    base_params,  # {{ AURA-X: Fix - 使用直接参数配置. }}
                ],
            ),

            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
                name="joint_state_broadcaster_spawner",  # 添加名称以便引用
            ),

            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller", "--controller-manager", "controller_manager"],
                name="robot_controller_spawner",  # 添加名称
            ),

            # Node(
            #     package="controller_manager",
            #     executable="spawner",
            #     # arguments=["imu_sensor_broadcaster", "--controller-manager", "controller_manager"],
            #     # name="imu_sensor_broadcaster_spawner",
            # ),

            # 独立的 IMU 驱动节点 - 使用外置 HipNuc IMU
            Node(
                package='hipnuc_imu',
                executable='talker',
                name='IMU_publisher',
                output='screen',
                parameters=[
                    PathJoinSubstitution([
                        FindPackageShare("hipnuc_imu"),
                        "config",
                        "hipnuc_config.yaml"
                    ])
                ],
            ),

            # Livox MID360 驱动 (含 IMU) - 已注释，改用外置 IMU
            # Node(
            #     package='livox_ros_driver2',
            #     executable='livox_ros_driver2_node',
            #     name='livox_lidar_publisher',
            #     output='screen',
            #     parameters=[
            #         {'xfer_format': 4},    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
            #         {'multi_topic': 0},    # 0-All LiDARs share the same topic
            #         {'data_src': 0},       # 0-lidar
            #         {'publish_freq': 10.0},
            #         {'output_data_type': 0},
            #         {'frame_id': 'livox_frame'},
            #         {'lvx_file_path': '/home/livox/livox_test.lvx'},
            #         {'user_config_path': PathJoinSubstitution([
            #             FindPackageShare("livox_ros_driver2"),
            #             "config",
            #             "MID360_config.json"
            #         ])},
            #         {'cmdline_input_bd_code': 'livox0000000001'}
            #     ],
            #     remappings=[
            #         ('/livox/imu', imu_topic),
            #         ('/livox/lidar', '/scan_points')
            #     ]
            # ),
    ]

    # 根据命名空间是否为空来决定是否使用PushRosNamespace和TF重映射
    if namespace:
        # 有命名空间时使用完整的命名空间配置
        bringup_cmd_group = GroupAction([
            PushRosNamespace(namespace=namespace),
            SetRemap("tf", "/tf"),  # TF重映射，将命名空间内的tf重映射到全局
            SetRemap("tf_static", "/tf_static"),  # TF重映射，将命名空间内的tf_static重映射到全局
        ] + nodes_list)
    else:
        # 无命名空间时直接使用节点列表
        bringup_cmd_group = GroupAction(nodes_list)

    nodes = [
        bringup_cmd_group,  # {{ AURA-X: Replace - 使用GroupAction替代单独的节点. }}
    ]

    return LaunchDescription(declared_arguments + nodes) 
