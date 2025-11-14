#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('robotcar_base')
    
    # 声明激光雷达位置参数
    laser_args = []
    
    # 前右激光雷达参数
    for param in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
        default_values = {
            'x': '0.1875', 'y': '-0.15', 'z': '0.15',
            'roll': '0.0', 'pitch': '0.0', 'yaw': '-0.6747'
        }
        laser_args.append(
            DeclareLaunchArgument(
                f'front_right_laser_{param}',
                default_value=default_values[param],
                description=f'Front right laser {param} position/orientation'
            )
        )
    
    # 后左激光雷达参数
    for param in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
        default_values = {
            'x': '-0.1875', 'y': '0.15', 'z': '0.15',
            'roll': '0.0', 'pitch': '0.0', 'yaw': '2.4669'
        }
        laser_args.append(
            DeclareLaunchArgument(
                f'rear_left_laser_{param}',
                default_value=default_values[param],
                description=f'Rear left laser {param} position/orientation'
            )
        )

    def launch_setup(context, *args, **kwargs):
        # 获取URDF文件路径
        urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'robotcar.xacro'])
        
        # 构建robot_description参数
        robot_description_content = Command([
            'xacro ', urdf_file,
            ' front_right_laser_x:=', LaunchConfiguration('front_right_laser_x'),
            ' front_right_laser_y:=', LaunchConfiguration('front_right_laser_y'),
            ' front_right_laser_z:=', LaunchConfiguration('front_right_laser_z'),
            ' front_right_laser_roll:=', LaunchConfiguration('front_right_laser_roll'),
            ' front_right_laser_pitch:=', LaunchConfiguration('front_right_laser_pitch'),
            ' front_right_laser_yaw:=', LaunchConfiguration('front_right_laser_yaw'),
            ' rear_left_laser_x:=', LaunchConfiguration('rear_left_laser_x'),
            ' rear_left_laser_y:=', LaunchConfiguration('rear_left_laser_y'),
            ' rear_left_laser_z:=', LaunchConfiguration('rear_left_laser_z'),
            ' rear_left_laser_roll:=', LaunchConfiguration('rear_left_laser_roll'),
            ' rear_left_laser_pitch:=', LaunchConfiguration('rear_left_laser_pitch'),
            ' rear_left_laser_yaw:=', LaunchConfiguration('rear_left_laser_yaw'),
        ])

        # robot_state_publisher节点
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(robot_description_content, value_type=str),
                'publish_frequency': 10.0
            }]
        )

        # 激光雷达标定参数节点（用于rqt动态调参）
        laser_calibration_node = Node(
            package='robotcar_base',
            executable='laser_calibration_node.py',
            name='laser_calibration_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([pkg_share, 'config', 'laser_calibration_params.yaml'])
            ]
        )

        # 动态TF发布器节点（实时更新激光雷达TF）
        dynamic_tf_publisher = Node(
            package='robotcar_base',
            executable='dynamic_laser_tf_publisher.py',
            name='dynamic_laser_tf_publisher',
            output='screen',
            parameters=[{
                'front_right_laser.position.x': LaunchConfiguration('front_right_laser_x'),
                'front_right_laser.position.y': LaunchConfiguration('front_right_laser_y'),
                'front_right_laser.position.z': LaunchConfiguration('front_right_laser_z'),
                'front_right_laser.orientation.roll': LaunchConfiguration('front_right_laser_roll'),
                'front_right_laser.orientation.pitch': LaunchConfiguration('front_right_laser_pitch'),
                'front_right_laser.orientation.yaw': LaunchConfiguration('front_right_laser_yaw'),
                'rear_left_laser.position.x': LaunchConfiguration('rear_left_laser_x'),
                'rear_left_laser.position.y': LaunchConfiguration('rear_left_laser_y'),
                'rear_left_laser.position.z': LaunchConfiguration('rear_left_laser_z'),
                'rear_left_laser.orientation.roll': LaunchConfiguration('rear_left_laser_roll'),
                'rear_left_laser.orientation.pitch': LaunchConfiguration('rear_left_laser_pitch'),
                'rear_left_laser.orientation.yaw': LaunchConfiguration('rear_left_laser_yaw'),
            }]
        )

        # 激光雷达坐标系重映射节点（将激光雷达数据映射到动态坐标系）
        laser_frame_remapper = Node(
            package='robotcar_base',
            executable='laser_frame_remapper.py',
            name='laser_frame_remapper',
            output='screen'
        )

        return [
            robot_state_publisher,
            laser_calibration_node,
            dynamic_tf_publisher,
            laser_frame_remapper,
        ]

    return LaunchDescription(laser_args + [
        OpaqueFunction(function=launch_setup)
    ])
