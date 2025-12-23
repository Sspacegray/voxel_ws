#!/usr/bin/env python3
#
# Copyright 2023 6-robot.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Zhang Wanjie

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import time

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('wpr_simulation2'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to start Gazebo GUI')

    world = os.path.join(
        get_package_share_directory('wpr_simulation2'),
        'worlds',
        'robocup_home.world'
    )

    # Force software rendering to avoid driver-related crashes
    set_libgl_software = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')

    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', world, 
             '-slibgazebo_ros_init.so', 
             '-slibgazebo_ros_factory.so', 
             '-slibgazebo_ros_force_system.so',
             '--verbose'],
        output='screen',
        sigterm_timeout='5',
        sigkill_timeout='10'
    )

    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gui),
        sigterm_timeout='5',
        sigkill_timeout='10'
    )

    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_wpb_lidar.launch.py')
        ),
        launch_arguments={
            'pose_x': '-6.0',
            'pose_y': '-0.5',
            'pose_theta': '0.0',
            'use_sim_time': use_sim_time
        }.items()
    )

    spawn_objects = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_objects.launch.py')
        )
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    # ld.add_action(set_libgl_software)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(spawn_objects)
    
    return ld
