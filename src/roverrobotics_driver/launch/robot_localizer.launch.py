#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )

    declare_config_file_argument = DeclareLaunchArgument(
        'config_file',
        default_value=str(Path(get_package_share_directory(
            'roverrobotics_driver'), 'config/localization_ekf.yaml')),
        description='Path to the localization config file'
    )

    # Start robot localization using an Extended Kalman filter
    localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )

    # Create LaunchDescription
    ld = LaunchDescription()

    # Add actions
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_config_file_argument)
    ld.add_action(localization_node)

    return ld
