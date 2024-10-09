from pathlib import Path

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    roverrobotics_driver_dir = get_package_share_directory('roverrobotics_driver')
    topic_config = Path(get_package_share_directory('switch_control'), 'config', 'topics.yaml')
    
    # Put the name of the map in map directory
    map_file_name = 'lab131_2' 


    return LaunchDescription([
        # Robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [roverrobotics_driver_dir, '/launch/zero.launch.py']),
            launch_arguments={'use_sim_time': 'false'}.items()
        ),

        # Controller - Joystick
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [roverrobotics_driver_dir, '/launch/ps4_controller.launch.py']),
            launch_arguments={'topics_config': str(topic_config)}.items()
        ),

        # Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [roverrobotics_driver_dir, '/launch/navigation_launch.py']
            ),
            launch_arguments={'map_file_name': map_file_name}.items()
        ),

        # Switch control node
        Node(
            package='switch_control',
            executable='cmd_vel_muxer',
            name='command_controller',
            output='screen'
        ),
    ])
