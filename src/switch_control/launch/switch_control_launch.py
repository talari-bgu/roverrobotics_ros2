import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='switch_control',
            executable='cmd_vel_muxer',
            name='command_controller',
            output='screen'
        ),
    ])