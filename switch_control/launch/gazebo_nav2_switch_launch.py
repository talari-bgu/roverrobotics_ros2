from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():

    roverrobotics_driver_dir = get_package_share_directory('roverrobotics_driver')


    return LaunchDescription([
        # Include switch_control launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/switch_control_launch.py']),
        ),

        # Include 2wd_rover_gazebo launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('roverrobotics_gazebo'), '/launch/2wd_rover_gazebo.launch.py']
            )
        ),

        # Include roverrobotics_driver launch with parameters
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [roverrobotics_driver_dir, '/launch/navigation_launch.py']),
            launch_arguments={'use_sim_time': 'true', 'map_file_name': 'map2'}.items()
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', roverrobotics_driver_dir + '/config/rviz_configs/nav2_default_view.rviz'],
            name='rviz2'
        ),
    ])
