from setuptools import setup

package_name = 'variable_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/zero_bringup.launch.py']),
        ('share/' + package_name + '/config', ['config/ps4_joystick.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Package for managing variable autonomy levels.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'command_controller = variable_autonomy.command_controller:main',
            'realsense_imu_filter = variable_autonomy.realsense_imu_filter:main',
            'experiment_manager = variable_autonomy.experiment_manager:main',
            'ps4_joystick = variable_autonomy.ps4_joystick:main',
        ],
    },
)
