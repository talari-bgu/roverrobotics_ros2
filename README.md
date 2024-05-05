# ROS2 Driver for Rover Robots - BGU
## About
* This repository uses original Roverrobotics repository found in - https://github.com/RoverRobotics/roverrobotics_ros2.
* This package is exclusively built for ROS2. It is being tested on Ubuntu 22.04 with ROS2-Humble and on Rover Zero 3.
## Prerequisite and Installation
First, clone this repository and build it using colcon like any other package.
```
cd ~
git clone ttps://github.com/talari-bgu/roverrobotics_ros2
colcon build --symlink-install
echo "source ~/roverrobotics_ros2/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

* *--synlink-install* - syncs between the directories "src" -> "install". Instead of building each time after modifying the code, files can be changed in the source ("src") space (e.g. Python files or other non-compiled resources) for faster iteration.  
* *.bashrc* - is a file that runs each time we open a new terminal. To make our environment acknowledge our new workspace named *'roverrobotics_ros2,'* we need to source (i.e., open) the workspace installation file, which is *'install/setup.bash.'* Instead of sourcing it each time, we can save it in the .bashrc file, which will run it automatically for us. Make sure that in the .bashrc file, you only source the workspace once, as more than one instance is irrelevant.

Next, you would want to install rplidar s2  nd intel realsense d435i packages, since they dont come with the original repository.  
Install the lidar using:
```
cd ~/roverrobotics_ros2/src
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
cd ~/roverrobotics_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ~/.bashrc
```

Install the camera using:
```
sudo apt install ros-humble-librealsense2*
sudo apt install ros-humble-realsense2-*
```
First command installs the intel realsense SDK, and second line installs ROS2 wrapper.  
If face any problems with this part please visit https://github.com/IntelRealSense/realsense-ros.

## Setup  
From now on, 'roverrobotics_ros2' is our main directory, and every path will be relative to it unless stated otherwise.
You need to manually check and activate lidar or/and camera. to do that, open *'accessories.yaml'* which located in *'roverrobotics_driver/config'*
