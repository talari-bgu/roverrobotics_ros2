# ROS2 Driver for Rover Robots - BGU
## About
* This repository uses original Roverrobotics repository found in - https://github.com/RoverRobotics/roverrobotics_ros2.
* This package is exclusively built for ROS2. It is being tested on Ubuntu 22.04 with ROS2-Humble and on Rover Zero 3.

## Robots Specefications and Instructions
There are two platforms in the lab, numbered 1 and 2. Each of them contains the following accessories:
* Dell laptop and charger.
* Docking stations.
* PS controller
* 2wd and 4wd kits (6 wheel each in total).
* Payload

Sensors:
* Intel realsense d435i
* Slamtech rplidar S2
* IMU bno055

There are two options to work on the robots
### Option 1 - SSH remote connection
_Recommneded option for both developing and operating the robot_
Make sure that the SSH server and client are installed on the corresponding machines. The NUCs should have the SSH server preinstalled, and the laptops should have the SSH client preinstalled. The command is: ssh {_User_}@{_IP_ADDRESS_}. if the command was successful than a new line will appear asking for a password. The default user and password are both __rover__. To check the IPs of the robot use the command _ifconfig_.  
eg.
```bash
ssh rover@132.73.222.123
rover@132.73.222.123 password:
Welcome to Ubuntu 22.04.3 LTS (GNU/Linux 6.2.0-33-generic x86_64)

 * Documentation:  https://help.ubuntu.com
 * Management:     https://landscape.canonical.com
 * Support:        https://ubuntu.com/advantage

Expanded Security Maintenance for Applications is not enabled.

689 updates can be applied immediately.
227 of these updates are standard security updates.
To see these additional updates run: apt list --upgradable

27 additional security updates can be applied with ESM Apps.
Learn more about enabling ESM Apps service at https://ubuntu.com/esm

Last login: Mon May  6 11:44:20 2024 from 192.168.1.122
rover@rover-NUC-2:~$
```

### Option 2 - direct connection
_This option is not suitable for moving the robot_
Connect the NUC pc inside of the robot to a monitor using HDMI cable and mouse and keyboard using USB port (you might need an USB hub).  
If you're planning to work for an extended period of time then it is recommended to place the robot on the docking station for the battery to be charged. Don't leave the platform charging when not in use, as lithium batteries wear out if they are constantly being charged.


## Prerequisite and Installation
First, clone this repository and build it using colcon like any other package.
```bash
cd ~
git clone https://github.com/talari-bgu/roverrobotics_ros2
cd ~/roverrobotics_ros2
colcon build --symlink-install
bash installation.sh
source ~/.bashrc
```

* *--synlink-install* - syncs between the directories "src" -> "install". Instead of building each time after modifying the code, files can be changed in the source ("src") space (e.g. Python files or other non-compiled resources) for faster iteration.  
* *.bashrc* - is a file that runs each time we open a new terminal. To make our environment acknowledge our new workspace named *'roverrobotics_ros2,'* we need to source (i.e., open) the workspace installation file, which is *'install/setup.bash.'* Instead of sourcing it each time, we can save it in the .bashrc file, which will run it automatically for us. Make sure that in the .bashrc file, you only source the workspace once, as more than one instance is irrelevant.

The repository already contains rplidar s2  and IMU bno055 packages.  
Since camera isn't preinstalled, install using: 
```bash
sudo apt install ros-humble-librealsense2*
sudo apt install ros-humble-realsense2-*
```
First command installs the intel realsense SDK, and second line installs ROS2 wrapper.  
If face any problems with this part please visit https://github.com/IntelRealSense/realsense-ros.

If you want to reinstall sensors or update them:
Install the lidar using:
```bash
cd ~/roverrobotics_ros2/src
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
cd ~/roverrobotics_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ~/.bashrc
```

Install the IMU using:
```bash
cd ~/roverrobotics_ros2/src
git clone https://github.com/flynneva/bno055.git
cd ~/roverrobotics_ros2
colcon build --symlink-install
source ~/.bashrc
```

After installtion you need to setup udev rules.  
```udev rules let you define a name for specific devices or device types based on attributes like vendor ID, product ID, serial number, or other properties, allowing for consistent device names across reboots and different USB ports.```

Verify that sensor are being detected as defined name in the following list.
```bash
ls /dev
```

## Setup  
From now on, 'roverrobotics_ros2' is our main directory, and every path will be relative to it unless stated otherwise.
You need to manually check and activate sensors. to do that, open *'accessories.yaml'* which located in *'roverrobotics_driver/config'*. 
Each of the accessories located there (BNO055 irelevant) has 'active' line. simply change true/false to de/activate the corresponding attachment. Example:
```yaml
# RPLidar S2 Settings
rplidar:
  ros__parameters:
    active: false # can be changed to "true"
    serial_port: "/dev/ttyUSB0" # might cause problem if not set correctly
    serial_baudrate: 1000000
    frame_id: "lidar_link"
```

If for some reason the lidar doesnt get recognized, first thing to check is if the serial port configured correctly. use *'lsusb -t'* to list serial devices and check for the ttyUSBX (X is a number). if the command doesnt show tty ids then 
you should create udev. google it.

## Usage

### 1. Command to bringup robot:
With ps3 joystick:
```bash
ros2 launch roverrobotics_driver zero_teleop.launch.py
```
Without:
```bash
ros2 launch roverrobotics_driver zero.launch.py
```

### 2. Mapping environment
First, you bringup the robot, then in a second terminal run:
```bash
ros2 launch roverrobotics_driver slam_launch.py
```
In a third terminal, run:
```bash
rviz2
```
On the rviz window open slam config:
File -> Open config -> slam_rviz_layout.rviz

After moving the robot around with the joystick and mapping the environment save the map:
```bash

```

### 3. Autonomous navigation using interface:

```bash
ros2 launch roverrobotics_driver navigation_launch.py map_file_name:=<file_name>
```


## Troubleshooting

### Caught exception in launch (see debug for traceback): executable 'joys_manager.py' not found on the libexec directory
Check if joys_manager.py exists in 'install/roverrobotics_input_manager/lib/roverrobotics_input_manager'.  
___FIX (file exists?)___:  
YES - Navigate to the dir using cd, then run the command chmod +x joys_manager.py to make it executable.  
NO - Problem might causes of the cmakelists file in the package check that its correct.  
