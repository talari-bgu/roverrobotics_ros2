# export to ~/.bashrc
echo 'source ~/roverrobotics_ros2/install/setup.sh' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc

# add etc/udev/rules.d/rov
cd ~/roverrobotics_ros2
sudo cp 55-roverrobotics.rules /etc/udev/rules.d/55-roverrobotics.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
