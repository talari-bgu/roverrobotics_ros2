#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to a quaternion.
    :param roll: Rotation around the X-axis in radians.
    :param pitch: Rotation around the Y-axis in radians.
    :param yaw: Rotation around the Z-axis in radians.
    :return: A tuple (x, y, z, w) representing the quaternion.
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (x, y, z, w)

class ImuToOdomNode(Node):
    def __init__(self):
        super().__init__('imu_to_odom_node')

        # Create a subscriber to /imu/data
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Create a publisher for /odometry/imu
        self.odom_publisher = self.create_publisher(Odometry, '/odometry/imu', 10)

        # Initialize variables for velocity integration
        self.prev_time = self.get_clock().now()
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.theta = 0.0  # Orientation angle

        # IMU bias calibration
        self.calibration_samples = 1  # Number of samples to use for calibration
        self.calibration_counter = 0
        self.accel_bias_x = 0.0
        self.accel_bias_y = 0.0
        self.gyro_bias_z = 0.0
        self.is_calibrated = False

    def imu_callback(self, msg: Imu):
        # Perform calibration if not calibrated yet
        if not self.is_calibrated:
            self.calibrate_imu(msg)
            return

        # Get the current time
        current_time = self.get_clock().now()

        # Calculate the time difference (dt) in seconds
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        # Extract linear acceleration from IMU data and subtract bias
        accel_x = msg.linear_acceleration.x - self.accel_bias_x
        accel_y = msg.linear_acceleration.y - self.accel_bias_y

        # Extract angular velocity from IMU data and subtract bias
        angular_velocity_z = msg.angular_velocity.z - self.gyro_bias_z

        # Integrate angular velocity to update the orientation (theta)
        self.theta += angular_velocity_z * dt

        # Integrate acceleration to get velocity in the robot's frame
        self.velocity_x += accel_x * dt
        self.velocity_y += accel_y * dt
        self.velocity_x = 0.0
        self.velocity_y = 0.0

        # Transform velocity to the global frame
        vel_x_global = self.velocity_x * np.cos(self.theta) - self.velocity_y * np.sin(self.theta)
        vel_y_global = self.velocity_x * np.sin(self.theta) + self.velocity_y * np.cos(self.theta)

        # Integrate velocity to get position
        self.position_x += vel_x_global * dt
        self.position_y += vel_y_global * dt
        self.position_x = 0.0
        self.position_y = 0.0

        # Create an Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set the position
        odom_msg.pose.pose.position.x = self.position_x
        odom_msg.pose.pose.position.y = self.position_y
        odom_msg.pose.pose.position.z = 0.0  # Assuming a flat 2D plane

        # Set the orientation using theta
        q_new = quaternion_from_euler(0, 0, self.theta)  # Convert roll=0, pitch=0, yaw=theta to quaternion
        odom_msg.pose.pose.orientation = Quaternion(
            x=q_new[0],
            y=q_new[1],
            z=q_new[2],
            w=q_new[3]
        )

        # Set the velocity in the robot's frame
        odom_msg.twist.twist.linear.x = self.velocity_x
        odom_msg.twist.twist.linear.y = self.velocity_y
        odom_msg.twist.twist.angular.z = angular_velocity_z

        # Publish the odometry message
        self.odom_publisher.publish(odom_msg)

    def calibrate_imu(self, msg: Imu):
        # Accumulate IMU readings to compute the average bias
        self.accel_bias_x += msg.linear_acceleration.x
        self.accel_bias_y += msg.linear_acceleration.y
        self.gyro_bias_z += msg.angular_velocity.z
        self.calibration_counter += 1

        if self.calibration_counter >= self.calibration_samples:
            # Calculate the average bias
            self.accel_bias_x /= self.calibration_samples
            self.accel_bias_y /= self.calibration_samples
            self.gyro_bias_z /= self.calibration_samples

            self.is_calibrated = True
            self.get_logger().info(f'IMU calibration completed: Bias X = {self.accel_bias_x}, Bias Y = {self.accel_bias_y}, Bias Z = {self.gyro_bias_z}')

def main(args=None):
    rclpy.init(args=args)
    node = ImuToOdomNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
