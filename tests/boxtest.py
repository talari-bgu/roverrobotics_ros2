#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class BoxTestNode(Node):
    def __init__(self):
        super().__init__('box_test_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_robot)

        # Define speed variables
        self.linear_vel = 0.1  # Linear velocity (m/s)
        self.rotation_vel = 0.5  # Angular velocity (rad/s)

        # Set the angle to rotate 
        self.degrees_to_rotate = 109  # in degrees
        self.radians_to_rotate = self.degrees_to_rotate * (3.14159 / 180)

        # Calculate duration for forward movement
        self.distance_to_travel = 0.60  # in meters
        self.linear_duration = self.distance_to_travel / self.linear_vel  # Time to drive forward

        # Calculate duration for rotation
        self.rotation_duration = self.radians_to_rotate / self.rotation_vel  # Time to rotate by the specified degrees

        self.state = 0
        self.start_time = time.time()
        self.waiting = False
        self.pause_duration = 1.0  # Pause for 1 second between each state
        self.loop_counter = 0  # Loop counter for repeating 4 times

    def move_robot(self):
        cmd = Twist()

        # If in waiting period after a state transition
        if self.waiting:
            if time.time() - self.start_time > self.pause_duration:
                self.waiting = False  # End waiting period and move to the next state
                self.start_time = time.time()
            return  # Do nothing during the pause

        if self.state == 0:  # Drive forward
            self.get_logger().info('Driving forward')
            cmd.linear.x = self.linear_vel  # Use the linear velocity
            if time.time() - self.start_time > self.linear_duration:  # Use precomputed linear duration
                self.state = 1
                self.start_time = time.time()
                self.waiting = True  # Start pause period

        elif self.state == 1:  # Rotate by the set degrees
            self.get_logger().info(f'Rotating counterclockwise')
            cmd.angular.z = self.rotation_vel  # Use the angular velocity
            if time.time() - self.start_time > self.rotation_duration:  # Use precomputed rotation duration
                self.state = 0
                self.start_time = time.time()
                self.waiting = True  # Start pause period
                self.loop_counter += 1  # Increment loop counter after a full drive+rotate cycle

        if self.loop_counter >= 4:  # If the robot has completed the box (4 sides)
            self.get_logger().info('Test finished, stopping robot')
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.publisher_.publish(cmd)
            rclpy.shutdown()  # This will stop the node and exit the program

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = BoxTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
