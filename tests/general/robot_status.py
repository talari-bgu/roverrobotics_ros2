#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import csv
import os

class RobotStatusSubscriber(Node):

    def __init__(self):
        super().__init__('robot_status_subscriber')

        # Create a subscription to the 'robot_status' topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/robot_status',  # Topic name
            self.robot_status_callback,
            10  # QoS history depth
        )
        self.subscription  # Prevent unused variable warning

        # List to store the selected data elements
        self.data_list = []

    def robot_status_callback(self, msg):
        # Ensure the message has at least the required number of elements
        if len(msg.data) >= 9:
            # Extract the required elements (2, 3, 7, and 8 -> indices 1, 2, 6, 7)
            selected_data = [msg.data[1], msg.data[2], msg.data[6], msg.data[7]]

            # Append the selected data to the list
            self.data_list.append(selected_data)

    def save_to_csv(self, file_path='robot_status_data.csv'):
        # Save the collected data to a CSV file
        self.get_logger().info(f'Saving data to {file_path}')
        with open(file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Motor 1 rpm', 'Motor 1 current', 'Motor 2 rpm', 'Motor 2 current'])  # Header
            writer.writerows(self.data_list)  # Data rows

        self.get_logger().info(f'Data successfully saved to {file_path}')


def main(args=None):
    rclpy.init(args=args)

    # Instantiate the node
    robot_status_subscriber = RobotStatusSubscriber()

    try:
        # Keep the node running to listen to messages
        rclpy.spin(robot_status_subscriber)
    except KeyboardInterrupt:
        # Handle termination, save to CSV
        robot_status_subscriber.get_logger().info('Terminating node...')
    finally:
        # Save the data to a CSV file when the node terminates
        robot_status_subscriber.save_to_csv()
        # Cleanup
        robot_status_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
