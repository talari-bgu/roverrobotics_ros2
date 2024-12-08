#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import time

class MoveTurtle(Node):
    def __init__(self):
        super().__init__('manual_control_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.twist = Twist()

        self.initial_x = 0
        self.initial_y = 0
        self.distance_travelled = 0

        # Declare variables to track button presses
        self.button_click = 0
        self.time_first_click = 0
        self.time_second_click = 0

    def check_word(self):
        button_click = 0
        time_first_click = 0
        time_second_click = 0

        const_time = 1.05
        decc = 0.092
        init_time = time.time()  # Using time.time() for the current time
        original_time = init_time
        acc_time = time.time() - init_time
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher.publish(self.twist)

        while acc_time <= (10 / 3):
            self.twist.linear.x = 0.36 * (time.time() - init_time)
            self.twist.angular.z = 0.0
            # print(self.twist.linear.x)
            self.publisher.publish(self.twist)
            data = self.wait_for_message('/joy', Joy, timeout_sec=1.0)
            acc_time = time.time() - init_time
            # if button_click == 0 and data.buttons[1] == 1:
            #     time_first_click = time.time() - original_time
            #     button_click += 1
            # if button_click == 1 and data.buttons[0] == 1:
            #     time_second_click = time.time() - original_time
            #     button_click += 1
            #     break

        init_time = time.time()
        while (time.time() - init_time) <= 1.05 and button_click <= 1:
            self.twist.linear.x = 1.2
            self.twist.angular.z = 0.0
            self.publisher.publish(self.twist)
            # print(self.twist.linear.x)
            data = self.wait_for_message('/joy', Joy, timeout_sec=1.0)
            if button_click == 0 and data.buttons[1] == 1:
                time_first_click = time.time() - original_time
                button_click += 1
            if button_click == 1 and data.buttons[0] == 1:
                time_second_click = time.time() - original_time
                button_click += 1
                break

        init_time = time.time()
        while (time.time() - init_time) <= 0.092 and button_click <= 1:
            self.twist.linear.x = 1.2 - 13.04 * (time.time() - init_time)
            self.twist.angular.z = 0.0
            self.publisher.publish(self.twist)
            # print(self.twist.linear.x)
            # data = self.wait_for_message('/joy', Joy, timeout_sec=1.0)
            # if button_click == 0 and data.buttons[1] == 1:
            #     time_first_click = time.time() - original_time
            #     button_click += 1

        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher.publish(self.twist)

        max_dist = 0
        min_dist = 0
        self.get_logger().info(f'first_time: {time_first_click}')
        self.get_logger().info(f'second_time: {time_second_click}')

        if time_first_click <= 10 / 3:
            max_dist = 0.36 * 0.5 * time_first_click ** 2
        elif 10 / 3 <= time_first_click and time_first_click <= 10 / 3 + 1.05:
            max_dist = 0.36 * 0.5 * (10 / 3) ** 2 + 1.2 * (time_first_click - 10 / 3)
        else:
            max_dist = 0.36 * 0.5 * (10 / 3) ** 2 + 1.2 * (1.05) + 1.2 * (time_first_click - 10 / 3 - 1.05) - 0.5 * 13.04 * (time_first_click - 10 / 3 - 1.05) ** 2

        if time_second_click <= 10 / 3:
            min_dist = 0.36 * 0.5 * time_second_click ** 2
        elif 10 / 3 <= time_second_click and time_second_click <= 10 / 3 + 1.05:
            min_dist = 0.36 * 0.5 * (10 / 3) ** 2 + 1.2 * (time_second_click - 10 / 3)
        else:
            min_dist = 0.36 * 0.5 * (10 / 3) ** 2 + 1.2 * (1.05) + 1.2 * (time_second_click - 10 / 3 - 1.05) - 0.5 * 13.04 * (time_second_click - 10 / 3 - 1.05) ** 2

        self.get_logger().info(f'max_distance: {max_dist}')
        self.get_logger().info(f'min_distance: {min_dist}')

    def wait_for_message(self, topic_name, msg_type, timeout_sec):
        # Custom function to wait for a single message with a timeout
        msg = None
        start_time = time.time()
        while (time.time() - start_time) < timeout_sec:
            try:
                msg = self.create_subscription(msg_type, topic_name, lambda msg: msg, 10)
                if msg:
                    print(msg)
            except Exception as e:
                self.get_logger().warn(f'Error waiting for message: {e}')
        return msg

def main(args=None):
    rclpy.init(args=args)
    move_turtle = MoveTurtle()
    move_turtle.check_word()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
