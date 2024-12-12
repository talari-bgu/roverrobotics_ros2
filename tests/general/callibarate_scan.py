import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import csv

class ScanSaver(Node):
    def __init__(self):
        super().__init__('scan_saver')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.data = []

        # Set a timer to stop after 10 seconds
        self.timer = self.create_timer(10.0, self.timer_callback)

    def scan_callback(self, msg):
        self.data.append(list(msg.ranges))

    def timer_callback(self):
        self.save_data()
        self.get_logger().info('Terminating node after 10 seconds.')
        rclpy.shutdown()

    def save_data(self):
        with open('scan_data.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([f'Range {i}' for i in range(len(self.data[0]))])
            writer.writerows(self.data)
        self.get_logger().info('Scan data saved to scan_data.csv')

def main(args=None):
    rclpy.init(args=args)
    node = ScanSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
