import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import csv
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class GyroDataRecorder(Node):
    def __init__(self):
        super().__init__('gyro_data_recorder')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribe to the gyro topic
        self.subscription = self.create_subscription(
            Imu,
            '/camera/realsense/gyro/filtered',
            self.gyro_callback,
            qos_profile
        )
        self.subscription  # prevent unused variable warning

        # Initialize variables
        self.start_time = time.time()
        self.data = []

        # Set recording duration (5 minutes)
        self.recording_duration = 2 * 60  # seconds

    def gyro_callback(self, msg):
        # Check if recording time is up
        if time.time() - self.start_time > self.recording_duration:
            self.save_data()
            rclpy.shutdown()
            return

        # Record angular_velocity.z
        self.data.append(msg.angular_velocity.z)

    def save_data(self):
        # Save the data to a CSV file
        filename = 'gyro_angular_velocity_z.csv'
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['AngularVelocityZ'])
            for value in self.data:
                writer.writerow([value])

        self.get_logger().info(f"Data saved to {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = GyroDataRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
