import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import time

class TransformAngularVelocity(Node):
    def __init__(self):
        super().__init__('realsense_imu_filter')

        # Define QoS profile matching the RealSense topic
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )


        # Subscription to the RealSense IMU topic
        self.subscription = self.create_subscription(Imu, '/camera/realsense/gyro/sample', self.imu_callback, qos_profile)

        # Publisher for the transformed angular velocity
        self.publisher = self.create_publisher(Imu, '/camera/realsense/gyro/filtered', qos_profile)

        # Calibration variables
        self.calibration_start_time = time.time()
        self.calibration_duration = 10  # seconds
        self.calibration_data = []
        self.calibration_offset = 0.0
        self.is_calibrating = True

    def imu_callback(self, msg):
        if self.is_calibrating:
            # Collect calibration data for 10 seconds
            current_time = time.time()
            if current_time - self.calibration_start_time < self.calibration_duration:
                self.calibration_data.append(msg.angular_velocity.y)
            else:
                # Calculate the calibration offset after 10 seconds
                if self.calibration_data:
                    self.calibration_offset = sum(self.calibration_data) / len(self.calibration_data)
                    self.get_logger().info(f"Calibration complete. Offset: {self.calibration_offset}")
                self.is_calibrating = False
            return

        # Create a new IMU message with transformed angular velocity
        transformed_msg = Imu()
        transformed_msg.header = msg.header

        # Apply calibration offset
        transformed_msg.angular_velocity.x = 0.0
        transformed_msg.angular_velocity.y = 0.0
        transformed_msg.angular_velocity.z = (-1) * (msg.angular_velocity.y - self.calibration_offset)

        # Publish the transformed message
        self.publisher.publish(transformed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TransformAngularVelocity()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()