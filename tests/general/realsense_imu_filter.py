import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

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

    def imu_callback(self, msg):
        # Create a new IMU message with transformed angular velocity
        transformed_msg = Imu()
        transformed_msg.header = msg.header

        # Set only the Z angular velocity to the Y angular velocity
        transformed_msg.angular_velocity.x = 0.0
        transformed_msg.angular_velocity.y = 0.0
        transformed_msg.angular_velocity.z = (-1) * msg.angular_velocity.y - 0.00355 # callibrated

        # Publish the transformed message
        self.publisher.publish(transformed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TransformAngularVelocity()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()