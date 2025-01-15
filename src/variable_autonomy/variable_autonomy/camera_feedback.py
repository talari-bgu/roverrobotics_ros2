import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/webcam/color/image_raw', 15)
        self.timer = self.create_timer(1 / 15, self.publish_frame)  # 10 Hz
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(6)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open the camera")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert the OpenCV image to a ROS2 Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)

    def destroy_node(self):
        super().destroy_node()
        self.cap.release()  # Release the camera


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
