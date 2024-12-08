import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/webcam', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.publish_frame)  # Publish at 10 Hz
        
        # check using v4l2-ctl --device=/dev/video6 --list-formats-ext
        # pipeline = (
        # 'v4l2src device=/dev/video6 ! '
        # 'image/jpeg, width=640, height=480, framerate=20/1 ! '
        # 'jpegdec ! videoconvert ! appsink'
        # )   
        # self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        self.cap = cv2.VideoCapture(4)  # Open default webcam (change index if needed)
        # Set resolution (optional)
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # self.cap.set(cv2.CAP_PROP_FPS, 20)  # Set desired FPS
        
        if not self.cap.isOpened():
            self.get_logger().error("Webcam not found or cannot be opened!")
            self.destroy_node()
            rclpy.shutdown()


    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert the OpenCV frame to a ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
            # self.get_logger().info("Frame published")
        else:
            self.get_logger().error("Failed to capture frame from webcam")

    def destroy_node(self):
        self.cap.release()  # Release the webcam when shutting down
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
