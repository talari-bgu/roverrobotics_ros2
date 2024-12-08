import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger
from custom_srv.srv import SetString, SetMode
import threading

class CommandController(Node):
    def __init__(self):
        super().__init__("command_controller")

        self.dev_sub = self.create_subscription(Twist, "cmd_vel_dev", self.dev_callback, 1)
        self.teleop_sub = self.create_subscription(Twist, "cmd_vel_teleop", self.teleop_callback, 1)

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 1)
        self.cmd_vel_feedback_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_feedback_callback, 1)

        # ps4 joystick
        self.mode_dev_service = self.create_service(SetMode, "set_mode", self.set_mode_callback)
        self.lock_service = self.create_service(Trigger, "set_lock", self.set_lock_callback)
        
        self.set_ui_service = self._connect_service(SetString, '/set_ui', self.call_ui_service)

        # 0 - operator, 1 - dev
        self.current_mode = 1
        self.is_locked = False

        # Store last received commands
        self.last_op_cmd = Twist()
        self.last_dev_cmd = Twist()

        self.current_vel = Twist()
        self.linear_acc_limit = 0.5  # Maximum change in linear velocity per second
        self.linear_dec_limit = 1.0
        self.angular_acc_limit = 2.0  # Maximum change in angular velocity per second
        self.publish_rate = 50.0  # Hz

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_cmd_vel)
    
    def _connect_service(self, srv_type, srv_name, timeout=1.0):
        client = self.create_client(srv_type, srv_name)
        while not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().info(f"Waiting for '{srv_name}' service...")

        self.get_logger().info(f"Service '{srv_name}' is available.")
        return client

    def call_ui_service(self, data=None):
        request = SetString.Request()
        request.data = data

        future = self.set_ui_service.call_async(request)
        # rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Service response: {future.result().message}")
        else:
            self.get_logger().error("Failed to call ui service.")

    def teleop_callback(self, msg: Twist):
        self.last_op_cmd = msg

    def dev_callback(self, msg: Twist):
        self.last_dev_cmd = msg

    def cmd_vel_feedback_callback(self, msg: Twist):
        """Feedback callback for actual cmd_vel."""
        self.current_vel = msg

    def velocity_smoother(self, target_vel, current_vel, dt):
        smoothed_vel = Twist()

        # Smooth linear velocity
        linear_diff = target_vel.linear.x - current_vel.linear.x
        if linear_diff > 0:  # Accelerating
            max_linear_step = self.linear_acc_limit * dt
        else:  # Decelerating
            max_linear_step = self.linear_dec_limit * dt
    
        max_linear_step = self.linear_acc_limit * dt
        smoothed_vel.linear.x = current_vel.linear.x + max(
            -max_linear_step, min(max_linear_step, linear_diff)
        )

        # Smooth angular velocity
        angular_diff = target_vel.angular.z - current_vel.angular.z
        max_angular_step = self.angular_acc_limit * dt
        smoothed_vel.angular.z = current_vel.angular.z + max(
            -max_angular_step, min(max_angular_step, angular_diff)
        )

        return smoothed_vel
    
    def publish_cmd_vel(self):
        target_vel = self.last_op_cmd if self.current_mode == 0 else self.last_dev_cmd

        # Smooth the velocity using feedback
        dt = 1.0 / self.publish_rate
        smoothed_vel = self.velocity_smoother(target_vel, self.current_vel, dt)

        # Publish the smoothed velocity
        self.cmd_vel_pub.publish(smoothed_vel)
    
    def set_mode_callback(self, request, response):

        if request.origin == 'op' and self.is_locked:
            response.message = "Operator tried to switch but locked."
            response.success = True
            self.get_logger().info(response.message)
            return response

        # Dev
        if request.data == 'low' and self.current_mode != 0:
            self.current_mode = 0  # Manual mode
            response.message = "Dev changed to low."
            # self._call_service(self.set_mode_ui_service, True)
        elif request.data == 'high' and self.current_mode != 1:
            self.current_mode = 1  # Autonomous navigation mode
            response.message = "Dev changed to high."
            # self._call_service(self.set_mode_ui_service, False)

        # Operator
        elif request.data == 'switch':
            if self.current_mode == 0:
                self.current_mode = 1  # Autonomous navigation mode
                response.message = "Operator changed to high."
                # self._call_service(self.set_mode_ui_service, False)
            elif self.current_mode == 1:
                self.current_mode = 0  # Manual mode
                response.message = "Operator changed to low."
                # self._call_service(self.set_mode_ui_service, True)

        response.success = True
        self.get_logger().info(response.message)
        return response

    def set_lock_callback(self, request, response):

        if self.is_locked:
            self.is_locked = False  # Only dev can change
            response.message = "Set lock false"
        else:
            self.is_locked = True  # Both op and dev can change
            response.message = "Set lock true"

        response.success = True
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CommandController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
