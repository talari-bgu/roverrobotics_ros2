import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, SetBool
from robot_srv.srv import SetString, SetMode


class CommandController(Node):
    def __init__(self, initial_loa, initial_control_mode, initial_developer_lock):
        super().__init__("command_controller")

        # Muxer and Smoother
        self.dev_sub = self.create_subscription(Twist, "cmd_vel_dev", self.dev_callback, 1)
        self.teleop_sub = self.create_subscription(Twist, "cmd_vel_teleop", self.teleop_callback, 1)
        self.teleop_sub = self.create_subscription(Twist, "cmd_vel_nav2", self.nav2_callback, 5)

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 1)
        self.cmd_vel_feedback_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_feedback_callback, 1)

        # Services
        self.loa_service = self.create_service(SetMode, "set_loa", self.set_loa_callback)
        self.lock_service = self.create_service(Trigger, "set_lock", self.set_lock_callback)
        self.control_mode_service = self.create_service(SetBool, "set_control_mode", self.set_control_mode_callback)

        # UI service
        self.set_ui_service = self._connect_service(SetString, '/set_ui')

        # Initializing paramaters
        self.automation_level = initial_loa
        self.developer_lock = initial_developer_lock
        self.control_mode = initial_control_mode

        # Store last received commands
        self.last_op_cmd = Twist()
        self.last_dev_cmd = Twist()
        self.last_nav2_cmd = Twist()

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

        if future.result() is not None:
            self.get_logger().info(f"Service response: {future.result().message}")
        else:
            self.get_logger().error("Failed to call ui service.")

    def teleop_callback(self, msg: Twist):
        self.last_op_cmd = msg

    def dev_callback(self, msg: Twist):
        self.last_dev_cmd = msg

    def nav2_callback(self, msg: Twist):
        self.last_nav2_cmd = msg

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

        if self.developer_lock and self.last_dev_cmd:
            # Developer Joystick has priority
            target_vel = self.last_dev_cmd
        elif self.automation_level == 'low':
            target_vel = self.last_op_cmd
        elif self.automation_level == 'high':
            target_vel = self.last_nav2_cmd

        # Smooth the velocity using feedback
        dt = 1.0 / self.publish_rate
        smoothed_vel = self.velocity_smoother(target_vel, self.current_vel, dt)

        # Publish the smoothed velocity
        self.cmd_vel_pub.publish(smoothed_vel)
    
    def set_loa_callback(self, request, response):

        # Dev
        if request.origin == "dev":
            if request.data == "low" and self.automation_level != 'low':
                self.automation_level = 'low'  # Manual mode
                response.message = "Dev changed to low."
                # self.call_ui_service('low')
            elif request.data == "high" and self.automation_level != 'high':
                self.automation_level = 'high'  # Autonomous navigation mode
                response.message = "Dev changed to high."
                # self.call_ui_service('high')

        # Operator
        elif request.origin == "operator" and self.control_mode == 'HI':
            if self.developer_lock:
                response.message = "Operator tried to switch but locked."
                response.success = True
                self.get_logger().info(response.message)
                return response
            elif request.data == "switch":
                if self.automation_level == 'low':
                    self.automation_level = 'high'
                    response.message = "Operator changed to high."
                    self.call_ui_service('high')
                elif self.automation_level == 'high':
                    self.automation_level = 'low'
                    response.message = "Operator changed to low."
                    self.call_ui_service('low')
        # Robot
        elif request.origin == "robot" and self.control_mode == 'RI':
            if request.data == "low" and self.automation_level != 'low':
                self.automation_level = 'low'
                response.message = "robot changed to low."
                self.call_ui_service('low')
            elif request.data == "high" and self.automation_level != 'high':
                self.automation_level = 'high'
                response.message = "robot changed to high."
                self.call_ui_service('high')
        else:
            response.message = "set_loa empty."

        response.success = True
        self.get_logger().info(response.message)
        return response

    def set_lock_callback(self, request, response):

        if self.developer_lock:
            self.developer_lock = False  # Only dev can change
            response.message = "Set lock false"
        else:
            self.developer_lock = True  # Both op and dev can change
            response.message = "Set lock true"

        response.success = True
        self.get_logger().info(response.message)
        return response

    def set_control_mode_callback(self, request, response):
        if request.data:
            # HI
            self.control_mode = 'HI'
            response.message = "Set switch control mode to HI"
        else:
            # RI
            self.control_mode = 'RI'
            response.message = "Set switch control mode to RI"

        response.success = True
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CommandController('low', 'HI', False)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
