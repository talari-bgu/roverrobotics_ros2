import pygame
import time
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger

class JoystickHandler(Node):
    def __init__(self):
        super().__init__("joystick_dev")
        
        self.linear_scale = 0.5
        self.angular_scale = 0.8
        self.deadzone = 0.1

        # ROS publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, "cmd_vel_dev", 1)
        
        self.set_mode_client = self._connect_service(SetBool, "set_mode_dev")
        self.set_lock_client = self._connect_service(Trigger, "set_lock")



        # Initialize Pygame and the joystick
        pygame.init()
        pygame.joystick.init()

        # Check if any joysticks are connected
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No joystick connected.")

        # Connect to the first joystick
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"Connected to joystick: {self.joystick.get_name()}")

        self.running = True
        self.state = {"axes": [0.0, 0.0], "buttons": [0]}

    def _connect_service(self, srv_type, srv_name, timeout=1.0):

        client = self.create_client(srv_type, srv_name)
        while not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().info(f"Waiting for '{srv_name}' service...")

        self.get_logger().info(f"Service '{srv_name}' is available.")
        return client

    def get_joystick_state(self):
        while self.running:
            pygame.event.pump()  # Process events to update the joystick state
            # linear - axes[1], angular - axes[3]
            axes = [self.joystick.get_axis(i) for i in range(1,5,2)]

            # 0 - X
            # 1 - O
            # 2 - triangle
            # 3 - square
            # 4 - L1
            # 5 - R1
            # 6 - L2
            # 7 - R2
            # 8 - Share
            # 9 - options
            # 10 - center
            # 11 - left joystick
            # 12 - right joystick
            buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]

            # Arrow buttons
            # hats = [self.joystick.get_hat(i) for i in range(self.joystick.get_numhats())]

            self.state = {"axes": axes, "buttons": buttons}
            # print(self.state)
            time.sleep(0.02) # 50 Hz

    def _publish_cmd_vel(self):
        while self.running and rclpy.ok():
            axes = self.state["axes"]
            
            # Apply deadzone and scaling
            linear = 0.0 if abs(axes[0]) < self.deadzone else axes[0] * self.linear_scale * (-1)
            angular = 0.0 if abs(axes[1]) < self.deadzone else axes[1] * self.angular_scale * (-1)

            # Create and publish Twist message
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.publisher.publish(twist)
            time.sleep(0.1) # 10 Hz
            # self.get_logger().info(f"{axes}")
            # self.get_logger().info(f"Published Twist: linear={linear}, angular={angular}")

    def _monitor_buttons(self):
        """Monitor button 1 and call the set_mode service when pressed."""
        while self.running:
            # Check if button [1] is pressed
            if self.joystick.get_button(4):
                print(f"L1 pressed. Toggling set_mode to low.")
                self._call_service(self.set_mode_client ,True)

            elif self.joystick.get_button(6):
                print(f"L2 pressed. Toggling set_mode to high.")
                self._call_service(self.set_mode_client ,False)

            elif self.joystick.get_button(8):
                print(f"Share pressed. Toggling lock.")
                self._call_service(self.set_lock_client ,None)
            time.sleep(0.5)

    def _call_service(self, client, data=None):
        if data is None:
            request = Trigger.Request()
        else:
            request = SetBool.Request()
            request.data = data

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Service response: {future.result().message}")
        else:
            self.get_logger().error("Failed to call set_mode service.")

    def run(self):
        joystick_thread = threading.Thread(target=self.get_joystick_state)
        joystick_thread.start()

        button_monitor_thread = threading.Thread(target=self._monitor_buttons)
        button_monitor_thread.start()

        print("Press Ctrl+C to stop.")
        try:
            self._publish_cmd_vel()

        except KeyboardInterrupt:
            print("\nExiting...")
        finally:
            self.running = False
            joystick_thread.join()
            # button_monitor_thread.join()
            self.close()

    def close(self):
        """
        Cleans up resources used by the joystick and ROS.
        """
        self.joystick.quit()
        pygame.joystick.quit()
        pygame.quit()
        rclpy.shutdown()
        print("Joystick handler closed.")

if __name__ == "__main__":
    try:
        # Initialize ROS 2
        rclpy.init()

        joystick_handler = JoystickHandler()

        # Run the joystick handler
        joystick_handler.run()
    except RuntimeError as e:
        print(e)
