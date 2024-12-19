# ROS 2 core modules
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Standard ROS 2 message and service types
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from action_msgs.msg import GoalStatus

# Custom message and service types
from robot_srv.srv import SetString, SetMode

# ROS 2 Transform modules
from tf2_ros import Buffer, TransformListener

# Standard Python modules
import threading
import time
from datetime import datetime
import json
import os
import math

class ExperimentManager(Node):
    def __init__(self):
        super().__init__("experiment_manager")
        
        # 
        self.participant_num = 1
        self.switch_mode = "RI"

        # Parameters
        self.enable_triggers = True

        # Experiment
        self.enable_recording = False  # Enable recording
        self.observation_rate = 1.0  # 1 Hz observation rate
        self.save_rate = 5  # Save every 30 seconds
        self.started = False

        # Experiment record instance
        if self.enable_recording:
            self.experiment_record = ExperimentRecord(participant_num=self.participant_num, switch_mode=self.switch_mode, 
                                                      observation_rate=self.observation_rate, save_rate=self.save_rate)

        self.goal_manager = GoalManager()

        if self.enable_triggers:
            self.trigger_manager = TriggerManager(self.switch_mode, self.handle_trigger_event)

        # Position
        self.position_update_rate = 0.5
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.stop_event = threading.Event()
        self.position_thread = threading.Thread(target=self.position_callback)

        # Sub
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 1)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 1)
        self.loa_sub = self.create_subscription(String, "/current_loa", self.loa_callback, 1)
        
        # Robots state
        self.current_position = None
        self.current_orientation = None
        self.current_cmd_vel = None
        self.current_scan = None
        self.current_loa = None

        # Services - Creation
        self.run_experiment_service = self.create_service(Trigger, "run_experiment", self.run_experiment_callback)
        self.next_goal_service = self.create_service(Trigger, "next_goal", self.next_goal_callback)
        self.previous_goal_service = self.create_service(Trigger, "previous_goal", self.previous_goal_callback)

        # Services - Connection
        self.set_ui_client = self._connect_service(SetString, "set_ui")
        self.set_sa_client = self._connect_service(Trigger, "set_sa")
        self.set_loa_client = self._connect_service(SetMode, "set_loa")
        self.set_lock_client = self._connect_service(SetBool, "set_lock_bool")
        self.set_control_mode_client = self._connect_service(SetBool, "set_control_mode")


        # Nav2 Action
        self.navigate_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        while not self.navigate_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Waiting for NavigateThroughPoses action server...')
        self.get_logger().info('Connected to NavigateThroughPoses action server...')
        
        
        self.get_logger().info("ExperimentManager is ready.")

    ######################################################################
    # Run method
    def run_experiment_callback(self, request, response):
        
        if self.started:
            response.success = False
            response.message = "Started already."
            return response
        
        self.started = True
        self.goals_reached = []

        # Starting parameters
        if self.switch_mode == 'RI':
            self.call_ui_service('navigating')
            time.sleep(0.2)
            self.call_switch_mode_service(False)
            time.sleep(0.2)
            self.call_loa_switch_service('high')

        elif self.switch_mode == 'HI':
            self.call_ui_service('ready')
            time.sleep(0.2)
            self.call_switch_mode_service(True)
            time.sleep(0.2)
            self.call_loa_switch_service('low')

        if self.enable_recording:
            self.record_timer = self.create_timer(1.0 / self.observation_rate, self.record_observation)

        if self.enable_triggers:
            self.trigger_check_timer = self.create_timer(0.5, self.check_active_trigger)

        # Starting listening to position
        self.position_thread.start()

        # Send the first goal
        first_goal = self.goal_manager.get_current_goal()
        if first_goal:
            name, waypoints = first_goal
            self.get_logger().info(f"Sending first goal: {name}")
            self.send_goal(waypoints)
            response.success = True
            response.message = f"Experiment started with first goal: {name}"
        else:
            self.get_logger().error("No goals available in GoalManager.")
            response.success = False
            response.message = "No goals available in GoalManager."

        return response
    
    
    ######################################################################
    # Scenario Manager
    def handle_trigger_event(self, trigger):
        # Handle the event based on the trigger name or type
        self.get_logger().info(f"Handling triger: {trigger.name}")
        if trigger.name in ['SA1', 'SA2', 'SA3', 'SA4']:
            if trigger.name == 'SA4':
                self.trigger_manager.running = False
            self.call_lock_service(True)
            time.sleep(0.2)
            self.call_sa_service()

        elif trigger.name in ['O1_start', 'O2_start', 'O3_start', 'O4_start',]:
            if self.switch_mode == "RI":
            # lower loa and ui
                self.call_loa_switch_service('low')
                time.sleep(0.2)
                self.call_ui_service('help')
            else:
                self.call_ui_service('collision')

        elif trigger.name in ['O1_end', 'O2_end', 'O3_end', 'O4_end']:
            if self.switch_mode == "RI":
            # increase loa and ui
                self.call_loa_switch_service('high')
                time.sleep(0.2)
                self.call_ui_service('navigating')
            else:
                self.call_ui_service('ready')
            next_goal = self.goal_manager.return_next_goal()
            if next_goal:
                name, waypoints = next_goal
                self.send_goal(waypoints)

        # elif trigger.name == 'O2_start':
        #     if self.switch_mode == "RI":
        #     # lower loa and ui
        #         self.call_loa_switch_service('low')
        #         time.sleep(0.2)
        #         self.call_ui_service('help')
        #     else:
        #         self.call_ui_service('collision')
        # elif trigger.name == 'O2_end':
        #     if self.switch_mode == "RI":
        #     # increase loa and ui
        #         self.call_loa_switch_service('high')
        #         time.sleep(0.2)
        #         self.call_ui_service('navigating')
        #     else:
        #         self.call_ui_service('ready')
        #     next_goal = self.goal_manager.return_next_goal()
        #     if next_goal:
        #         name, waypoints = next_goal
        #         self.send_goal(waypoints)

        # elif trigger.name == 'O3_start':
        #     if self.switch_mode == "RI":
        #     # lower loa and ui
        #         self.call_loa_switch_service('low')
        #         time.sleep(0.2)
        #         self.call_ui_service('help')
        #     else:
        #         self.call_ui_service('collision')
        # elif trigger.name == 'O3_end':
        #     if self.switch_mode == "RI":
        #     # increase loa and ui
        #         self.call_loa_switch_service('high')
        #         time.sleep(0.2)
        #         self.call_ui_service('navigating')
        #     else:
        #         self.call_ui_service('ready')
        #     next_goal = self.goal_manager.return_next_goal()
        #     if next_goal:
        #         name, waypoints = next_goal
        #         self.send_goal(waypoints)

        # elif trigger.name == 'O4_start':
        #     if self.switch_mode == "RI":
        #     # lower loa and ui
        #         self.call_loa_switch_service('low')
        #         time.sleep(0.2)
        #         self.call_ui_service('help')
        #     else:
        #         self.call_ui_service('collision')
        # elif trigger.name == 'O4_end':
        #     if self.switch_mode == "RI":
        #     # increase loa and ui
        #         self.call_loa_switch_service('high')
        #         time.sleep(0.2)
        #         self.call_ui_service('navigating')
        #     else:
        #         self.call_ui_service('ready')
        #     next_goal = self.goal_manager.return_next_goal()
        #     if next_goal:
        #         name, waypoints = next_goal
        #         self.send_goal(waypoints)
    
    ######################################################################
    # Utils
    def cmd_vel_callback(self, msg):
        self.current_cmd_vel = (msg.linear.x, msg.linear.y, msg.linear.z)

    def scan_callback(self, msg):
        self.current_scan = msg.ranges

    def position_callback(self):
        while not self.stop_event.is_set():
            try:
                # Fetch transform (map -> base_link)
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                position = (transform.transform.translation.x, transform.transform.translation.y)
                orientation = (
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                )
                # Update current position
                self.current_position = position
                self.current_orientation = orientation

                # Calculate yaw (z-angle) from quaternion
                x, y, z, w = orientation
                z_angle = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
                # self.get_logger().info(f"Current Position: {self.current_position}, yaw:{z_angle}")
            except Exception as e:
                self.get_logger().warn(f"Could not get transform: {e}")

            time.sleep(self.position_update_rate)  # Adjust the polling frequency
    
    def loa_callback(self, msg):
        self.current_loa = msg.data

    def _connect_service(self, srv_type, srv_name, timeout=1.0):

        client = self.create_client(srv_type, srv_name)
        while not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().info(f"Waiting for '{srv_name}' service...")

        self.get_logger().info(f"Service '{srv_name}' is available.")
        return client
    
    def call_switch_mode_service(self, switch_mode):
        request = SetBool.Request()
        request.data = switch_mode

        future = self.set_control_mode_client.call_async(request)
        future.add_done_callback(self.handle_response)

    def call_loa_switch_service(self, loa):
        request = SetMode.Request()
        request.origin = "robot"
        request.data = loa

        future = self.set_loa_client.call_async(request)
        future.add_done_callback(self.handle_response)

    def call_lock_service(self, lock):
        request = SetBool.Request()
        request.data = lock

        future = self.set_lock_client.call_async(request)
        future.add_done_callback(self.handle_response)

    def call_ui_service(self, status):
        request = SetString.Request()
        request.data = status

        future = self.set_ui_client.call_async(request)
        future.add_done_callback(self.handle_response)

    def call_sa_service(self):
        request = Trigger.Request()

        future = self.set_sa_client.call_async(request)
        future.add_done_callback(self.handle_response)
    
    def handle_response(self, future):
        try:
            # Retrieve the result from the future
            response = future.result()
            if response.success:
                self.get_logger().info("Service call succeeded: " + response.message)
            else:
                self.get_logger().error("Service call failed: " + response.message)
        except Exception as e:
            # Handle any exceptions raised during the service call
            self.get_logger().error(f"Service call resulted in an error: {e}")

    def next_goal_callback(self, request, response):
        """Service callback to move to the next goal."""
        self.get_logger().info("Received request to move to the next goal.")

        next_goal = self.goal_manager.return_next_goal()
        if next_goal:
            name, waypoints = next_goal
            self.send_goal(waypoints)
            response.success = True
            response.message = f"Moved to next goal: {name}"
        else:
            self.get_logger().error("Already at the last goal.")
            response.success = False
            response.message = "Already at the last goal."

        return response

    def previous_goal_callback(self, request, response):
        """Service callback to move to the previous goal."""
        self.get_logger().info("Received request to move to the previous goal.")

        previous_goal = self.goal_manager.return_previous_goal()
        if previous_goal:
            name, waypoints = previous_goal
            self.send_goal(waypoints)
            response.success = True
            response.message = f"Moved to previous goal: {name}"
        else:
            self.get_logger().error("Already at the first goal.")
            response.success = False
            response.message = "Already at the first goal."

        return response
    
    ######################################################################
    # Nav2
    def send_goal(self, waypoints):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = []  # Initialize the list of poses

        # Build a list of PoseStamped messages for each waypoint
        for x, y, yaw in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            goal_msg.poses.append(pose)

        # self.get_logger().info(f"Sending goal with {len(waypoints)} waypoints.")
        goal_msg.behavior_tree = ''

        # Send the goal
        # self._send_goal_future = self.navigate_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future = self.navigate_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    

    def feedback_callback(self, feedback_msg):
        # Callback for receiving feedback during navigation
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result:
            current_goal = self.goal_manager.get_current_goal()
            if current_goal:
                goal_name, _ = current_goal
                self.get_logger().info(f"Goal {goal_name} reached successfully.")
                if goal_name == 'I1':
                    if self.enable_triggers:
                        self.trigger_manager.running = True
                        self.get_logger().info(f"Activating trigger checking.")
        else:
            self.get_logger().info('Failed to reach goal.')

    ######################################################################
    # Triggers
    def check_active_trigger(self):
        self.trigger_manager.check_active_trigger(self.current_position)

    def record_observation(self):
        if self.enable_recording and self.experiment_record:
            self.experiment_record.record(self.current_loa, self.current_position, self.current_orientation ,self.current_cmd_vel, self.current_scan)

    def destroy(self):
        # Stop recording gracefully
        if self.enable_recording and self.experiment_record:
            self.experiment_record.stop()
        super().destroy_node()


class GoalManager:
    def __init__(self):
        # Robot position 
        # I1 : (7.65, -7.00)
        # SA1 : (7.47, -1.37)
        # O1 : (7.46, 0.32)
        # O1 : (7.32, 3.14)
        # O2 : (4.59, 16.10)
        # O2 : (2.08, 16.90)
        # SA2 : (0.96, 17.30)
        # I2 : (-0.43, 17.20)
        # I3 : (0.08, 21.7)
        # I4 : (6.95, 15.80)
        # O3 : (6.86, 20.3)
        # O3 : (6.65, 22.9)
        # I5 : (3.61, 26.2)
        # SA3 : (6.30, 22.00)
        # O4 : (6.43, 13.90)
        # O4 : (6.60, 11.20)
        # SA4: (6.90, 5.98)

        self.goals = [
            ('I1', [(7.0, -5.5, -1,57),(7.65, -7.0, -1.57)]),
            # ('SA1', (7.47, -1.37, 1.57)),
            ('O1', [(7.6, 0.32, 1.57)]),
            ('O2', [(6.9, 16.0, 1.57), (4.59, 16.10, 2.9)]),
            # ('SA2', (0.96, 17.30, 3.14)),
            ('I2', [(-0.43, 17.20, 3.14)]),
            ('I3', [(0.82, 19.6, 1.57), (0.08, 21.7, 1.57)]),
            ('I4', [(3.2, 20.9, 0.0),(3.2, 17.1, -1.57),(6.95, 15.80, 0.0)]),
            ('O3', [(6.86, 20.3, 1.57)]),
            ('I5', [(6.5, 25.4, -1.57),(3.61, 26.2, 3.14)]),
            # ('SA3', (6.30, 22.00, -1.57)),
            ('O4', [(6.4, 13.90, -1.57)]),
            # ('SA4', (6.90, 5.98, -1.57)),
            ('end', [(0.0, 0.0, 3.14)])
        ]
        self.current_index = 0  # Start with the first goal

    def return_next_goal(self):
        """Move to the next goal and return it."""
        if self.current_index < len(self.goals) - 1:
            self.current_index += 1
            return self.goals[self.current_index]
        else:
            print("Already at the last goal!")
            return None

    def return_previous_goal(self):
        """Move to the previous goal and return it."""
        if self.current_index > 0:
            self.current_index -= 1
            return self.goals[self.current_index]
        else:
            print("Already at the first goal!")
            return None

    def get_current_goal(self):
        """Return the current goal without changing the index."""
        return self.goals[self.current_index]


class TriggerManager:
    def __init__(self, switch_mode, on_trigger_callback=None):
        # Robot position 
        # I1 : (7.65, -7.00)
        # SA1 : (7.47, -1.37)
        # O1 : (7.46, 0.32)
        # O1 : (7.32, 3.14)
        # O2 : (4.59, 16.10)
        # O2 : (2.08, 16.90)
        # SA2 : (0.96, 17.30)
        # I2 : (-0.43, 17.20)
        # I3 : (0.08, 21.7)
        # I4 : (6.95, 15.80)
        # O3 : (6.86, 20.3)
        # O3 : (6.65, 22.9)
        # I5 : (3.61, 26.2)
        # SA3 : (6.30, 22.00)
        # O4 : (6.43, 13.90)
        # O4 : (6.60, 11.20)
        # SA4: (6.90, 5.98)

        self.triggers = [
            EventTrigger("SA1", (7.32, -1.37 + 0.5), 0.5, 2),
            EventTrigger("O1_start", (7.46, 0.32 + 0.5), 0.5, 2),
            EventTrigger("O1_end", (7.32, 3.14 + 0.5),0.5, 2),
            EventTrigger("O2_start", (4.59 - 0.5, 16.10), 2, 0.5),
            EventTrigger("O2_end", (2.08 - 0.5, 16.90), 2, 0.5),
            EventTrigger("SA2", (0.96 - 0.5, 17.30), 2, 0.5),
            EventTrigger("O3_start", (6.3, 20.3 + 0.5), 0.5, 2),
            EventTrigger("O3_end", (6.3, 22.9 + 0.5), 0.5, 2),
            EventTrigger("SA3", (6.30, 22.00 - 0.5), 0.5 , 2),
            EventTrigger("O4_start", (6.64, 13.90 - 0.5), 0.5, 2),
            EventTrigger("O4_end", (6.70, 11.20 - 0.5), 0.5, 2),
            EventTrigger("SA4", (6.90, 5.98 - 0.5), 0.5, 2),
        ]

        self.current_index = 0 if self.triggers else -1
        
        self.running = False
        self.on_trigger_callback = on_trigger_callback

    def get_active_trigger(self):
        """Get the current active trigger."""
        if 0 <= self.current_index < len(self.triggers):
            return self.triggers[self.current_index]
        return None

    def next_trigger(self):
        """Move to the next trigger and return it."""
        if self.current_index < len(self.triggers) - 1:
            self.current_index += 1
            active_trigger = self.get_active_trigger()
            return active_trigger
        else:
            return None

    def check_active_trigger(self, current_position):
        """Check if the active trigger is activated."""
        if self.running:
            active_trigger = self.get_active_trigger()
            if not current_position or not active_trigger:
                return

            if active_trigger.is_triggered(current_position):
                self.handle_event(active_trigger)
                self.next_trigger()

    def handle_event(self, trigger):
        """Handle the event and notify the callback."""
        if self.on_trigger_callback:
            self.on_trigger_callback(trigger)  # Notify the ExperimentManager
    

class EventTrigger:
    def __init__(self, name, point=None, width=1.0, height=1.0):
        """
        Initialize an event trigger with a rectangle (axis-aligned).
        
        :param name: Name of the trigger.
        :param point: Center point (x, y) of the rectangle.
        :param width: Width of the rectangle.
        :param height: Height of the rectangle.
        """
        self.name = name
        self.center = point
        self.width = width
        self.height = height

        # Calculate the rectangle's axis-aligned borders
        half_width = width / 2
        half_height = height / 2
        self.border = (
            point[0] - half_width,  # x1 (left)
            point[1] - half_height, # y1 (bottom)
            point[0] + half_width,  # x2 (right)
            point[1] + half_height  # y2 (top)
        )

    def is_triggered(self, position):
        """
        Check if the trigger is activated based on the robot's current position.

        :param position: Current position (x, y) of the robot.
        :return: True if the trigger is activated, False otherwise.
        """
        if not self.border or not position:
            return False
        x1, y1, x2, y2 = self.border
        x, y = position
        return x1 <= x <= x2 and y1 <= y <= y2

    def __repr__(self):
        return (f"EventTrigger(name={self.name}, center={self.center}, width={self.width}, "
                f"height={self.height}, border={self.border})")
    

class ExperimentRecord:
    def __init__(self, participant_num, switch_mode, observation_rate, save_rate):
        self.participant_num = participant_num
        self.switch_mode = switch_mode
        self.observation_rate = observation_rate
        self.save_rate = save_rate

        # Define the base directory for saving data
        self.base_dir = os.path.expanduser("~/Desktop/Experiment2")
        self.participant_dir = os.path.join(self.base_dir, f"Participant_{self.participant_num}")
        os.makedirs(self.participant_dir, exist_ok=True)
        self.file_path = os.path.join(self.participant_dir, f"{self.switch_mode}.json")

        self.start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.end_time = None
        self.data = []
        self.lock = threading.Lock()
        self.running = True

        # Start the saving thread
        self.save_thread = threading.Thread(target=self.save_periodically)
        self.save_thread.start()

    def record(self, loa, position, orientation, cmd_vel, scan):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        observation = {
            "timestamp": timestamp,
            "loa": loa if loa else None,
            "position": position if position else None,
            "orientation": orientation if orientation else None,
            "cmd_vel":  cmd_vel if cmd_vel else None,
            "scan": list(scan) if scan else None,
        }

        # Lock for thread-safe access
        with self.lock:
            self.data.append(observation)

    def save_periodically(self):
        while self.running:
            time.sleep(self.save_rate)
            self.save_to_file()

    def save_to_file(self):
        with self.lock:
            if self.data:
                with open(self.file_path, "a") as f:
                    json.dump(self.data, f, indent=4)
                    self.data = []  # Clear the buffer after saving

    def stop(self):
        self.end_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.running = False
        self.save_thread.join()
        self.save_to_file()  # Final save at the end


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ExperimentManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Gracefully handle Ctrl+C
        node.get_logger().info("Experiment interrupted by user. Shutting down...")
    finally:
        # Ensure proper cleanup
        if node is not None:
            node.stop_event.set()
            node.position_thread.join()
            node.destroy_node()
        rclpy.shutdown()
        print("ExperimentManager has shut down cleanly.")

if __name__ == "__main__":
    main()