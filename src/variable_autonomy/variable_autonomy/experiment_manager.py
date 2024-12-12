import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger
from robot_srv.srv import SetString, SetMode
from nav_msgs.msg import Odometry
import threading
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class ExperimentManager(Node):
    def __init__(self):
        super().__init__("experiment_manager")
        
        # Parameters
        self.enable_triggers = True

        self.position_subscriber = self.create_subscription(Odometry, "/odometry/filtered", self.position_callback, 10)
        self.current_position = None

        self.run_experiment_service = self.create_service(Trigger, "run_experiment", self.run_experiment_callback)
        self.run_experiment_service = self.create_service(Trigger, "next_goal", self.next_goal_callback)
        self.run_experiment_service = self.create_service(Trigger, "previous_goal", self.previous_goal_callback)
        
        self.navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        while not self.navigate_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Waiting for NavigateToPose action server...')
        self.get_logger().info('Connected to NavigateToPose action server...')

        self.goal_manager = GoalManager()

        if self.enable_triggers:
            self.trigger_manager = TriggerManager(self.handle_trigger_event)
            # Timer to check triggers every 0.5 seconds
            self.trigger_check_timer = self.create_timer(0.5, self.check_active_trigger)

        self.get_logger().info("ExperimentManager is ready.")

    def position_callback(self, msg):
        position = msg.pose.pose.position

        self.current_position = (position.x, position.y)
        # self.get_logger().info(f"Current position: {self.current_position}")


    def run_experiment_callback(self, request, response):
        # Send the first goal
        first_goal = self.goal_manager.get_current_goal()
        if first_goal:
            name, (x, y, yaw) = first_goal
            self.get_logger().info(f"Sending first goal: {name} at (x={x}, y={y}, yaw={yaw})")
            self.send_goal(x, y, yaw)
            response.success = True
            response.message = f"Experiment started with first goal: {name}"
        else:
            self.get_logger().error("No goals available in GoalManager.")
            response.success = False
            response.message = "No goals available in GoalManager."

        return response
    

    def next_goal_callback(self, request, response):
        """Service callback to move to the next goal."""
        self.get_logger().info("Received request to move to the next goal.")

        next_goal = self.goal_manager.return_next_goal()
        if next_goal:
            name, (x, y, yaw) = next_goal
            self.get_logger().info(f"Sending next goal: {name} at (x={x}, y={y}, yaw={yaw})")
            self.send_goal(x, y, yaw)
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
            name, (x, y, yaw) = previous_goal
            self.get_logger().info(f"Sending previous goal: {name} at (x={x}, y={y}, yaw={yaw})")
            self.send_goal(x, y, yaw)
            response.success = True
            response.message = f"Moved to previous goal: {name}"
        else:
            self.get_logger().error("Already at the first goal.")
            response.success = False
            response.message = "Already at the first goal."

        return response
    

    def send_goal(self, x, y, yaw):


        goal_msg = NavigateToPose.Goal()
        
        # Set target pose
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = yaw  # Assuming a simple 2D orientation

        self.get_logger().info(f'Sending goal: x={x}, y={y}, yaw={yaw}')
        
        self._send_goal_future = self.navigate_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

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
            self.get_logger().info('Goal reached successfully.')
        else:
            self.get_logger().info('Failed to reach goal.')


    def check_active_trigger(self):
        self.trigger_manager.check_active_trigger(self.current_position)

    def handle_trigger_event(self, trigger):
        # Handle the event based on the trigger name or type
        self.get_logger().info(f"Handling event: {trigger.event_type} from trigger {trigger.name}")

    def activate_next_trigger(self):
        # Activate the next trigger in the list, if available
        self.active_trigger = self.triggers.pop(0) if self.triggers else None
        if self.active_trigger:
            self.get_logger().info(f"Next active trigger: {self.active_trigger.name}")

class GoalManager:
    def __init__(self):
        # Define goals as a list of tuples
        self.goals = [
            ('I_1', (7.4, -7.0, 0.0)),
            ('O_1', (7.3, 3.2, 0.0)),
            ('O_2', (3.2, 16.7, 0.0)),
            ('I_2', (0.0, 17.2, 0.0)),
            ('I_3', (-0.2, 21.4, 0.0)),
            ('I_4', (6.9, 16.0, 0.0)),
            ('O_3', (6.7, 20.5, 0.0)),
            ('I_5', (3.5, 26.0, 0.0)),
            ('O_4', (6.5, 10.5, 0.0)),
            ('end', (0.0, 0.0, 0.0))
        ]
        # self.goals = [
        #     ('I_1', (7.4, -7.0, -1.57)),
        #     ('O_1', (7.3, 3.2, 1.57)),
        #     ('O_2', (3.2, 16.7, -3.14)),
        #     ('I_2', (0.0, 17.2, -3.14)),
        #     ('I_3', (-0.2, 21.4, -1.57)),
        #     ('I_4', (6.9, 16.0, 0.0)),
        #     ('O_3', (6.7, 20.5, 1.57)),
        #     ('I_5', (3.5, 26.0, -3.14)),
        #     ('O_4', (6.5, 10.5, -1.57)),
        #     ('end', (0.0, 0.0, -3.14))
        # ]
        # self.goals = [
        #     ('I_1', (3.4, 0.0, 0.0)),
        #     ('O_1', (4.4, 0.0, 0.0)),
        #     ('O_2', (5.4, 0.0, 0.0)), 
        # ]
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
    def __init__(self, on_trigger_callback=None):

        self.triggers = [
            EventTrigger("trigger_1", "region_entry", (0.0, 0.0)),
            EventTrigger("trigger_2", "region_entry", (2.0, 0.0)),
            EventTrigger("trigger_3", "custom_event", (4.0, 4.0))
        ]

        self.current_index = 0 if self.triggers else -1

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
    

class EventTrigger():
    def __init__(self, name, event_type, point=None, precision = 0.5):

        self.name = name
        self.event_type = event_type
        self.border = (point[0] - precision, point[1] - precision, point[0] + precision, point[1] + precision)

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
        return f"EventTrigger(event_type={self.event_type}, border={self.border})"
    


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()