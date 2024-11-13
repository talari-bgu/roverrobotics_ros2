import json
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.duration import Duration

class NavigateThroughPosesNode(Node):
    def __init__(self):
        super().__init__('navigate_through_poses_node')
        # Create an action client for NavigateThroughPoses
        self.action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        
        # Center of person relative to map origin
        x0 = 18.0
        y0 = 0.16

        # Trajectory rotation in degrees
        rotation = 95

        with open("trajectory_points_flat_ellipse-front-left.json", "r") as file:
            data = json.load(file)

        self.waypoints = []
        for point in data:
            point_rotated = self.transform_point(math.ceil(float(point[0]) * 100) / 100, 
                                              math.ceil(float(point[1]) * 100) / 100, 
                                              math.ceil(float(point[2]) * 100) / 100, 
                                              x0,
                                              y0,
                                              rotation)
            self.waypoints.append(self.create_pose(point_rotated[0],point_rotated[1],point_rotated[2]))

    def transform_point(self, x, y, w, origin_x, origin_y, rotation_degrees):
    # Transforms a point (x, y) from trajectory coordinates to map coordinates
    # by applying a rotation and translation relative to a specified origin.
    
    # Parameters:
    #     x (float): X-coordinate of the point in trajectory space.
    #     y (float): Y-coordinate of the point in trajectory space.
    #     origin_x (float): X-coordinate of the origin in map space.
    #     origin_y (float): Y-coordinate of the origin in map space.
    #     rotation_degrees (float): Rotation angle in degrees, applied counterclockwise.
    
    # Returns:
    #     (float, float): Transformed (x, y) coordinates in map space.
    
        # Convert rotation angle to radians
        theta = math.radians(rotation_degrees)
        
        # Apply rotation
        x_rotated = x * math.cos(theta) - y * math.sin(theta)
        y_rotated = x * math.sin(theta) + y * math.cos(theta)
        w_rotated = w + theta

        # Translate relative to the map origin
        x_transformed = x_rotated + origin_x
        y_transformed = y_rotated + origin_y
        
        return x_transformed, y_transformed, w_rotated

    def create_pose(self, x, y, theta):
        # Helper function to create a PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(theta / 2)
        pose.pose.orientation.w = math.cos(theta / 2)
        return pose

    def send_goal(self):
        # Wait for the action server to be available
        self.action_client.wait_for_server()
        
        # Create a goal message
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self.waypoints
        goal_msg.behavior_tree = ''  # Optional: specify a behavior tree XML file if needed

        # Send the goal to the action server
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        # Callback for receiving feedback during navigation
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback}')

    def get_result_callback(self, future):
        result = future.result().result
        if result:  # If a result is returned, consider it as successful.
            self.get_logger().info('Navigation succeeded')
        else:
            self.get_logger().info('Navigation failed')

        # Shut down the node after completing the navigation
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = NavigateThroughPosesNode()
    node.send_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()