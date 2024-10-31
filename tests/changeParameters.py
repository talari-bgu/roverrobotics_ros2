from rcl_interfaces.msg import ParameterValue

class InflationRadiusToggler(Node):
    def __init__(self):
        super().__init__('inflation_radius_toggler')
        self.local_costmap_client = self.create_client(SetParameters, '/local_costmap/set_parameters')
        self.global_costmap_client = self.create_client(SetParameters, '/global_costmap/set_parameters')
        
        # Initial toggle state
        self.inflation_radius = 0.35
        self.is_toggled = False

        # Wait for the services to be available
        self.local_costmap_client.wait_for_service()
        self.global_costmap_client.wait_for_service()

    def toggle_inflation_radius(self):
        # Toggle the inflation radius value
        self.inflation_radius = 0.1 if not self.is_toggled else 0.35
        self.is_toggled = not self.is_toggled

        # Create parameter request for inflation radius
        parameter = Parameter(
            name='inflation_layer.inflation_radius',
            value=ParameterValue(type=Parameter.Type.DOUBLE, double_value=self.inflation_radius)
        )

        local_request = SetParameters.Request(parameters=[parameter])
        global_request = SetParameters.Request(parameters=[parameter])

        # Send requests
        local_future = self.local_costmap_client.call_async(local_request)
        global_future = self.global_costmap_client.call_async(global_request)

        rclpy.spin_until_future_complete(self, local_future)
        rclpy.spin_until_future_complete(self, global_future)

        if local_future.result() is not None and global_future.result() is not None:
            self.get_logger().info(f"Inflation radius set to {self.inflation_radius}")
        else:
            self.get_logger().error("Failed to update inflation radius.")
