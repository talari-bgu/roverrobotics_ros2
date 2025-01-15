cmake_minimum_required(VERSION 3.5)
project(robot_srv)

# Required packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)  # Add this line

# Generate the service interface
rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/SetString.srv"
    "srv/SetMode.srv"
    DEPENDENCIES builtin_interfaces
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Install the service definition
install(
    DIRECTORY srv
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
