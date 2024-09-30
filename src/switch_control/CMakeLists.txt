cmake_minimum_required(VERSION 3.8)
project(switch_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(class_loader REQUIRED)   # <-- Add this line
find_package(pluginlib REQUIRED)      # It's good to ensure all dependencies are found
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

set (THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp  
  class_loader
  pluginlib
  Qt5
  rviz2
  rviz_common
  rviz_default_plugins
  rviz_rendering
  rviz_ogre_vendor
)

include_directories(
  include
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# Define source file
set(${PROJECT_NAME}_SRCS
  src/switch_panel.cpp
  src/switch_widget.cpp
)

# Define header file
set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/switch_panel.h
  include/${PROJECT_NAME}/switch_widget.h
)

# Define ui file
set(${PROJECT_NAME}_UIS
  resource/pointcloud_controller.ui
)

# Add the executable
add_executable(cmd_vel_muxer src/cmd_vel_muxer.cpp)
ament_target_dependencies(cmd_vel_muxer rclcpp geometry_msgs std_msgs)

message(STATUS "Generate header for ui with rviz2_QT_VERSION: ${rviz2_QT_VERSION}")
qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
foreach(header "${${PROJECT_NAME}_HDRS}")
  qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${header})
endforeach()

## Add library is needed in order to generate the header file from ui file.
add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_UIS_H}
  ${${PROJECT_NAME}_MOCS}
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(${PROJECT_NAME} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} rviz_common::rviz_common)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common switch_plugin.xml)

# Install the executable
install(TARGETS cmd_vel_muxer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install launch file
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
