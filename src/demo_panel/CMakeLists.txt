cmake_minimum_required(VERSION 3.5)
project(demo_panel)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
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


# I prefer the Qt signals and slots to avoid defining "emit", "slots",
# etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here
# e.g. http://muddyazian.blogspot.de/2012/04/getting-qt-app-working-with-boost-using.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)
#add_definitions(-DQT_NO_KEYWORDS)

# Define source file
set(${PROJECT_NAME}_SRCS
  src/demo_panel.cpp
  src/demo_widget.cpp
  #src/plugin_init.cpp
)

# Define header file
set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/demo_panel.h
  include/${PROJECT_NAME}/demo_widget.h
)


# Define ui file
set(${PROJECT_NAME}_UIS
  resource/pointcloud_controller.ui
)

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
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")


pluginlib_export_plugin_description_file(rviz_common demo_plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()