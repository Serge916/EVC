cmake_minimum_required(VERSION 3.0.2)
project(jetson_camera)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
)

# Declare a catkin package
catkin_package()

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  src/camera_publisher_node.py
  src/camera_subscriber_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)