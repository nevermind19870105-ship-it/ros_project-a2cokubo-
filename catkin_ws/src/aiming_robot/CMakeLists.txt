cmake_minimum_required(VERSION 3.0.2)
project(aiming_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  gazebo_msgs
  urdf
  xacro
  gazebo_ros
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge gazebo_msgs urdf xacro gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/vision_marker_tracker.py
  scripts/move_demo_marker.py
  scripts/train_trajectory_predictor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)
