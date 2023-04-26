cmake_minimum_required(VERSION 3.0.2)
project(labrob_detector)

find_package(catkin REQUIRED COMPONENTS
  OpenCV
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
  vision_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/yolo_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)