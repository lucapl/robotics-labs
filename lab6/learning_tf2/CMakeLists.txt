cmake_minimum_required(VERSION 3.0.2)
project(learning_tf2)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
  #turtlesim
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES learning_tf2
#  CATKIN_DEPENDS roscpp rospy tf2 tf2_ros turtlesim
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   #scripts/turtle_tf2_broadcaster.py scripts/turtle_tf2_listener.py 
   scripts/walking_robot_listener.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)