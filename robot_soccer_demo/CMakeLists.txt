cmake_minimum_required(VERSION 3.0.2)
project(robot_soccer_demo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  moveit_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   nav_msgs
)

#roslaunch_add_file_check(launch)

#add_message_files(DIRECTORY msg)

#generate_messages(
#  DEPENDENCIES
#  geometry_msgs
#  moveit_msgs
#  nav_msgs
#  roscpp
#  rospy
#  sensor_msgs
#  std_msgs
#  trajectory_msgs
#)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/kafka_consumer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})






