cmake_minimum_required(VERSION 3.0.2)
project(sar)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  RescueModeSwitch.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories()

catkin_install_python(PROGRAMS
  scripts/search_node.py
  scripts/rescue_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
