cmake_minimum_required(VERSION 3.0.2)
project(building_map)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_ros
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  message_generation
)

find_package(OpenCV REQUIRED)

add_service_files(
  FILES
  CleanedMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

catkin_package(
 CATKIN_DEPENDS cv_bridge gazebo_ros image_transport roscpp rospy sensor_msgs std_msgs nav_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(map_cleaner_node src/map_cleaner_node.cpp)
add_executable(map_transfer_node src/map_transfer_node.cpp)

add_dependencies(map_cleaner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(map_transfer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(map_cleaner_node
  ${catkin_LIBRARIES}
)
target_link_libraries(map_transfer_node
  ${catkin_LIBRARIES}
)
