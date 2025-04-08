cmake_minimum_required(VERSION 3.0.2)
project(hum_det)

set(CMAKE_CXX_STANDARD 14)
set(SOURCES
	src/utils.cc
	src/inference.cc
)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  roslib
  rospy
  sensor_msgs
  stereo_msgs
  std_msgs
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_message_files(
  FILES
  Box.msg
  Det.msg
  DetArray.msg
)

add_service_files(
  FILES
  ModeSwitch.srv
  StereoMode.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  stereo_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} /opt/intel/openvino/runtime/include
)

add_library(inference_lib
  ${SOURCES}
)

add_executable(yolov10_node src/yolov10_node.cc)

target_link_libraries(inference_lib ${OpenCV_LIBS} /opt/intel/openvino/runtime/lib/intel64/libopenvino.so yaml-cpp)
target_link_libraries(yolov10_node
  inference_lib
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/det_img_group_node.py
  scripts/stereo_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
