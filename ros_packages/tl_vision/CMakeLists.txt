cmake_minimum_required(VERSION 3.0.2)
project(tl_vision)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

# 让 src/tl_vision 成为可被任意节点 import 的 Python 包
catkin_python_setup()

add_message_files(
  FILES
  Detection.msg
  DetectionArray.msg
  PlateCrop.msg
  PlateText.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/tl_vision_node
  scripts/tl_vision_ocr_node
  scripts/tl_vision_offline
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test)
endif()
