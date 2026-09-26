cmake_minimum_required(VERSION 3.0.2)
project(safe_escape_recovery)

find_package(catkin REQUIRED COMPONENTS
  roscpp nav_core costmap_2d base_local_planner geometry_msgs nav_msgs pluginlib tf2_ros)

catkin_package(LIBRARIES safe_escape_recovery CATKIN_DEPENDS
  roscpp nav_core costmap_2d base_local_planner geometry_msgs nav_msgs pluginlib tf2_ros)

include_directories(${catkin_INCLUDE_DIRS})
add_library(safe_escape_recovery src/safe_escape_recovery.cpp)
target_link_libraries(safe_escape_recovery ${catkin_LIBRARIES})

install(TARGETS safe_escape_recovery LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
