cmake_minimum_required(VERSION 3.0.2)
project(forward_path_follower)

find_package(catkin REQUIRED COMPONENTS
  angles base_local_planner costmap_2d geometry_msgs nav_core nav_msgs
  pluginlib roscpp std_msgs tf2_geometry_msgs tf2_ros)

catkin_package(LIBRARIES forward_path_follower CATKIN_DEPENDS
  angles base_local_planner costmap_2d geometry_msgs nav_core nav_msgs
  pluginlib roscpp std_msgs tf2_geometry_msgs tf2_ros)

include_directories(${catkin_INCLUDE_DIRS})
add_library(forward_path_follower src/forward_path_follower.cpp)
target_link_libraries(forward_path_follower ${catkin_LIBRARIES})
add_dependencies(forward_path_follower ${catkin_EXPORTED_TARGETS})

install(TARGETS forward_path_follower LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
