cmake_minimum_required(VERSION 3.0.2)
project(community_inspection)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY scripts navigation models insert worlds maps assets weights
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)
install(FILES README.md Dockerfile DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
