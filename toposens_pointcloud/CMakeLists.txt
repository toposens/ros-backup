cmake_minimum_required(VERSION 2.8.3)
project(toposens_pointcloud)

set(${PROJECT_NAME}_DEPS
  message_runtime
  pcl_ros
  roscpp
  rospy
  rviz
  tf
  toposens_driver
  toposens_msgs
)

find_package(
  catkin REQUIRED
  COMPONENTS ${${PROJECT_NAME}_DEPS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_DEPS}
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

##########
## Test ##
##########
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
