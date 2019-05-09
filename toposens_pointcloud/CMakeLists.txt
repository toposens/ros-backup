cmake_minimum_required(VERSION 2.8.3)
project(toposens_pointcloud)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    pcl_ros
    roscpp
    rospy
    tf
    rviz
    toposens_driver
    toposens_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  tf
  message_runtime
  pcl_ros
  roscpp
  rospy
  rviz
  toposens_driver
  toposens_msgs
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
