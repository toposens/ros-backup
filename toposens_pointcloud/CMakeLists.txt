cmake_minimum_required(VERSION 2.8.3)
project(toposens_pointcloud)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  pcl_ros
  roscpp
  rospy
  tf
  toposens_driver
  toposens_msgs
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
#generate_dynamic_reconfigure_options(
# cfg/TsMarkers.cfg
#)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  dynamic_reconfigure
  message_runtime
  pcl_ros
  roscpp
  rospy
  tf
  toposens_driver
  toposens_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_node
  src/cloud.cpp
  src/ts_cloud_node.cpp)

target_link_libraries(
  ${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


install(
  TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  COMPONENT main
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
