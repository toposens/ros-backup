cmake_minimum_required(VERSION 2.8.3)
project(toposens_markers)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  catkin REQUIRED
  COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  rviz_visual_tools
  toposens_driver
  toposens_msgs
)


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
generate_dynamic_reconfigure_options(
 cfg/TsMarkers.cfg
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  dynamic_reconfigure
  message_runtime
  roscpp
  rospy
  rviz_visual_tools
  toposens_driver
  toposens_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)


add_executable(
  ${PROJECT_NAME}_node
  src/markers.cpp
  src/ts_markers_node.cpp
)

add_dependencies(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)


install(
  TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  COMPONENT main
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
