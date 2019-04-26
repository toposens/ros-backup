cmake_minimum_required(VERSION 2.8.3)
project(toposens_markers)

set(${PROJECT_NAME}_DEPS
  dynamic_reconfigure
  message_runtime
  roscpp
  rospy
  rviz_visual_tools
  toposens_driver
  toposens_msgs
)

find_package(
  catkin REQUIRED
  COMPONENTS ${${PROJECT_NAME}_DEPS}
)


############
## Config ##
############

generate_dynamic_reconfigure_options(
    cfg/TsMarkers.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_DEPS}
)


###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
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
