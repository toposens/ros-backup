cmake_minimum_required(VERSION 2.8.3)
project(toposens_msgs)

set(${PROJECT_NAME}_DEPS
  geometry_msgs
	message_generation
	message_runtime
	std_msgs
)

find_package(
	catkin REQUIRED
	COMPONENTS ${${PROJECT_NAME}_DEPS}
)

add_message_files(
	DIRECTORY msg
	FILES
	TsPoint.msg
	TsScan.msg
)

generate_messages(
	DEPENDENCIES
	geometry_msgs
	std_msgs
)

catkin_package(
	CATKIN_DEPENDS ${${PROJECT_NAME}_DEPS}
)