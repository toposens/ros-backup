### Unit tests
#
#   Only configured when CATKIN_ENABLE_TESTING is true.

# these dependencies are only needed for unit testing

find_package(roslaunch REQUIRED)
find_package(rostest REQUIRED)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O2 -D_GNU_SOURCE -Wno-unused-but-set-variable")
add_definitions(-Wall -O2 -D_GNU_SOURCE -Wno-unused-but-set-variable)

add_executable(
	${PROJECT_NAME}_mock
	tty_emulator/tty0tty.c
)

install(
	TARGETS ${PROJECT_NAME}_mock
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

################
## Unit Tests ##
################

#add_rostest_gtest(${PROJECT_NAME}_command_test unit/command_node.test unit/command_test.cpp)
#add_dependencies(${PROJECT_NAME}_command_test ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_command_test ${catkin_LIBRARIES} ${PROJECT_NAME})
#
#add_rostest_gtest(${PROJECT_NAME}_sensor_test unit/sensor_node.test unit/sensor_test.cpp)
#add_dependencies(${PROJECT_NAME}_sensor_test ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_sensor_test ${catkin_LIBRARIES} ${PROJECT_NAME})

add_rostest_gtest(
	${PROJECT_NAME}_serial_test
	unit/serial_node.test
	unit/serial_test.cpp
)

add_dependencies(
	${PROJECT_NAME}_serial_test
	${catkin_EXPORTED_TARGETS}
	${PROJECT_NAME}_mock
)

target_link_libraries(
	${PROJECT_NAME}_serial_test
	${catkin_LIBRARIES}
	${PROJECT_NAME}
)

#######################
## Integration Tests ##
#######################
#add_rostest_gtest(${PROJECT_NAME}_param_test integration/param_node.test integration/param_test.cpp)
#add_dependencies(${PROJECT_NAME}_param_test ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_param_test ${catkin_LIBRARIES} ${PROJECT_NAME})


###################
## Code Coverage ##
###################
#include(code_coverage.cmake)
#coverage_add_target(run_tests_${PROJECT_NAME})