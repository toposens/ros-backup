find_package(roslaunch REQUIRED)
find_package(rostest REQUIRED)

set(TEST_DATA_LOCATION
	http://85.214.248.206/ros
)

add_definitions(
	-Wall
	-O2
	-D_GNU_SOURCE
	-Wno-unused-but-set-variable
	-Wno-unused-result
	-Wno-conversion-null
)

###############
## Emulation ##
###############

add_executable(
	${PROJECT_NAME}_mock
	tty_emulator/tty0tty.c
)

install(
	TARGETS ${PROJECT_NAME}_mock
  RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#@todo can we just execute emulator here directly, have all tests run
# and kill the process later?

###############
## Data Dump ##
###############

if(PLUGGED_IN)
	set(LOG_PERIOD 3s)
	set(PORT /dev/ttyUSB0)

	#@todo for launch file to work, the node executable needs to be first built
	# see if you can issue a post build command instead
	# or make sure catkin_make has been called before the run_tests

	message(STATUS "Extracting data from live stream for ${LOG_PERIOD}")
	execute_process(
		COMMAND timeout ${LOG_PERIOD}
						roslaunch ${PROJECT_NAME} ${PROJECT_NAME}.launch
		COMMAND timeout ${LOG_PERIOD}
						cat ${PORT}
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		OUTPUT_FILE raw_stream
	)
	message(STATUS "Raw stream extraction complete")

endif()


########################
## Download Test Data ##
########################


catkin_download_test_data(
  ${PROJECT_NAME}_raw_stream
  ${TEST_DATA_LOCATION}/raw_stream
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
	MD5 3ec2f56b1cf5372cc98499c4458d37ff
)

catkin_download_test_data(
  ${PROJECT_NAME}_ts_scans.bag
  ${TEST_DATA_LOCATION}/ts_scans.bag
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
	MD5 4abee53fddcf902bab3ca02511ea861b
)


#############
## Modules ##
#############

set(TEST_MODULES
	command
	reconfig
	sensor
	serial
)

foreach(M ${TEST_MODULES})

	add_rostest_gtest(
		${PROJECT_NAME}_${M}_test
		modules/${M}/launch.test
		modules/${M}/tests.cpp
	)

	add_dependencies(
		${PROJECT_NAME}_${M}_test
		${PROJECT_NAME}_mock
		${PROJECT_NAME}_node
	)

	target_link_libraries(
		${PROJECT_NAME}_${M}_test
		${PROJECT_NAME}
	)
endforeach()

#############
## Hz test ##
#############

add_rostest(
	hertz/ts_scans.test
	DEPENDENCIES ${PROJECT_NAME}_node
)

###################
## Code Coverage ##
###################
#include(code_coverage.cmake)
#coverage_add_target(run_tests_${PROJECT_NAME})
