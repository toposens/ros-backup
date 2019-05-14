 find_package(roslaunch REQUIRED)
 find_package(rostest REQUIRED)

#############
## Modules ##
#############

set(TEST_MODULES
	mapping
)

foreach(M ${TEST_MODULES})

	add_rostest_gtest(
		${PROJECT_NAME}_${M}_test
		modules/${M}/launch.test
		modules/${M}/tests.cpp
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
	hertz/ts_cloud.test
	DEPENDENCIES ${PROJECT_NAME}_node
)
