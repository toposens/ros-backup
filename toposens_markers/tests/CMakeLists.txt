find_package(roslaunch REQUIRED)
find_package(rostest REQUIRED)

#############
## Modules ##
#############

set(TEST_MODULES
	plot
	reconfig
)

foreach(M ${TEST_MODULES})

	add_rostest_gtest(
		${PROJECT_NAME}_${M}_test
		modules/${M}/launch.test
		modules/${M}/tests.cpp
	)

	add_dependencies(
		${PROJECT_NAME}_${M}_test
		${PROJECT_NAME}_node
	)

	target_link_libraries(
		${PROJECT_NAME}_${M}_test
    ${PROJECT_NAME}
	)

endforeach()

add_rostest(
	hertz/ts_markers.test
	DEPENDENCIES ${PROJECT_NAME}_node
)
