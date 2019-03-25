### Unit tests
#
#   Only configured when CATKIN_ENABLE_TESTING is true.

# these dependencies are only needed for unit testing

find_package(roslaunch REQUIRED)
find_package(rostest REQUIRED)
#find_package(tf2_ros REQUIRED)
find_package(code_coverage REQUIRED)

#if(ENABLE_COVERAGE_TESTING)
#    include(CodeCoverage)
#    APPEND_COVERAGE_COMPILER_FLAGS()
#endif()

################
## Unit Tests ##
################
add_rostest_gtest(${PROJECT_NAME}_command_test unit/command_node.test unit/command_test.cpp)
add_dependencies(${PROJECT_NAME}_command_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_command_test ${catkin_LIBRARIES} ${PROJECT_NAME})

add_rostest_gtest(${PROJECT_NAME}_sensor_test unit/sensor_node.test unit/sensor_test.cpp)
add_dependencies(${PROJECT_NAME}_sensor_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_sensor_test ${catkin_LIBRARIES} ${PROJECT_NAME})

add_rostest_gtest(${PROJECT_NAME}_serial_test unit/serial_node.test unit/serial_test.cpp)
add_dependencies(${PROJECT_NAME}_serial_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_serial_test ${catkin_LIBRARIES} ${PROJECT_NAME})

#######################
## Integration Tests ##
#######################
#add_rostest_gtest(${PROJECT_NAME}_integration_test integration/toposens_driver_integration.test integration/param_test.cpp)
#add_dependencies(${PROJECT_NAME}_integration_test ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_integration_test ${catkin_LIBRARIES} ${PROJECT_NAME})

##########################
## Code Coverage Report ##
##########################
#if(ENABLE_COVERAGE_TESTING)
#    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
#    add_code_coverage(
#            NAME ${PROJECT_NAME}_coverage
#            DEPENDS tests
#    )
#endif()