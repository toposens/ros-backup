#############
## Library ##
#############

add_library(
        ${PROJECT_NAME}
        lib/mapping.cpp
)

add_dependencies(
        ${PROJECT_NAME}
        ${PCL_LIBRARIES} #check if needed
        ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
        ${PROJECT_NAME}
        ${PCL_LIBRARIES} #check if needed
        ${catkin_LIBRARIES}
)

install(
        TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


################
## Executable ##
################

add_executable(
        ${PROJECT_NAME}_node
        ts_cloud_node.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${catkin_LIBRARIES}     #check if needed
        ${PCL_LIBRARIES}        #check if needed
        ${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)
