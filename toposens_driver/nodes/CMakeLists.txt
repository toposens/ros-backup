################
## Executable ##
################

add_executable(
  ${PROJECT_NAME}_node
  ts_driver_node.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  COMPONENT main
)
