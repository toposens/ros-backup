#############
## Library ##
#############

add_library(
  ${PROJECT_NAME}
  lib/command.cpp
  lib/sensor.cpp
  lib/serial.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
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
  ts_driver_node.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  COMPONENT main
)
