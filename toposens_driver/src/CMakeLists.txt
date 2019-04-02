#############
## Library ##
#############

add_library(
  ${PROJECT_NAME}
  command.cpp
  sensor.cpp
  serial.cpp
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