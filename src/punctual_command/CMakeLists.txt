cmake_minimum_required(VERSION 2.8.3)
project(punctual_command)

find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


generate_dynamic_reconfigure_options(
  cfg/PunctualCommand.cfg
)


catkin_package(INCLUDE_DIRS
    LIBRARIES punctual_command
    CATKIN_DEPENDS
        dynamic_reconfigure
        roscpp)

add_executable(punctual_command_node src/punctual_command_node.cpp)

add_dependencies(punctual_command_node ${PROJECT_NAME}_gencfg)

target_link_libraries(punctual_command_node ${catkin_LIBRARIES})

install(TARGETS punctual_command_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
