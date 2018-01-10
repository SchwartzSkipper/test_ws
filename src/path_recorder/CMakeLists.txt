cmake_minimum_required(VERSION 2.8.3)
project(path_recorder)

include_directories(
  "${PROJECT_SOURCE_DIR}/include"
)

find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


generate_dynamic_reconfigure_options(
  cfg/PathPlayer.cfg
)

catkin_package()

add_executable(path_recorder_node 
	src/path_recorder_node.cpp
)
target_link_libraries(path_recorder_node 
${catkin_LIBRARIES})

add_executable(path_player_node src/path_player_node.cpp)
target_link_libraries(path_player_node ${catkin_LIBRARIES})

install(TARGETS path_recorder_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
