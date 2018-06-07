cmake_minimum_required(VERSION 2.8.3)
project(local_pose_talker)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            tf
        )

find_package(Eigen3 REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES local_pose_talker
    CATKIN_DEPENDS
        roscpp
)
link_directories(${EIGEN3_LIBRARY_DIRS})
catkin_package(
    CATKIN_DEPENDS
        roscpp
)

add_executable(local_pose_talker_node 
	src/local_pose_talker_node.cpp
)
add_dependencies(local_pose_talker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_pose_talker_node 
${catkin_LIBRARIES} /opt/ros/indigo/lib/libeigen_conversions.so)

install(TARGETS local_pose_talker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)