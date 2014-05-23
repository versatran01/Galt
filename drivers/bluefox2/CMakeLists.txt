cmake_minimum_required(VERSION 2.8.3)
project(bluefox2)

find_package(catkin REQUIRED roscpp sensor_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
message(STATUS "CATKIN_DIR: " ${catkin_INCLUDE_DIRS})

set(MVIMPACT_DIR "${PROJECT_SOURCE_DIR}/mvIMPACT_acquire-x86_64-2.7.1")
include_directories(${MVIMPACT_DIR})
message(STATUS "MVIMPACT_DIR: " ${MVIMPACT_DIR})

add_executable(bluefox2 src/camera_node.cpp src/camera.cpp)
target_link_libraries(bluefox2
    ${catkin_LIBRARIES}
    ${MVIMPACT_DIR}/lib/x86_64/libmvDeviceManager.so)
