cmake_minimum_required(VERSION 2.8.3)
project(bluefox2)

find_package(catkin REQUIRED roscpp sensor_msgs camera_calibration_parsers)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

set(MVIMPACT_DIR "${PROJECT_SOURCE_DIR}/mvIMPACT") # 2.7.2
include_directories(${MVIMPACT_DIR})

add_executable(bluefox2 src/camera_node.cpp src/camera.cpp)
target_link_libraries(bluefox2
    ${catkin_LIBRARIES}
    ${MVIMPACT_DIR}/lib/x86_64/libmvDeviceManager.so)
