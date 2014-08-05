cmake_minimum_required(VERSION 2.8.3)
project(stereo_vo)

set(CMAKE_CXX_FLAGS "-Wall -std=c++11 -fPIC")

find_package(catkin REQUIRED COMPONENTS
    roscpp nodelet
    image_transport message_filters
    sensor_msgs
    cv_bridge
)

find_package(OpenCV)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stereo_vo
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
    src/stereo_vo_node.cc
    src/main.cc
)
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
