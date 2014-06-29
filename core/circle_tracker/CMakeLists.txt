cmake_minimum_required(VERSION 2.8.3)
project(circle_tracker)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_transport
  OpenCV
  roscpp
  std_msgs
  cv_bridge
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# additional warnings
add_definitions("-Wall -Wunused -std=c++11")

# build node
add_executable(${PROJECT_NAME}
  src/circle_tracker_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${roscpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)
