cmake_minimum_required(VERSION 2.8.3)
project(fruit_tracker)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  kr_math
  kr_vision
  message_filters
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/node.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
