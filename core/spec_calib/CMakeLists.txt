cmake_minimum_required(VERSION 2.8.3)
project(spec_calib)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ocean_optics
  kr_math
  roscpp
  std_msgs
  OpenCV
)

catkin_package()

# include headers
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${kr_math_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# additional warnings
add_definitions("-Wall -Wunused")

# build node
add_executable(${PROJECT_NAME}
  src/spec_calib_node.cpp
  src/calibrate_pose.cpp
)
target_link_libraries(${PROJECT_NAME} ${roscpp_LIBRARIES} ${OpenCV_LIBRARIES})

# depends on messages from spectrometer
add_dependencies(${PROJECT_NAME} ${ocean_optics_EXPORTED_TARGETS})
