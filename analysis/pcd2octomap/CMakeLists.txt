cmake_minimum_required(VERSION 2.8.3)
project(pcd2octomap)

find_package(catkin REQUIRED COMPONENTS
  octomap_ros
  octomap_msgs
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf2
)

catkin_package()

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/pcd2octomap.cpp)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
