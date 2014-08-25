cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_ukf)

find_package(catkin REQUIRED COMPONENTS
  kr_math
  nav_msgs
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/quadrotor_ukf.cpp
  src/quadrotor_ukf_node.cpp
)

add_definitions("-O3")

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
