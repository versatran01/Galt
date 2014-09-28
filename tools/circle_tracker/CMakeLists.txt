cmake_minimum_required(VERSION 2.8.3)
project(circle_tracker)

find_package(OpenCV REQUIRED)
find_package(cmake_modules)
find_package(Eigen)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_transport
  roscpp
  std_msgs
  cv_bridge
  message_generation
  dynamic_reconfigure
)

# generate reconfigure options
generate_dynamic_reconfigure_options(cfg/CircleTracker.cfg)

# generate messages
add_message_files(FILES Circle.msg Circles.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

# additional warnings
add_definitions("-Wall -Wunused -std=c++11")

# build node
add_executable(${PROJECT_NAME}
  src/circle_tracker_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)

# ensures messages are generated before hand
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencf
)
