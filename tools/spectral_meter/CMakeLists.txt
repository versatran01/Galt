cmake_minimum_required(VERSION 2.8.3)
project(spectral_meter)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  sensor_msgs
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(cfg/SpectralMeterDyn.cfg)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs sensor_msgs dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_definitions("-Wall -Werror -std=c++11")

add_executable(${PROJECT_NAME}_node
  src/node.cpp
  src/main.cpp
  )

add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )
