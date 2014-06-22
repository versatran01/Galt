cmake_minimum_required(VERSION 2.8.3)
project(galt_attitude_eskf)
find_package(catkin REQUIRED COMPONENTS roscpp tf geometry_msgs sensor_msgs message_filters galt_common)

catkin_package(CATKIN_DEPENDS roscpp tf geometry_msgs sensor_msgs message_filters galt_common)

# include boost
find_package(Boost REQUIRED)
link_directories(${Boost_LIBRARY_DIR})
include_directories(${Boost_INCLUDE_DIR})

# include EIGEN
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIR})

# include headers
include_directories(${galt_common_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

add_executable(${PROJECT_NAME} src/AttitudeESKF.cpp src/galt_attitude_eskf.cpp)
target_link_libraries(${PROJECT_NAME} ${roscpp_LIBRARIES} ${galt_common_LIBRARIES} ${tf_LIBRARIES} ${message_filters_LIBRARIES})

#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
