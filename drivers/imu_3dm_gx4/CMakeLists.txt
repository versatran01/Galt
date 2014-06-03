cmake_minimum_required(VERSION 2.8.3)
project(imu_3dm_gx4)
find_package(catkin REQUIRED)

catkin_package()

find_package(Boost REQUIRED)
link_directories(${Boost_LIBRARY_DIR})
include_directories(${Boost_INCLUDE_DIR})

add_executable(${PROJECT_NAME} src/imu_3dm_gx4.cpp)
target_link_libraries(${PROJECT_NAME} ${roscpp_LIBRARIES})
