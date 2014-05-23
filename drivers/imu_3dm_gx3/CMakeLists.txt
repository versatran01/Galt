cmake_minimum_required(VERSION 2.8.3)
project(imu_3dm_gx3)

find_package(catkin REQUIRED roscpp sensor_msgs tf)
find_package(Eigen3 REQUIRED)

catkin_package()

add_executable(imu_3dm_gx3 src/imu_3dm_gx3.cpp)

target_link_libraries(imu_3dm_gx3 ${catkin_LIBRARIES})
