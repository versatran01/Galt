cmake_minimum_required(VERSION 2.8.3)
project(imu_3dm_gx4)
find_package(catkin REQUIRED COMPONENTS message_generation roscpp geometry_msgs sensor_msgs galt_common)

add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(INCLUDE_DIRS include
               CATKIN_DEPENDS message_runtime roscpp geometry_msgs sensor_msgs galt_common)

# include boost
find_package(Boost REQUIRED)
link_directories(${Boost_LIBRARY_DIR})
include_directories(${Boost_INCLUDE_DIR})

# include headers
include_directories(include)
include_directories(${galt_common_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(${PROJECT_NAME} src/imu_3dm_gx4.cpp src/imu.cpp)
target_link_libraries(${PROJECT_NAME} ${roscpp_LIBRARIES} ${galt_common_LIBRARIES})
