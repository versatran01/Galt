cmake_minimum_required(VERSION 2.8.3)
project(iris_tf)

add_definitions("-std=c++11 -Wall")
find_package(catkin REQUIRED COMPONENTS
    tf2_ros
    )

catkin_package(
    #INCLUDE_DIRS include
    #LIBRARIES iris_tf
    CATKIN_DEPENDS tf2_ros
    #DEPENDS system_lib
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )
