cmake_minimum_required(VERSION 2.8.3)
project(iris_tf)

add_definitions("-std=c++11 -Wall")
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2 tf2_ros
    geometry_msgs visualization_msgs
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES iris_tf
    CATKIN_DEPENDS roscpp tf2 tf2_ros geometry_msgs visualization_msgs
    DEPENDS system_lib
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_executable(${PROJECT_NAME}_node
    src/iris_tf_node.cpp
    src/rviz_helper.cpp
    )

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    )
