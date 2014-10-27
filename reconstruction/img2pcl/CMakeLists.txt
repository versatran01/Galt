cmake_minimum_required(VERSION 2.8.3)
project(img2pcl)

add_definitions(-std=c++11 -Wall)

find_package(catkin REQUIRED COMPONENTS
    roscpp image_transport sensor_msgs tf2_ros image_geometry pcl_ros
    pcl_conversions eigen_conversions cv_bridge
    )

include_directories(include
    ${catkin_INCLUDE_DIRS}
    )

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES thermal_map
    #  CATKIN_DEPENDS other_catkin_pkg
    #  DEPENDS system_lib
    )

add_executable(${PROJECT_NAME}_node
    src/img2pcl_node.cpp
    src/img2pcl_main.cpp
    )
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    )
