cmake_minimum_required(VERSION 2.8.3)
project(img2pcl)

add_definitions(-std=c++11 -Wall)

find_package(cmake_modules)
find_package(Eigen)
find_package(catkin REQUIRED COMPONENTS
    roscpp image_transport sensor_msgs nav_msgs cv_bridge
    std_msgs geometry_msgs tf
    message_filters laser_geometry image_geometry
    )

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    )

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES thermal_map
    #  CATKIN_DEPENDS other_catkin_pkg
    #  DEPENDS system_lib
    )

