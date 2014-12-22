cmake_minimum_required(VERSION 2.8.3)
project(odom2tf)

find_package(catkin REQUIRED COMPONENTS
    nav_msgs tf2_ros
    )

catkin_package()

include_directories(include
    ${catkin_INCLUDE_DIRS}
    )

add_executable(${PROJECT_NAME}_node src/odom2tf_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
