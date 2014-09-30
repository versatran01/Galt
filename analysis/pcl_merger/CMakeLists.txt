cmake_minimum_required(VERSION 2.8.3)
project(pcl_merger)

add_definitions(-Wall -std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp pcl_ros
    dynamic_reconfigure
    )

generate_dynamic_reconfigure_options(cfg/MergerDyn.cfg)

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES tree_merger
    #  CATKIN_DEPENDS pcl_ros roscpp
    #  DEPENDS system_lib
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_executable(pcl_merger_node
    src/pcl_merger_main.cpp
    src/pcl_merger_node.cpp
    )
target_link_libraries(pcl_merger_node
    ${catkin_LIBRARIES}
    )

add_dependencies(pcl_merger_node
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    )
