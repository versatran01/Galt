cmake_minimum_required(VERSION 2.8.3)
project(pcl_editor)

add_definitions(-Wall -std=c++11 -Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
    roscpp pcl_ros
    dynamic_reconfigure
    )

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

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

add_library(${PROJECT_NAME}
    src/pcl_editor.cpp
    )
target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
    )

add_executable(pcl_merger_node
    src/pcl_merger_main.cpp
    src/pcl_merger_node.cpp
    )
target_link_libraries(pcl_merger_node
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    )

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    )
