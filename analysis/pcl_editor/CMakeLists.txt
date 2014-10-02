cmake_minimum_required(VERSION 2.8.3)
project(pcl_editor)

add_definitions(-Wall -Wno-deprecated -Wl,--as-needed)

find_package(catkin REQUIRED COMPONENTS
    roscpp pcl_ros
    dynamic_reconfigure
    )

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

generate_dynamic_reconfigure_options(
    cfg/MergerDyn.cfg
    cfg/CropperDyn.cfg
    )

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
    ${PCL_LIBRARIES}
    )

add_executable(pcl_cropper_node
    src/pcl_cropper_main.cpp
    src/pcl_cropper_node.cpp
    )
target_link_libraries(pcl_cropper_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    )

add_dependencies(pcl_merger_node
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    )
add_dependencies(pcl_cropper_node
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    )
