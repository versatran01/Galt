cmake_minimum_required(VERSION 2.8.3)
project(bag_tools)

find_package(catkin REQUIRED COMPONENTS
    rospy rosbag std_msgs geometry_msgs sensor_msgs)

catkin_python_setup()
catkin_package()

if(CATKIN_ENABLE_TESTING)
   catkin_add_nosetests(tests/test_bag_tools.py)
   catkin_add_nosetests(tests/test_utils.py)
endif()

catkin_install_python(PROGRAMS
    scripts/fix_stamp.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})