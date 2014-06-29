/*
 * spec_calib_node.cpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 28/6/2014
 *      Author: gareth
 */

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_msgs/Header.h>

#include <Eigen/Core>
#include <Eigen/Dense>

// spectrometer output
#include <ocean_optics/Spectrum.h>

// kr_math
#include <quaternion.hpp>
#include <SO3.hpp>

ros::NodeHandlePtr nh;

int main(int argc, char ** argv) {
  ros::init(argc,argv,"spec_calib_node");
  nh = ros::NodeHandlePtr( new ros::NodeHandle("~") );  
  
  ros::spin();
  return 0;
}
