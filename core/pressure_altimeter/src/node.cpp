/*
 * node.cpp
 *  _   _             _           _____         _
 * | \ | | ___  _   _| | ____ _  |_   _|__  ___| |__
 * |  \| |/ _ \| | | | |/ / _` |   | |/ _ \/ __| '_ \
 * | |\  | (_) | |_| |   < (_| |   | |  __/ (__| | | |
 * |_| \_|\___/ \__,_|_|\_\__,_|   |_|\___|\___|_| |_|
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of pressure_altimeter.
 *
 *	Created on: 24/08/2014
 */

#include <pressure_altimeter/node.hpp>
#include <pressure_altimeter/Height.h>  //  published message

namespace galt {
namespace pressure_altimeter {

Node::Node(const ros::NodeHandle &nh) : nh_(nh) {
  subPressure_ = nh_.subscribe("fluid_pressure", 1, 
                              &Node::pressureCallback, this);
  pubHeight_ = nh_.advertise<::pressure_altimeter::Height>("height", 1);
}

void Node::pressureCallback(const sensor_msgs::FluidPressure& pressure) {
  ::pressure_altimeter::Height height_msg;
  height_msg.header.stamp = pressure.header.stamp;
  
  //  calculate height in meters
  
}

} //  pressure_altimeter
} //  galt
