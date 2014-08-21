/*
 * main.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of sam_estimator.
 *
 *	Created on: 21/08/2014
 */

#include <sam_estimator/sam_estimator_node.hpp>

int main(int argc, char ** argv) {
  ros::init(argc, argv, "sam_estimator");
  ros::NodeHandle nh("~");
  galt::sam_estimator::SamEstimatorNode node(nh);
  ros::spin();
  return 0;
}
