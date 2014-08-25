/*
 * visualizer.hpp
 *  _   _             _           _____         _
 * | \ | | ___  _   _| | ____ _  |_   _|__  ___| |__
 * |  \| |/ _ \| | | | |/ / _` |   | |/ _ \/ __| '_ \
 * | |\  | (_) | |_| |   < (_| |   | |  __/ (__| | | |
 * |_| \_|\___/ \__,_|_|\_\__,_|   |_|\___|\___|_| |_|
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of sam_estimator.
 *
 *	Created on: 21/08/2014
 */

#ifndef GALT_SAM_ESTIMATOR_VISUALIZER_HPP
#define GALT_SAM_ESTIMATOR_VISUALIZER_HPP

#include <ros/ros.h>
#include <memory>
#include <sam_estimator/common.hpp>

namespace galt {
namespace sam_estimator {

class Visualizer {
public:
  typedef std::shared_ptr<Visualizer> Ptr;
  
  Visualizer(const ros::NodeHandle& nh);
  
  void SetTrajectory(const std::vector<kr::Posed>& poses);
  
private:
  ros::NodeHandle nh_;
  ros::Publisher pub_path_;
};
}  // namespace sam_estimator
}  // namespace galt

#endif  // GALT_SAM_ESTIMATOR_VISUALIZER_HPP
