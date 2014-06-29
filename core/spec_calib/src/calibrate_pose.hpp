/*
 * calibrate_pose.hpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 28/6/2014
 *      Author: gareth
 */

#ifndef CALIBRATE_POSE_HPP_
#define CALIBRATE_POSE_HPP_

#include <vector>
#include <opencv2/opencv.hpp>
#include <pose.hpp> //  kr_math

struct CalibData {
  kr::Pose<double> pose;  /**< Pose of camera during this observation */
  cv::Point2d meas;       /**< Point (in normalized coordinates) of observation */
};

kr::Pose<double> calibrate_pose(std::vector<CalibData> observations);

#endif // CALIBRATE_POSE_HPP_
