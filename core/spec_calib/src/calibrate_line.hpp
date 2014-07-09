/*
 * calibrate_line.hpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 28/6/2014
 *      Author: gareth
 */

#ifndef CALIBRATE_LINE_HPP_
#define CALIBRATE_LINE_HPP_

#include <vector>
#include <array>

#include <opencv2/opencv.hpp>
#include <kr_math/pose.hpp> //  kr_math

struct CalibData {
  kr::Pose<double> pose;  /**< Pose of camera during this observation */
  cv::Point2d meas;       /**< Point of observation, after camera matrix has been removed */
};

/**
 * @brief calibrate_line Determine the line equation (camera coordinates) of the spectrometer FOV.
 * @param observations Vector of observations.
 * @param reps See cv::fitLine
 * @param aeps See cv::fitLine
 * @return Vector of 6 doubles: [vx,vy,vz,x0,y0,z0] where v is parallel to the line and |v| = 1.
 */
std::vector<double> calibrate_line(const std::vector<CalibData>& observations, double reps=0.01, double aeps=0.01);

#endif // CALIBRATE_LINE_HPP_
