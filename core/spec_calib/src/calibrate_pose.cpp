/*
 * calibrate_pose.cpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 28/6/2014
 *      Author: gareth
 */

#include "calibrate_pose.hpp"
#include <galt_common/error_handling.hpp>

std::array<double, 6> calibrate_pose(const std::vector<CalibData>& observations)
{
  dbg_assert( observations.size() >= 2 );

  std::vector <cv::Point2d> camPoints;

  //  calculate depths

}
