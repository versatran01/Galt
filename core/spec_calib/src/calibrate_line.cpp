/*
 * calibrate_line.cpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 28/6/2014
 *      Author: gareth
 */

#include "calibrate_line.hpp"
#include <galt_common/error_handling.hpp>

using namespace Eigen;

std::vector<double> calibrate_line(const std::vector<CalibData>& observations, double reps, double aeps) {
  dbg_assert( observations.size() >= 2 );

  std::vector <cv::Point3d> camPoints;
  camPoints.resize(observations.size());

  //  calculate observation depths
  kr::mat3<double> bRw;
  kr::vec3<double> v,n;
  kr::vec3<double> p0;
  kr::vec3<double> p;

  for (const CalibData& data : observations) {

    //  normalized vector in camera coordinates
    v[0] = data.meas.x;
    v[1] = data.meas.y;
    v[2] = 1.0;
    v /= v.norm();

    //  transform ground plane to camera coordinates
    bRw = data.pose.q.toMatrix();
    n = bRw.block<3,1>(0,2);
    p0 = bRw * -data.pose.p;

    //  vector-plane intersection
    double d = (p0[0]*n[0] + p0[1]*n[1] + p0[2]*n[2]) / (v[0]*n[0] + v[1]*n[1] + v[2]*n[2]);
    p = v * d;

    camPoints.push_back( cv::Point3d(p[0],p[1],p[2]) );
  }

  //  fit to a line with least squares
  std::vector<double> line(6,0.0);
  cv::fitLine(camPoints,line,CV_DIST_L2,0,reps,aeps);
  return line;
}
