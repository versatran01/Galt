/*
 * elliptical_fit.hpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 28/6/2014
 *      Author: gareth
 */

#ifndef ELLIPTICAL_FIT_HPP_
#define ELLIPTICAL_FIT_HPP_

#include <vector>
#include <array>
#include <algorithm>
#include <numeric>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/QR>

template <typename Scalar>
bool elliptical_fit(std::vector<cv::Point_<Scalar>>& points,
                    std::array<Scalar,6>& params,
                    std::vector<Scalar>& residuals,
                    bool leastSquaresEst = false,
                    unsigned int max_iter = 10,
                    Scalar thresh=1e-2) {

  using Eigen::Matrix;
  typedef cv::Point_<Scalar> point_;

  const size_t N = points.size();

  //  parameters of ellipse
  Scalar A,B,C,D,E,F;

  if (!leastSquaresEst) {
    //  simple min-max initial guess
    point_ mean = point_(0,0), max, min;
    min.y = min.x = std::numeric_limits<Scalar>::infinity();
    max.y = max.x = -min.x;

    //  find max,min and mean points
    for (const point_& p : points) {
      max.x = std::max(p.x,max.x);
      max.y = std::max(p.y,max.y);
      min.x = std::min(p.x,min.x);
      min.y = std::min(p.y,min.y);
      mean.x += p.x;
      mean.y += p.y;
    }
    mean.x /= N;
    mean.y /= N;

    //  initial guess
    const Scalar sx = (max.x - min.x) / 2;
    const Scalar sy = (max.y - min.y) / 2;
    const Scalar cx = mean.x;
    const Scalar cy = mean.y;

    //  initial guess
    A = 1.0;
    B = 0.0;
    C = 1.0 * (sx * sx) / (sy * sy);
    D = -2 * cx;
    E = -2 * cy * (sx * sx) / (sy * sy);
    F = (cx*cx) + (cy*cy)*(sx*sx)/(sy*sy) - 1*(sx*sx);
  }
  else {
    //  use linear least squares initial estimate
    //  TODO: check size of data here...

    //  build the S matrix
    Matrix<Scalar,Eigen::Dynamic,6> S(N,6);
    const Scalar sqrt2 = std::sqrt(static_cast<Scalar>(2.0));
    for (size_t i=0; i < N; i++) {
      const Scalar x = points[i].x;
      const Scalar y = points[i].y;

      //  v
      S(i,0) = x;
      S(i,1) = y;
      S(i,2) = 1;

      //  w
      S(i,3) = x*x;
      S(i,4) = sqrt2*x*y;
      S(i,5) = y*y;
    }

    auto QR = S.householderQr();
    Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> matQR = QR.matrixQR();
    assert(matQR.rows() >= 6 || matQR.cols() == 6);

    Matrix<Scalar,3,3> R11 = matQR.template block<3,3>(0,0);
    Matrix<Scalar,3,3> R12 = matQR.template block<3,3>(0,3);
    Matrix<Scalar,3,3> R22 = matQR.template block<3,3>(3,3);

    //  zero under the diagonal to eliminate temporary work values
    //  see documentation of lapack function `geqrf` for reasoning
    for (int i=0; i < 3; i++) {
      for (int j=0; j < 3; j++) {
        if (j < i) {
          R11(i,j) = 0;
          R22(i,j) = 0;
        }
      }
    }

    //  solve for vector w
    auto SVD = R22.jacobiSvd(Eigen::ComputeFullV);
    Matrix<Scalar,3,3> V = SVD.matrixV();
    Matrix<Scalar,3,1> w = V.template block<3,1>(0,2);

    //  solve for vector v
    Matrix <Scalar,3,3> R11inv;
    bool invertible;

    R11.computeInverseWithCheck(R11inv,invertible);
    if (!invertible) {
      //  TODO: deal with this...
    }

    Matrix <Scalar,3,1> v = -R11inv * R12 * w;

    A = w[0];
    B = sqrt2* w[1] / A;
    C = w[2] / A;
    D = v[0] / A;
    E = v[1] / A;
    F = v[2] / A;
    A = 1.0;
  }

  //ROS_INFO("Initial guess: %f, %f, %f, %f, %f, %f", A, B, C, D, E, F);

  //  perform non-linear regression
  Matrix <Scalar,Eigen::Dynamic,5> J(N,5);
  Matrix <Scalar,Eigen::Dynamic,1> r(N,1);
  Matrix <Scalar,5,5> H, Hinv;
  Matrix<Scalar,5,1> delta;

  for (unsigned int iter=0; iter < max_iter;) {
    iter++;

    for (size_t j=0; j < N; j++) {
      const Scalar x = points[j].x;
      const Scalar y = points[j].y;
      const Scalar res = A*x*x + B*x*y + C*y*y + D*x + E*y + F;
      J(j,0) = x*y;
      J(j,1) = y*y;
      J(j,2) = x;
      J(j,3) = y;
      J(j,4) = 1;
      r(j,0) = -res;
    }

    H = J.transpose() * J;
    auto LU = H.fullPivLu();

    if (!LU.isInvertible()) {
      //  failed
      return false;
    }
    Hinv = LU.inverse();
    delta = Hinv * J.transpose() * r;

    B += delta[0];
    C += delta[1];
    D += delta[2];
    E += delta[3];
    F += delta[4];

    if (delta.norm() < thresh) {
      //  todo: early exit
    }
  }

  //ROS_INFO("params: %f,%f,%f,%f,%f,%f", A, B, C, D, E, F);
  //ROS_INFO("res norm: %f", r.norm());

  //  done
  params[0] = A;
  params[1] = B;
  params[2] = C;
  params[3] = D;
  params[4] = E;
  params[5] = F;

  //  residuals
  residuals.resize(N);
  for (size_t i=0; i < N; i++) {
    residuals[i] = r(i,0);
  }
  return true;
}

#endif // ELLIPTICAL_FIT_HPP_
