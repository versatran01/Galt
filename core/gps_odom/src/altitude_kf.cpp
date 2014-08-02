/*
 * altitude_kf.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of gps_odom.
 *
 *	Created on: 31/07/2014
 *		  Author: gareth
 */

#include <gps_odom/altitude_kf.hpp>

namespace gps_odom {

AltitudeKF::AltitudeKF(scalar_t posVariance, scalar_t velVariance) {
  x_.setZero();
  
  P_(0,0) = posVariance*posVariance;
  P_(0,1) = 0;
  P_(1,0) = 0;
  P_(1,1) = velVariance*velVariance;
  
  stable_ = true;
}

void AltitudeKF::predict(scalar_t accZ, scalar_t accVariance, scalar_t dt) {
  
  //  integrate state
  x_[0] += x_[1] * dt;
  x_[1] += accZ * dt;
  
  //  integrate covariance
  kr::mat2<scalar_t> A;
  A(0,0) = 1;
  A(0,1) = dt;
  A(1,0) = 0;
  A(1,1) = 1;
  
  kr::mat2<scalar_t> Q;
  Q.setZero();
  Q(1,1) = accVariance*accVariance;
  
  P_ = A*P_*A.transpose() + Q;
}

void AltitudeKF::updateHeightAndVelocity(scalar_t z, scalar_t zVariance, 
                              scalar_t velZ, scalar_t velVariance) {
  
  //  measurement jacobian is identity in this simple model
  kr::mat2<scalar_t> R;
  R(0,0) = zVariance*zVariance;
  R(0,1) = 0;
  R(1,0) = 0;
  R(1,1) = velVariance*velVariance;
  
  //  kalman gain
  kr::mat2<scalar_t> S = P_ + R;
  kr::mat2<scalar_t> Sinv;
  S.computeInverseWithCheck(Sinv, stable_);
  if (!stable_) {
    return;
  }
  
  //  state update
  kr::mat2<scalar_t> K = P_* Sinv;
  kr::vec2<scalar_t> meas = kr::vec2<scalar_t>(z,velZ);
  x_ = x_ + K * (meas - x_);
  
  //  covariance update
  P_ = (kr::mat2<scalar_t>::Identity() - K) * P_;
}

void AltitudeKF::updateHeight(scalar_t z, scalar_t zVariance) {
  //  measurement jacobian
  kr::mat<scalar_t,1,2> H;
  H(0,0) = 1;
  H(0,1) = 0;
  
  //  kalman gain
  scalar_t S = H*P_*H.transpose() + zVariance*zVariance;
  if (std::abs(S) < std::numeric_limits<scalar_t>::epsilon()) {
    stable_ = false;
    return;
  }
  
  kr::vec2<scalar_t> K = P_ * H.transpose() * (1 / S);
  
  //  state update
  x_ += K*(z - H*x_);
  P_ = (kr::mat2<scalar_t>::Identity() - K*H) * P_;
}

AltitudeKF::scalar_t AltitudeKF::getHeight() const {
  return x_[0];
}

}
