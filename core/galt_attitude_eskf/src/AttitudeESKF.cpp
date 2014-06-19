/*
 * AttitudeESKF.cpp
 *
 *  Copyright (c) 2013 Gareth Cross. All rights reserved.
 *
 *  This file is part of AttitudeESKF.
 *
 *	Created on: 12/24/2013
 *		Author: gareth
 */

#include "AttitudeESKF.hpp"
#include <error_handling.hpp>

using namespace std;
using namespace Eigen;

//	skew symmetric matrix
template <typename T>
static inline Matrix<T,3,3> cross_skew(const Matrix<T,3,1>& w)
{
    Matrix<T,3,3> W = Matrix<T,3,3>::Zero();
    
    W(0,1) = -w(2);
    W(0,2) = w(1);
   
    W(1,0) = w(2);
    W(1,2) = -w(0);
    
    W(2,0) = -w(1);
    W(2,1) = w(0);
    
    return W;
}

//	hardcoded 3x3 invert (unchecked)
template <typename T>
static inline Matrix<T,3,3> invert(const Matrix<T,3,3>& A, T det)
{
    Matrix<T,3,3> C;
    det = 1.0 / det;
    
    C(0,0) = (-A(2,1)*A(1,2) + A(1,1)*A(2,2)) * det;
    C(0,1) = (-A(0,1)*A(2,2) + A(0,2)*A(2,1)) * det;
    C(0,2) = ( A(0,1)*A(1,2) - A(0,2)*A(1,1)) * det;
    
    C(1,0) = ( A(2,0)*A(1,2) - A(1,0)*A(2,2)) * det;
    C(1,1) = (-A(2,0)*A(0,2) + A(0,0)*A(2,2)) * det;
    C(1,2) = ( A(1,0)*A(0,2) - A(0,0)*A(1,2)) * det;
    
    C(2,0) = (-A(2,0)*A(1,1) + A(1,0)*A(2,1)) * det;
    C(2,1) = ( A(2,0)*A(0,1) - A(0,0)*A(2,1)) * det;
    C(2,2) = (-A(1,0)*A(0,1) + A(0,0)*A(1,1)) * det;
    
    return C;
}

//	hardcoded determinant
template <typename T>
static inline T determinant(const Matrix<T,3,3>& A)
{
    return  A(0,0) * ( A(1,1)*A(2,2) - A(1,2)*A(2,1) ) -
            A(0,1) * ( A(1,0)*A(2,2) - A(1,2)*A(2,0) ) +
            A(0,2) * ( A(1,0)*A(2,1) - A(1,1)*A(2,0) );
}

AttitudeESKF::AttitudeESKF() :  
  q_(), isStable_(true), lastTime_(0.0), steadyCount_(0)
{
  P_.setZero();
  b_.setZero();
  
  estBias_ = true;
  useMag_ = false;
  
  angVel_.setZero();
}

void AttitudeESKF::predict(const Matrix<scalar_t,3,1>& wg, double time)
{
  static const Matrix<scalar_t,3,3> I3 = Matrix<scalar_t,3,3>::Identity(); //  identity R3
  
  scalar_t dt = 0.01;
  if (lastTime_ != 0.0 && time > lastTime_) {
    dt = static_cast<scalar_t>(time - lastTime_);
  }
  lastTime_ = time;  
  angVel_ = wg;

  const Matrix<scalar_t,3,1> wt = (wg - b_);   //	true gyro reading
  
  //	error-state jacobian
  Matrix<scalar_t,6,6> F;
  F.setZero();

  F.block<3,3>(0,0) = I3 - cross_skew<scalar_t>(wt * dt);
  F.block<3,3>(0,3) = I3 * -dt;
  F.block<3,3>(3,3) = I3;
  
  //  integrate nominal state
  q_.integrateRungeKutta4(quat(0.0, wt[0], wt[1], wt[2]), dt);
  
  Matrix<scalar_t,6,6> Q;
  Q.setZero();
  
  const bool rotating = angVel_.norm() > 1e-2;
  for (int i=0; i < 3; i++)
  {
    Q(i,i) = var_.gyro[i];
    Q(i+3,i+3) = (rotating) ? (0.0) : var_.gyroBias[i];
  }
  
  //  integrate covariance  
  P_ = F * P_ * F.transpose() + Q;
}

void AttitudeESKF::update(const Matrix<scalar_t,3,1>& ab)
{   
  Matrix<scalar_t,6,1> dx;  //  error state, we will calculate
  Matrix<scalar_t,6,6> A;   //  for updating covariance
  
  //  rotation matrix: body -> world
  const Matrix<scalar_t,3,3> R = q_.to_matrix().cast<scalar_t>();
  
  Matrix<scalar_t,3,1> gravity;
  gravity[0] = 0.0;
  gravity[1] = 0.0;
  gravity[2] = 1.0;

  //  predicted gravity vector
  const Matrix<scalar_t,3,1> aPred = R.transpose() * gravity;
  
  if (!useMag_)
  {   
    Matrix <scalar_t,3,6> H;
    H.setZero();
    
    Matrix <scalar_t,3,3> R;
    R.setZero();
    for (int i=0; i < 3; i++) {
      R(i,i) = var_.accel[i];
    }
    
    //  calculate gravity component of Jacobian and residual
    H.block<3,3>(0,0) = cross_skew(aPred);
    const Matrix<scalar_t,3,1> r = ab - aPred;
  
    //  solve for the kalman gain using fast 3x3 invert
    Matrix<scalar_t,3,3> S = H * P_ * H.transpose() + R;
    Matrix<scalar_t,3,3> Sinv;
    
    const scalar_t det = determinant(S);
    if (det < 1e-5) {
      isStable_ = false;
      return;
    } else {
      isStable_ = true;
    }
    
    Sinv = invert(S, det);  //  safe to invert
    const Matrix<scalar_t,6,3> K = P_ * H.transpose() * Sinv;
    
    dx = K * r;
    A = K * H;
  }
  else
  {
    Matrix <scalar_t,6,6> H;
    H.setZero();
  }
  
  //  perform state update  
  P_ = (Matrix<scalar_t,6,6>::Identity() - A) * P_;
  
  q_ = q_ * quat(1.0, dx[0], dx[1], dx[2]);
  q_ /= q_.norm();

  if (angVel_.norm() < 1.0e-2) {  //  angular velocity is small
    steadyCount_++;
  } else {
    steadyCount_ = 0;
  }
  
  if (estBias_ && updateCount_ > 10) //  don't estimate bias on first few updates
  {
    if (steadyCount_ > 10) {
      for (int i=0; i < 3; i++) {
          b_[i] += dx[i+3];
      }
    }
  }
  else {
    b_.setZero();
  }
  
  updateCount_++;
}


//  (world) = Rz * Ry * Rx (body)
Eigen::Matrix<double,3,1> AttitudeESKF::getRPY() const
{	
	const Matrix<double,3,3> R = q_.to_matrix().cast<double>();
	Matrix<double,3,1> rpy;

	double sth = -R(2,0);
	if (sth > 1.0f) {
		sth = 1.0f;
	} else if (sth < -1.0f) {
		sth = -1.0f;
	}

	const double theta = std::asin(sth);
	const double cth = std::sqrt(1.0f - sth*sth);

	double phi, psi;
	if (cth < 1e-6f)
	{
		phi = std::atan2(R(0,1), R(1,1));
		psi = 0.0f;
	}
	else
	{
		phi = std::atan2(R(2,1), R(2,2));
		psi = std::atan2(R(1,0), R(0,0));
	}

	rpy[0] = phi;   //  x
	rpy[1] = theta; //  y
	rpy[2] = psi;   //  z
	return rpy;
}
