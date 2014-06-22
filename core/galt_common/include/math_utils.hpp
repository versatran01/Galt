/*
 * math_utils.hpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 22/6/2014
 *	  	Author: gareth
 */

#ifndef MATHUTILS_H_
#define MATHUTILS_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

namespace galt
{

/**
 *  @brief getRPY Get the roll,pitch,yaw angles from a rotation matrix
 *  @param R Member of SO(3)
 *  @return 3x1 Vector with elements [roll,pitch,yaw] about [x,y,z] axes
 *
 *  @note Assumes matrix is of the form: (world) = Rz * Ry * Rx (body)
 */
template <typename T>
Eigen::Matrix<T,3,1> getRPY(const Eigen::Matrix<T,3,3>& R)
{
	Eigen::Matrix<T,3,1> rpy;

	T sth = -R(2,0);
	if (sth > 1.0) {
		sth = 1.0;
	} else if (sth < -1.0) {
		sth = -1.0;
	}

	const T theta = std::asin(sth);
	const T cth = std::sqrt(1.0 - sth*sth);

	T phi, psi;
	if (cth < std::numeric_limits<T>::epsilon()*10.0)
	{
		phi = std::atan2(R(0,1), R(1,1));
		psi = 0.0;
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

/**
 *  @brief X rotation matrix
 *  @param angle Angle in radians
 *  @return 3x3 member of SO(3)
 */
template <typename T>
Eigen::Matrix<T,3,3> rotation_x(T angle)
{
  const T c = std::cos(angle);
  const T s = std::sin(angle);

  Eigen::Matrix<T,3,3> R;
  R.setZero();

  R(0,0) = 1.0;
  R(1,1) = c; R(1,2) = -s;
  R(2,1) = s; R(2,2) = c;

  return R;
}

/**
 *  @brief Y rotation matrix
 *  @param angle Angle in radians
 *  @return 3x3 member of SO(3)
 */
template <typename T>
Eigen::Matrix<T,3,3> rotation_y(T angle)
{
  const T c = std::cos(angle);
  const T s = std::sin(angle);

  Eigen::Matrix<T,3,3> R;
  R.setZero();

  R(1,1) = 1.0;
  R(0,0) = c;  R(0,2) = s;
  R(2,0) = -s; R(2,2) = c;

  return R;
}

/**
 *  @brief Z rotation matrix
 *  @param angle Angle in radians
 *  @return 3x3 member of SO(3)
 */
template <typename T>
Eigen::Matrix<T,3,3> rotation_z(T angle)
{
  const T c = std::cos(angle);
  const T s = std::sin(angle);

  Eigen::Matrix<T,3,3> R;
  R.setZero();

  R(2,2) = 1.0;
  R(0,0) = c;  R(0,1) = -s;
  R(1,0) = s;  R(1,1) = c;

  return R;
}

}  // namespace galt

#endif // MATHUTILS_H_
