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

using namespace std;
using namespace Eigen;

//	skew symmetric matrix
static inline Matrix<float,3,3> cross_skew(const Matrix<float,3,1>& w)
{
    Matrix<float,3,3> W = Matrix<float,3,3>::Zero();
    
    W(0,1) = -w(2);
    W(0,2) = w(1);
    
    W(1,0) = w(2);
    W(1,2) = -w(0);
    
    W(2,0) = -w(1);
    W(2,1) = w(0);
    
    return W;
}

//	hardcoded 3x3 invert (unchecked)
static inline Matrix<float,3,3> invert(const Matrix<float,3,3>& A, float det)
{
    Matrix<float,3,3> C;
    det = 1.0f / det;
    
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
static inline float determinant(const Matrix<float,3,3>& A)
{
    return  A(0,0) * ( A(1,1)*A(2,2) - A(1,2)*A(2,1) ) -
            A(0,1) * ( A(1,0)*A(2,2) - A(1,2)*A(2,0) ) +
            A(0,2) * ( A(1,0)*A(2,1) - A(1,1)*A(2,0) );
}

AttitudeESKF::AttitudeESKF(float varAccel, float varGyro, float varGyroBias) :  m_q(), m_isStable(true)
{
	m_Q.setIdentity();
	m_R.setIdentity();
	m_P.setZero();

	//	gyro
	m_Q.block<3,3>(0,0) *= varGyro;

	//	gyro bias
	m_Q.block<3,3>(3,3) *= varGyroBias;

	//	accelerometer
	m_R *= varAccel;
}

void AttitudeESKF::predict(const Matrix<float,3,1>& wg, float dt)
{
    static const Matrix<float,3,3> I3 = Matrix<float,3,3>::Identity();
    const Matrix<float,3,1> wt = (wg - m_b);	//	true gyro reading
    
    //	error-state jacobian
    Matrix<float,6,6> F;
    F.setZero();

    F.block<3,3>(0,0) = I3 - cross_skew(wt * dt);
    F.block<3,3>(0,3) = I3 * -dt;
    F.block<3,3>(3,3) = I3;
    
    //  integrate nominal state
    m_q.integrateRungeKutta4(quat(0, wt[0], wt[1], wt[2]), dt);
    
    //  integrate covariance
    m_P = F * m_P * F.transpose() + m_Q;
}

void AttitudeESKF::update(const Matrix<float,3,1>& ab)
{
    //  Jacobian of h(x,dx) w.r.t. dx
    Matrix <float,3,6> H;
    H.setZero();
    
    //  rotation matrix: body -> world
    const Matrix<float,3,3> R = m_q.to_matrix();

    //  normalized measurement & prediction
    Matrix<float,3,1> a = ab;
    float anorm = a.norm();
    if (anorm > 1e-6f) {
    	a /= anorm;
    }

    Matrix<float,3,1> gravity;
    gravity[0] = 0.0f;
    gravity[1] = 0.0f;
    gravity[2] = 1.0f;

    const Matrix<float,3,1> aPred = R.transpose() * gravity;
        
    //  calculate gravity component of Jacobian and residual
    H.block<3,3>(0,0) = cross_skew(aPred);
    const Matrix<float,6,3> Ht = H.transpose();

    Matrix<float,3,1> r = a - aPred;
 
    Matrix<float,3,3> S = H * m_P * Ht + m_R;
    const float det = determinant(S);

    if (std::abs(det) <= 1e-6f) {	//	approaching machine epsilon
    	m_isStable = false;
    	return;
    } else {
    	m_isStable = true;
    }

    //	invert
    S = invert(S,det);

    //  calculate kalman gain and error
    Matrix<float,6,3> K = m_P * Ht * S;
    Matrix<float,6,1> dx = K * r;
    
    //  covariance update
    m_P = (Matrix<float,6,6>::Identity() - K * H) * m_P;

    //  state update
    m_q = m_q * quat(1.0f, dx(0), dx(1), dx(2));
    m_q /= m_q.norm();

    for (int i=0; i < 3; i++) {
        m_b[i] += dx[i+3];
    }
}

Eigen::Matrix<float,3,1> AttitudeESKF::getRPY() const
{	
	const Matrix<float,3,3> R = m_q.to_matrix();
	Matrix<float,3,1> rpy;

	float sth = -R(2,0);
	if (sth > 1.0f) {
		sth = 1.0f;
	} else if (sth < -1.0f) {
		sth = -1.0f;
	}

	const float theta = std::asin(sth);
	const float cth = std::sqrt(1.0f - sth*sth);

	float phi, psi;
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

	rpy[0] = phi;
	rpy[1] = theta;
	rpy[2] = psi;
	return rpy;
}
