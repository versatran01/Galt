/*
 * quaternion.cpp
 *
 *  Copyright (c) 2013 Gareth Cross. All rights reserved.
 *
 *  This file is part of AttitudeESKF.
 *
 *	Created on: 12/13/2013
 *		Author: gareth
 */

#include <cmath>
#include "quaternion.hpp"

using namespace std;

quat::quat()
{
    m_q[0] = 1.0f;
    m_q[1] = m_q[2] = m_q[3] = 0.0f;
}

quat::quat(float a, float b, float c, float d)
{
    this->a() = a;
    this->b() = b;
    this->c() = c;
    this->d() = d;
}

quat::quat(const quat& q)
{
    for (int i=0; i < 4; i++) {
        operator()(i) = q(i);
    }
}

quat::quat(const vec4& v)
{
	for (int i=0; i < 4; i++) {
		m_q[i] = v[i];
	}
}

quat& quat::operator = (const quat& q)
{
    for (int i=0; i < 4; i++) {
        operator()(i) = q(i);
    }
    return *this;
}

float quat::norm() const
{
    return sqrt(a()*a() + b()*b() + c()*c() + d()*d());
}

void quat::normalize()
{
    const float n = norm();
    operator *= (*this, 1.0f / n);
}

quat quat::conjugate() const
{
    return quat(a(), -b(), -c(), -d());
}

quat quat::transform(const quat& v) const
{
    const quat& q = *this;
    return q * v * q.conjugate();
}

Eigen::Matrix<float,3,3> quat::to_matrix() const
{
    Eigen::Matrix<float,3,3> R;
    
    const float aa = a()*a();
    const float bb = b()*b();
    const float cc = c()*c();
    const float dd = d()*d();
    
    R(0,0) = aa + bb - cc - dd;
    R(1,0) = 2*b()*c() + 2*a()*d();
    R(2,0) = 2*b()*d() - 2*a()*c();
    
    R(0,1) = 2*b()*c() - 2*a()*d();
    R(1,1) = aa - bb + cc - dd;
    R(2,1) = 2*c()*d() + 2*a()*b();
    
    R(0,2) = 2*b()*d() + 2*a()*c();
    R(1,2) = 2*c()*d() - 2*a()*b();
    R(2,2) = aa - bb - cc + dd;
    
    return R;
}

Eigen::Matrix<float,3,1> quat::vec() const
{
	Eigen::Matrix<float,3,1> v;
	v[0] = b();
	v[1] = c();
	v[2] = d();
    return v;
}

quat::operator vec4() const
{
	vec4 v;
	v[0] = a();
	v[1] = b();
	v[2] = c();
	v[3] = d();
    return v;
}

void quat::integrateRungeKutta4(const quat& w, float dt, bool normalize)
{
    quat& q = *this;
    quat qw = q * w * 0.5f;
    
    quat k2 = (q + qw*dt*0.5f) * w * 0.5f;
    quat k3 = (q + k2*dt*0.5f) * w * 0.5f;
    quat k4 = (q + k3*dt) * w * 0.5f;
    
    q += (qw + k2*2.0f + k3*2.0f + k4) * (dt / 6.0f);
    
    if (normalize) {
        q.normalize();
    }
}

void quat::integrateEuler(const quat& w, float dt, bool normalize)
{
    quat& q = *this;
    q += (q * w * 0.5f) * dt;
    
    if (normalize) {
        q.normalize();
    }
}

quat quat::rotation(float theta, float x, float y, float z)
{
    const float haversine = sin(0.5f * theta);
    const float havercosine = cos(0.5f * theta);
    
    return quat(
        havercosine,
        haversine * x,
        haversine * y,
        haversine * z
    );
}

quat quat::rotation_xyz(float x, float y, float z)
{
    quat q = rotation(x, 1.0f, 0.0f, 0.0f) * rotation(y, 0.0f, 1.0f, 0.0f) * rotation(z, 0.0f, 0.0f, 1.0f);
    return q;
}

quat quat::rotation(float x, float y, float z)
{
    const float theta = std::sqrt(x*x + y*y + z*z);
    
    if (theta < 1e-6f) {
        return quat(1.0f, 0.0f, 0.0f, 0.0f);
    }
    
    return rotation(theta, x / theta, y / theta, z / theta);
}

/**
 *  As documented here: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
 */
quat quat::from_matrix(const Eigen::Matrix<float,3,3>& m)
{
    quat Q;
    const float tr = m(0,0) + m(1,1) + m(2,2);	//	trace
    float s = 0.0f;
    
    if (tr > 0.0f)
    {
        s = 2.0f * sqrtf(1 + tr);
        
        Q.a() = 0.25f * s;
        Q.b() = (m(2,1) - m(1,2)) / s;
        Q.c() = (m(0,2) - m(2,0)) / s;
        Q.d() = (m(1,0) - m(0,1)) / s;
    }
    else if ( m(0,0) > m(1,1) && m(0,0) > m(2,2) )
    {
        s = 2.0f * sqrtf( 1 + m(0,0) - m(1,1) - m(2,2) );
        
        Q.a() = (m(2,1) - m(1,2)) / s;
        Q.b() = 0.25f * s;
        Q.c() = (m(0,1) + m(1,0)) / s;
        Q.d() = (m(0,2) + m(2,0)) / s;
    }
    else if ( m(1,1) > m(2,2) )
    {
        s = 2.0f * sqrtf( 1 + m(1,1) - m(0,0) - m(2,2) );
        
        Q.a() = (m(0,2) - m(2,0)) / s;
        Q.b() = (m(0,1) + m(1,0)) / s;
        Q.c() = 0.25f * s;
        Q.d() = (m(1,2) + m(2,1)) / s;
    }
    else
    {
        s = 2.0f * sqrtf( 1 + m(2,2) - m(0,0) - m(1,1) );
        
        Q.a() = (m(1,0) - m(0,1)) / s;
        Q.b() = (m(0,2) + m(2,0)) / s;
        Q.c() = (m(1,2) + m(2,1)) / s;
        Q.d() = 0.25f * s;
    }
    
    return Q;
}

/*
 *  The multiplication of quaternions is described in:
 *
 *  "Quaternions And Dynamics", Basile Graf, 2007
 */
quat operator * (const quat& a, const quat &b)
{
    quat lhs;
    
    lhs(0) = a(0)*b(0) - a(1)*b(1) - a(2)*b(2) - a(3)*b(3);
    lhs(1) = a(0)*b(1) + a(1)*b(0) + a(2)*b(3) - a(3)*b(2);
    lhs(2) = a(0)*b(2) - a(1)*b(3) + a(2)*b(0) + a(3)*b(1);
    lhs(3) = a(0)*b(3) + a(1)*b(2) - a(2)*b(1) + a(3)*b(0);
    
    return lhs;
}

quat operator * (const quat& a, const float s)
{
    quat lhs;
    
    for (int i=0; i < 4; i++) {
        lhs(i) = a(i) * s;
    }
    
    return lhs;
}

quat operator * (const float s, const quat& a)
{
    return operator * (a, s);
}

quat operator / (const quat& a, const float s)
{
    return operator * (a, 1.0f / s);
}

quat& operator *= (quat& a, const float s)
{
    for (int i=0; i < 4; i++) {
        a(i) *= s;
    }
    
    return a;
}

quat& operator /= (quat& a, const float s)
{
    return operator *= (a, 1.0f / s);
}

quat operator + (const quat& a, const quat &b)
{
    quat lhs;
    
    for (int i=0; i < 4; i++) {
        lhs(i) = a(i) + b(i);
    }
    
    return lhs;
}

quat& operator += (quat& a, const quat &b)
{
    for (int i=0; i < 4; i++) {
        a(i) += b(i);
    }
    
    return a;
}

