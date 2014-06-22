/*
 * quaternion.hpp
 *
 *  Copyright (c) 2013 Gareth Cross. All rights reserved.
 *
 *  This file is part of AttitudeESKF.
 *
 *	Created on: 12/13/2013
 *      Author: gareth
 */

#ifndef quaternion_hpp
#define quaternion_hpp

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

/**
 *  @class Floating point representation of a quaternion
 */
template <typename Scalar>
class quat
{
  static_assert(std::is_fundamental<Scalar>::value &&
                !std::numeric_limits<Scalar>::is_integer, "Scalar must be a floating point type");

public:

    typedef Eigen::Matrix<Scalar,4,1> vec4; /**< R4 representation */
    typedef Eigen::Matrix<Scalar,3,1> vec3; /**< R3 representation */
  
    /**
     *  @brief Create a quaternion with null rotation
     */
    quat() {
      m_q[0] = 1.0;
      m_q[1] = m_q[2] = m_q[3] = 0.0;
    }

    /**
     *  @brief Construct a quaterion
     *  @param a Scalar parameter
     *  @param b,c,d Complex parameters
     */
    quat(Scalar a, Scalar b, Scalar c, Scalar d) {
      m_q[0] = a;
      m_q[1] = b;
      m_q[2] = c;
      m_q[3] = d;
    }

    /**
     *  @brief Copy operator
     */
    quat(const quat& q) {
      for (int i=0; i < 4; i++) {
          operator()(i) = q(i);
      }
    }

    /**
     *  @brief Create quaternion from R4 vector
     *  @note Performs q = [v0, v1, v2, v3]
     */
    quat(const vec4& v) {
      for (int i=0; i < 4; i++) {
        operator()(i) = v[i];
      }
    }

    /**
     *  @brief Assignment operator
     */
    quat& operator = (const quat& q) {
      for (int i=0; i < 4; i++) {
          operator()(i) = q(i);
      }
      return *this;
    }

    /**
     *  @brief L2 norm of the quaternion
     */
    Scalar norm() const {
      return std::sqrt(a()*a() + b()*b() + c()*c() + d()*d());
    }

    /**
     *  @brief Normalize quaternion
     */
    void normalize() {
      const Scalar n = norm();
      operator *= (*this, 1.0 / n);
    }

    /**
     *  @brief Complex conjugate quaternion
     */
    quat conjugate() const {
      return quat(a(), -b(), -c(), -d());
    }

    /**
     *  @brief Rotate a vector using this quaternion
     *  @param v Vector stored in the three complex terms
     */
    quat transform(const quat& v) const {
      const quat& q = *this;
      return q * v * q.conjugate();
    }

    /**
     *  @brief Convert a rotation quaternion to its matrix form
     *  @note The result is not correct if this quaternion is not a member of S(4)
     *  @return 3x3 Rotation matrix
     */
    Eigen::Matrix<Scalar,3,3> to_matrix() const
    {
      Eigen::Matrix<Scalar,3,3> R;
      
      const Scalar aa = a()*a();
      const Scalar bb = b()*b();
      const Scalar cc = c()*c();
      const Scalar dd = d()*d();
      
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

    /**
     *  @brief Obtain vector component of this quaternion
     *  @return Complex three terms of the quaternion
     */
    vec3 vec() const {
      vec3 v;
      v[0] = b();
      v[1] = c();
      v[2] = d();
      return v;
    }

    /**
     *  @brief Cast to R4
     */
    operator vec4() const {
      vec4 v;
      v[0] = a();
      v[1] = b();
      v[2] = c();
      v[3] = d();
      return v;
    }

    /**
     *  @brief Integrate a rotation quaternion using 4th order Runge Kutta
     *  @param w Angular velocity (body frame), stored in 3 complex terms
     *  @param dt Time interval in seconds
     *  @param normalize If true, quaternion is normalized after integration
     */
    void integrateRungeKutta4(const quat& w, Scalar dt, bool normalize = true)
    {
      quat& q = *this;
      quat qw = q * w * 0.5;
      
      quat k2 = (q + qw*dt*0.5) * w * 0.5;
      quat k3 = (q + k2*dt*0.5) * w * 0.5;
      quat k4 = (q + k3*dt) * w * 0.5;
      
      q += (qw + k2*2.0 + k3*2.0 + k4) * (dt / 6.0);
      
      if (normalize) {
        q.normalize();
      }
    }

    /**
     *  @brief Integrate a rotation quaterion using Euler integration
     *  @param w Angular velocity (body frame), stored in 3 complex terms
     *  @param dt Time interval in seconds
     *  @param normalize If True, quaternion is normalized after integration
     */
    void integrateEuler(const quat& w, Scalar dt, bool normalize = true)
    {
      quat& q = *this;
      q += (q * w * 0.5) * dt;
      
      if (normalize) {
        q.normalize();
      }
    }

    /**
     *  @brief Create a rotation quaterion
     *  @param theta Angle of rotation, radians
     *  @param x X component of rotation vector
     *  @param y Y component
     *  @param z Z component
     */
    static quat rotation(Scalar theta, Scalar x, Scalar y, Scalar z) {
      const Scalar haversine = std::sin(0.5 * theta);
      const Scalar havercosine = std::cos(0.5 * theta);
      
      return quat(
        havercosine,
        haversine * x,
        haversine * y,
        haversine * z
      );
    }

    /**
     *  @brief Create a rotation quaternion from rotation vector [x,y,z]
     *  @note If x/y/z have 0 norm, this function returns the identity transform
     */
    static quat rotation(float x, float y, float z)
    {
        const float theta = std::sqrt(x*x + y*y + z*z);
        
        if (theta < 1e-6) {
            return quat(1.0, 0.0, 0.0, 0.0);
        }
        
        return rotation(theta, x / theta, y / theta, z / theta);
    }

    /**
     *  @brief Create quaternion from matrix
     *  @param m Rotation matrix, should be a member of SO(3).
     *  @note All singularities are handled, provided m belongs to SO(3).
     */
    static quat from_matrix(const Eigen::Matrix<Scalar,3,3>& m)
    {
        quat Q;
        const float tr = m(0,0) + m(1,1) + m(2,2);	//	trace
        float s = 0.0;
        
        if (tr > 0.0)
        {
            s = 2.0 * std::sqrt(1 + tr);
            
            Q.a() = 0.25 * s;
            Q.b() = (m(2,1) - m(1,2)) / s;
            Q.c() = (m(0,2) - m(2,0)) / s;
            Q.d() = (m(1,0) - m(0,1)) / s;
        }
        else if ( m(0,0) > m(1,1) && m(0,0) > m(2,2) )
        {
            s = 2.0 * std::sqrt( 1 + m(0,0) - m(1,1) - m(2,2) );
            
            Q.a() = (m(2,1) - m(1,2)) / s;
            Q.b() = 0.25 * s;
            Q.c() = (m(0,1) + m(1,0)) / s;
            Q.d() = (m(0,2) + m(2,0)) / s;
        }
        else if ( m(1,1) > m(2,2) )
        {
            s = 2.0 * std::sqrt( 1 + m(1,1) - m(0,0) - m(2,2) );
            
            Q.a() = (m(0,2) - m(2,0)) / s;
            Q.b() = (m(0,1) + m(1,0)) / s;
            Q.c() = 0.25 * s;
            Q.d() = (m(1,2) + m(2,1)) / s;
        }
        else
        {
            s = 2.0 * std::sqrt( 1 + m(2,2) - m(0,0) - m(1,1) );
            
            Q.a() = (m(1,0) - m(0,1)) / s;
            Q.b() = (m(0,2) + m(2,0)) / s;
            Q.c() = (m(1,2) + m(2,1)) / s;
            Q.d() = 0.25 * s;
        }
        
        return Q;
    }

    /*
     *  Accessors
     */

    /**
     * @brief operator () Element-wise accessor
     * @param i Index into quaternion, must be less than 4.
     * @return Element i of the quaternion.
     */
    float& operator () (unsigned int i) { return m_q[i]; }
    const float& operator () (unsigned int i) const { return m_q[i]; }

    float& a() { return m_q[0]; }   /**< Scalar component */
    const float& a() const { return m_q[0]; }

    float& b() { return m_q[1]; }   /**< First complex dimension (i) */
    const float& b() const { return m_q[1]; }

    float& c() { return m_q[2]; }   /**< Second complex dimension (j) */
    const float& c() const { return m_q[2]; }

    float& d() { return m_q[3]; }   /**< Third complex dimension (k) */
    const float& d() const { return m_q[3]; }

private:
    float m_q[4];
};

/**
 *  @brief Multiply two quaternions
 *  @param a Left quaternion
 *  @param b Right quaternion
 *  @return Product of both quaternions
 */
template <typename Scalar>
quat<Scalar> operator * (const quat<Scalar>& a, const quat<Scalar>& b)
{
  quat<Scalar> lhs;
  
  lhs(0) = a(0)*b(0) - a(1)*b(1) - a(2)*b(2) - a(3)*b(3);
  lhs(1) = a(0)*b(1) + a(1)*b(0) + a(2)*b(3) - a(3)*b(2);
  lhs(2) = a(0)*b(2) - a(1)*b(3) + a(2)*b(0) + a(3)*b(1);
  lhs(3) = a(0)*b(3) + a(1)*b(2) - a(2)*b(1) + a(3)*b(0);
  
  return lhs;
}

/**
 *  @brief Divide a quaternion by a scalar
 *  @param a Quaternion
 *  @param s Scalar
 *  @return Scaled quaternion
 */
template <typename Scalar>
quat<Scalar> operator / (const quat<Scalar>& a, const Scalar s) {
  return operator * (a, 1.0 / s);
}

/**
 *  @brief Right-multiply a quaternion by a scalar
 *  @param a Quaternion
 *  @param s Scalar
 *  @return Scaled quaterion
 */
template <typename Scalar>
quat<Scalar> operator * (const quat<Scalar>& a, const Scalar s)
{
  quat<Scalar> lhs;
  
  for (int i=0; i < 4; i++) {
      lhs(i) = a(i) * s;
  }
  
  return lhs;
}

/**
 *  @brief Left-multiply a quaternion by a scalar
 *  @param a Quaternion
 *  @param s Scalar
 *  @return Scaled quaterion
 */
template <typename Scalar>
quat<Scalar> operator * (const Scalar s, const quat<Scalar>& a) {
  return operator * (a, s);
}

/**
 *  @brief Multiply a quaternion by a scalar, in place
 *  @param a Quaternion to scale
 *  @param s Scalar
 *  @return a
 */
template <typename Scalar>
quat<Scalar>& operator *= (quat<Scalar>& a, const Scalar s)
{
  for (int i=0; i < 4; i++) {
    a(i) *= s;
  }
  
  return a;
}

/**
 *  @brief Divide a quaternion by a scalar, in place
 *  @param a Quaternion to scale
 *  @param s Scalar
 *  @return a
 */
template <typename Scalar>
quat<Scalar>& operator /= (quat<Scalar>& a, const Scalar s) {
  return operator *= (a, 1.0 / s);
}

/**
 *  @brief Add two quaternions (element-wise summation)
 *  @param a First quaternion
 *  @param b Second quaternion
 *  @return Sum
 */
template <typename Scalar>
quat<Scalar> operator + (const quat<Scalar>& a, const quat<Scalar> &b)
{
    quat<Scalar> lhs;
    
    for (int i=0; i < 4; i++) {
        lhs(i) = a(i) + b(i);
    }
    
    return lhs;
}

/**
 *  @brief Add-in place quaterion
 *  @param a First quaternion, is modified
 *  @param b Second quaternion
 *  @return Sum
 */
template <typename Scalar>
quat<Scalar>& operator += (quat<Scalar>& a, const quat<Scalar>& b)
{
    for (int i=0; i < 4; i++) {
        a(i) += b(i);
    }
    
    return a;
}

#endif
