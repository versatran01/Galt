/*
 * quaternion.hpp
 *
 *  Copyright (c) 2013 Gareth Cross. All rights reserved.
 *
 *  This file is part of AttitudeESKF.
 *
 *	Created on: 12/13/2013
 *		Author: gareth
 */

#ifndef quaternion_hpp
#define quaternion_hpp

#include <Eigen/Core>
#include <Eigen/Dense>

//	equivalent 4-vector representation
typedef Eigen::Matrix<float,4,1> vec4;

/**
 *  @class Floating point representation of a quaternion
 */
class quat
{
public:

    /**
     *  @brief Create a quaternion with null rotation
     */
    quat();

    /**
     *  @brief Construct a quaterion
     *  @param a Scalar parameter
     *  @param b,c,d Complex parameters
     */
    quat(float a, float b, float c, float d);

    /**
     *  @brief Copy operator
     */
    quat(const quat& q);

    /**
     *  @brief Create quaternion from R4 vector
     *  @note Performs q = [v0, v1, v2, v3]
     */
    quat(const vec4& v);

    /**
     *  @brief Assignment operator
     */
    quat& operator = (const quat& q);

    /**
     *  @brief L2 norm of the quaternion
     */
    float norm() const;

    /**
     *  @brief Normalize quaternion
     */
    void normalize();

    /**
     *  @brief Complex conjugate quaternion
     */
    quat conjugate() const;

    /**
     *  @brief Transform a vector using this quaternion
     *  @param v Vector stored in the three complex terms
     */
    quat transform(const quat& v) const;

    /**
     *  @brief Convert a rotation quaternion to its matrix form
     *  @note The result is not correct if this quaternion is not a member of S(4)
     *  @return 3x3 Rotation matrix
     */
    Eigen::Matrix<float,3,3> to_matrix() const;

    /**
     *  @brief Obtain vector component of this quaternion
     */
    Eigen::Matrix<float,3,1> vec() const;

    /**
     *  @brief Cast to vec4
     */
    operator vec4() const;

    /**
     *  @brief Integrate a rotation quaternion using 4th order Runge Kutta
     *  @param w Angular velocity (body frame), stored in 3 complex terms
     *  @param dt Time interval
     *  @param normalize If true, quaternion is normalized after integration
     */
    void integrateRungeKutta4(const quat& w, float dt, bool normalize = true);

    /**
     *  @brief Integrate a rotation quaterion using Euler stepping
     *  @param w Angular velocity (body frame), stored in 3 complex terms
     *  @param dt Time interval
     *  @param normalize If True, quaternion is normalized after integration
     */
    void integrateEuler(const quat& w, float dt, bool normalize = true);

    /**
     *  @brief Create a rotation quaterion
     *  @param theta Angle of rotation, radians
     *  @param x X component of rotation vector
     *  @param y Y component
     *  @param z Z component
     */
    static quat rotation(float theta, float x, float y, float z);

    /**
     *  @brief Rotation through sequence x,y,z
     *  @param x Angle about x axis
     *  @param y Angle about y axis
     *  @param z Angle about z axies
     */
    static quat rotation_xyz(float x, float y, float z);

    /**
     *  @brief Create a rotation quaternion from rotation vector [x,y,z]
     *  @note If x/y/z have 0 norm, this function returns the identity transform
     */
    static quat rotation(float x, float y, float z);

    /**
     *  @brief Create quaternion from matrix
     *  @note Result undefined if m is not a member of SO(3). This method handles all possible singularities.
     */
    static quat from_matrix(const Eigen::Matrix<float,3,3>& m);

    /*
     *  Accessors
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
quat operator * (const quat& a, const quat &b);

/**
 *  @brief Divide a quaternion by a scalar
 *  @param a Quaternion
 *  @param s Scalar
 *  @return Scaled quaternion
 */
quat operator / (const quat& a, const float s);

/**
 *  @brief Multiply a quaternion by a scalar
 *  @param a Quaternion
 *  @param s Scalar
 *  @return Scaled quaterion
 */
quat operator * (const quat& a, const float s);
quat operator * (const float s, const quat& a);

/**
 *  @brief Multiply a quaternion by a scalar, in place
 *  @param a Quaternion to scale
 *  @param s Scalar
 *  @return a
 */
quat& operator *= (quat& a, const float s);

/**
 *  @brief Divide a quaternion by a scalar, in place
 *  @param a Quaternion to scale
 *  @param s Scalar
 *  @return a
 */
quat& operator /= (quat& a, const float s);

/**
 *  @brief Add two quaternions (element-wise summation)
 *  @param a First quaternion
 *  @param b Second quaternion
 *  @return Sum
 */
quat operator + (const quat& a, const quat &b);

/**
 *  @brief Add-in place quaterion
 *  @param a First quaternion, is modified
 *  @param b Second quaternion
 *  @return Sum
 */
quat& operator += (quat& a, const quat &b);

#endif
