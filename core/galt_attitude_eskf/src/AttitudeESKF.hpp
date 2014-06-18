/*
 * AttitudeESKF.hpp
 *
 *  Copyright (c) 2013 Gareth Cross. All rights reserved.
 *
 *  This file is part of AttitudeESKF.
 *
 *	Created on: 12/24/2013
 *		Author: gareth
 */

#ifndef __AttitudeESKF__
#define __AttitudeESKF__

#include "quaternion.hpp"

/**
 *  @class AttitudeESKF
 *  @brief Implementation of an error-state EKF for attidude determination using Quaternions
 *  @note Gravity is the attitude vector in this imeplementaiton.
 *  @see 'Attitude Error Representations for Kalman Filtering' F. Landis Markley
 */
class AttitudeESKF
{
public:
    
    /**
     *  @brief Ctor, initializes P,Q and R matrices to reasonable values
     *
     *	@param varAccel Estimated variance of accerometer noise
     *	@param varGyro Estimated variance of gyroscope noise
     *	@param varGyroBias Estimated variance of gyroscope bias noise
     */
    AttitudeESKF(float varAccel = 1.0f, float varGyro = 0.001f, float varGyroBias = 0.001f);
    
    /**
     *  @brief Perform the prediction step
     *  @param wg Uncorrected gyroscope readings in body frame
     *  @param time Current time in seconds
     *
     *  @note Integrates the nominal state using RK4.
     */
    void predict(const Eigen::Matrix<float,3,1>& wg, double time);
    
    /**
     *  @brief Perform the update step
     *  @param ab Accelerometer reading in body frame (units of Gs)
     */
    void update(const Eigen::Matrix<float,3,1>& ab);
    
    /**
     *  @brief Orientation as quaternion
     */
    const quat& getState() const { return m_q; }

    /**
     *	@brief Get Roll-Pitch-Yaw as a 3-element vector
     */
    Eigen::Matrix<float,3,1> getRPY() const;
    
    /**
     *  @brief Bias estimate
     */
    const Eigen::Matrix<float,3,1>& getGyroBias() const { return m_b; }
    
    /**
     *  @brief Set the steady state estimate of gyro bias
     */
    void setGyroBias(const Eigen::Matrix<float,3,1>& bias) { m_b = bias; }

    /**
     *  @brief False if kalman gain becomes singular
     */
    bool isStable() const { return m_isStable; }
    
//  temp: public
    
    quat m_q;
    Eigen::Matrix<float,3,1> m_b;
    
    double lastTime_;
    Eigen::Vector3f predAccel_;
    
    Eigen::Matrix<float,6,6> m_P;
    Eigen::Matrix<float,6,6> m_Q;
    Eigen::Matrix<float,3,3> m_R;
    
    bool m_isStable;
};

#endif /* defined(__AttitudeESKF__) */
