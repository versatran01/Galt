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
 *  @brief Implementation of an error-state EKF for attitude determination using Quaternions
 *  @note Gravity is the attitude vector in this imeplementaiton.
 *  @see 'Attitude Error Representations for Kalman Filtering' F. Landis Markley
 */
class AttitudeESKF
{
public:
  
  typedef double scalar_t;  /**< Type used for all calculations, change as performance requires */

  static_assert(std::is_fundamental<scalar_t>::value && !std::numeric_limits<scalar_t>::is_integer, 
                "scalar_t must be non-integer fundamental type");
  
  typedef Eigen::Matrix<scalar_t,3,1> vec3; /**< Vector in R3 */
  
  /**
   * @brief The VarSettings struct
   */
  struct VarSettings {
    scalar_t accel[3];
    scalar_t gyro[3];
    scalar_t mag[3];
    
    VarSettings() {
      for (int i=0; i < 3; i++) {
        accel[i] = gyro[i] = mag[i] = 0.0;
      }
    }
  };
  
  /**
   * @brief The MagCalibration struct
   */
  struct MagCalibration {
    scalar_t bias[3];   /**< Magnetometer bias, default [0 0 0] */
    scalar_t scale[3];
    scalar_t rstd;
  };
  
  /**
   *  @brief Ctor, initializes state to all zeros
   */
  AttitudeESKF();

  /**
   *  @brief Perform the prediction step
   *  @param wg Uncorrected gyroscope readings in body frame
   *  @param time Current time in seconds
   *
   *  @note Integrates the nominal state using RK4.
   */
  void predict(const vec3& wg, double time);
  
  /**
   *  @brief Perform the update step
   *  @param ab Accelerometer reading in body frame (units of Gs)
   */
  void update(const vec3& ab, const vec3& mb = vec3::Zero());
  
  /**
   *	@brief Get Roll-Pitch-Yaw as a 3-element vector
   */
  Eigen::Matrix<scalar_t,3,1> getRPY() const;
  
  /**
   * @brief setEstimatesBias
   * @param estBias
   */
  void setEstimatesBias(bool estBias) { estBias_ = estBias; }
  
  /**
   * @brief setUsesMagnetometer
   * @param useMag
   */
  void setUsesMagnetometer(bool useMag) { useMag_ = useMag; }
  
  /**
   * @brief setVariances
   * @param var
   */
  void setVariances(const VarSettings& var) { var_ = var; }
  
  /**
   * @brief setMagneticReference
   * @param magRef
   */
  void setMagneticReference(const vec3& magRef) { magRef_ = magRef; }
  
  /**
   * @brief getQuat
   * @return 
   */
  const quat& getQuat() const { return q_; }
  
  /**
   * @brief getAngularVelocity
   * @return 
   */
  const vec3& getAngularVelocity() const { return w_; }
  
  /**
   * @brief getGyroBias
   * @return 
   */
  const vec3& getGyroBias() const { return b_; }
    
  /**
   * @brief getCovariance
   * @return 
   */
  const Eigen::Matrix<scalar_t,3,3>& getCovariance() const { return P_; }
  
  /**
   * @brief isStable
   * @return 
   */
  bool isStable() const { return isStable_; }
  
private:
  quat q_;                         /// Orientation
  Eigen::Matrix<scalar_t,3,3> P_;  /// System covariance
  double lastTime_;
  
  vec3 w_;
  vec3 b_;
  unsigned long steadyCount_;

  vec3 magRef_;  //  North
  
  bool isStable_;
  bool estBias_;
  bool useMag_;
  
  VarSettings var_;
};

#endif /* defined(__AttitudeESKF__) */
