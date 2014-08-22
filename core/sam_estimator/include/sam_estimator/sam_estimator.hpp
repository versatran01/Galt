/*
 * sam_estimator.hpp
 *  _   _             _           _____         _
 * | \ | | ___  _   _| | ____ _  |_   _|__  ___| |__
 * |  \| |/ _ \| | | | |/ / _` |   | |/ _ \/ __| '_ \
 * | |\  | (_) | |_| |   < (_| |   | |  __/ (__| | | |
 * |_| \_|\___/ \__,_|_|\_\__,_|   |_|\___|\___|_| |_|
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of sam_estimator.
 *
 *	Created on: 21/08/2014
 */

#ifndef GALT_SAM_ESTIMATOR_HPP_
#define GALT_SAM_ESTIMATOR_HPP_

#include <sam_estimator/gtsam.hpp>
#include <sam_estimator/common.hpp> //  must include after gstam

#include <memory>
#include <stdexcept>
#include <vector>
#include <deque>

namespace galt {
namespace sam_estimator {

/**
 * @brief SamEstimator performs smoothing and mapping using a combination of
 * IMU, GPS, visual odometry, and laser data.
 * @author Gareth Cross
 * @author Chao Qu
 */
class SamEstimator {
 public:
  typedef std::shared_ptr<SamEstimator> Ptr;
  typedef std::runtime_error exception;
  typedef double Timestamp;
  
  /**
   * @brief Options supported by SamEstimator
   * @todo Load these from ROS param in node class...
   */
  struct Configuration {
    double accel_std;       /// Uncertainty on accelerometer
    double gyro_std;        /// Gyroscope
    double integration_std; /// Integration (see ImuFactor)
    double accel_bias_std;  /// Accelerometer bias
    double gyro_bias_std;   /// Gyroscope bias
    
    double gravity_mag;     /// Magnitude of gravitational acceleration
    
    Configuration() {
      //  initialize defaults
      accel_std = 1.0;
      gyro_std = 1e-3;
      integration_std = 0.1;
      accel_bias_std = 0.1;
      gyro_bias_std = 1e-3;
      gravity_mag = 9.80665;
    }
  };
  
  /**
   * @brief IMU measurement: acceleration + gyroscope
   * @note Order: [ax,ay,az,wx,wy,wz]
   */
  struct ImuMeasurement {
    kr::vec<double,6> z;  /// Accel and angular rates
    Timestamp time;
    Timestamp dt;         /// Time over which measurement is valid         
  };
  
  /**
   * @brief GPS/attitude measurement: orientation + cartesian position
   * @note Order: [x,y,z,rot_x,rot_y,rot_z,vx,vy,vz]
   */
  struct GpsMeasurement {
    kr::Posed pose;           /// 6 DOF pose
    kr::vec3d vel;            /// [x,y,z] velocities
    kr::mat<double,9,9> cov;  /// 9x9 covariance
    Timestamp time;           /// Timestamp in seconds
  };
  
  SamEstimator();

  void AddImu(const ImuMeasurement& measurement);

  void AddGps(const GpsMeasurement& measurement);

  void AddStereo();
  
  void Initialize();

  void Optimize();
  
  bool IsInitialized() const { return initialized_; }

  Configuration& Config() { return config_; }
  const Configuration& Config() const { return config_; }
  
 private:
  
  bool CreateImuFactor(Timestamp time, gtsam::ImuFactor &factor);
  
  gtsam::Symbol PoseKey(int rel=0) const;
  gtsam::Symbol VelocityKey(int rel=0) const;
  gtsam::Symbol BiasKey(int rel=0) const;
  
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values estimates_;
  gtsam::imuBias::ConstantBias constantBias_;
  
  std::deque<ImuMeasurement> imu_buffer_;
  
  Configuration config_;
  
  int meas_index_;
  bool initialized_;
};

}  // namespace sam_estimator
}  // namespace galt

#endif  // GALT_SAM_ESTIMATOR_HPP_
