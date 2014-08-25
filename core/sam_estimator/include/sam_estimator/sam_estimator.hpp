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
#define KR_MATH_GTSAM_CONVERSIONS

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
    
    double gps_time_delay;  /// Seconds of delay on GPS measurements
    
    Configuration() {
      //  initialize defaults
      accel_std = 1.0;
      gyro_std = 1e-3;
      integration_std = 1.0;
      accel_bias_std = 0.01;
      gyro_bias_std = 1e-3;
      gravity_mag = 9.80665;
      gps_time_delay = 0.0;
    }
  };
  
  /**
   * @brief IMU measurement: acceleration + gyroscope
   * @note Order: [ax,ay,az,wx,wy,wz]
   */
  struct ImuMeasurement {
    kr::vec<double,6> z;  /// Accel and angular rates
    kr::quatd wQb;        /// 3DOF orientation, corrected to geo. north
    kr::mat3d cov;        /// Covariance on IMU measurement
    Timestamp time;
    Timestamp dt;         /// Time over which measurement is valid         
  };
  
  /**
   * @brief GPS/attitude measurement: orientation + cartesian position
   * @note Order: [x,y,z,rot_x,rot_y,rot_z,vx,vy,vz]
   */
  //  NOTE: unused
  struct GpsMeasurement {
    kr::Posed pose;           /// 6 DOF pose
    kr::vec3d vel;            /// [x,y,z] velocities
    kr::mat<double,9,9> cov;  /// 9x9 covariance
    Timestamp time;           /// Timestamp in seconds
  };
  
  /**
   * @brief Visual odometry measurement: relative position and orientation
   * @note Order: [x,y,z,rot_x,rot_y,rot_z]
   */
  struct VoMeasurement {
    kr::Posed pose;
    kr::mat<double,6,6> cov;
    Timestamp time;
  };
  
  SamEstimator();

  void AddImu(const ImuMeasurement& measurement);

  void AddVo(const VoMeasurement &measurement);
    
  void Optimize();
  
  void InitializeGraph(const kr::Posed& first_pose, const gtsam::Vector6 &sigmas, Timestamp start_time);
  
  bool IsInitialized() const { return initialized_; }

  const std::vector<kr::Posed>& AllPoses() const { return all_poses_; }
  
  Configuration& Config() { return config_; }
  const Configuration& Config() const { return config_; }
    
  const gtsam::Pose3& CurrentPose() const { return current_pose_; }
  const gtsam::Matrix6& CurrentMarginals() const { return current_marginals_; }
  
private:
  
  bool ProcessQueues();
  
  void HandleImu(const ImuMeasurement &imu);
  
  void HandleVo(const VoMeasurement& vo);
    
  void AddImuFactor();
  
  void PerformUpdate();
  
  gtsam::noiseModel::Diagonal::shared_ptr BiasNoiseModel() const;
  
  gtsam::Symbol PoseKey(int index) const;
  gtsam::Symbol VelKey(int index) const;
  gtsam::Symbol BiasKey(int index) const;
  gtsam::Symbol CurPoseKey() const { return PoseKey(meas_index_); }
  gtsam::Symbol CurVelKey() const { return VelKey(meas_index_); }
  gtsam::Symbol CurBiasKey() const { return BiasKey(meas_index_); }
  gtsam::Symbol PrevPoseKey() const { return PoseKey(meas_index_-1); }
  gtsam::Symbol PrevVelKey() const { return VelKey(meas_index_-1); }
  gtsam::Symbol PrevBiasKey() const { return BiasKey(meas_index_-1); }
  
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values estimates_;
  gtsam::ISAM2 isam_;
  
  gtsam::Pose3 current_pose_;
  gtsam::Matrix6 current_marginals_;
  
  kr::Posed last_vo_pose_;
  bool has_vo_pose_;
  
  kr::quatd last_rotation_;
  kr::mat3d last_rotation_cov_;
  kr::quatd last_vo_rotation_;
  bool rotation_set_;
  bool has_rotation_{false};
  
  std::deque<ImuMeasurement> imu_buffer_;
  std::deque<VoMeasurement> vo_buffer_;
  
  Configuration config_;
  
  int meas_index_;
  bool initialized_;
  
  std::vector<kr::Posed> all_poses_;
};

}  // namespace sam_estimator
}  // namespace galt

#endif  // GALT_SAM_ESTIMATOR_HPP_
