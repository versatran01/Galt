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
#include <stereo_vo/FeatureMsg.h>
#include <image_geometry/stereo_camera_model.h>

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
  
  SamEstimator();

  /// Add a new frame with pose estimate and match stereo features    
  void AddFrame(const kr::Posed& pose, const kr::mat<double,6,6>& pose_cov,
                const std::vector<stereo_vo::FeatureMsg>& feat_left,
                const std::vector<stereo_vo::FeatureMsg>& feat_right);
    
  /// Check if the graph is initialized.
  bool IsInitialized() const { return meas_index_ >= kNumInitPoses; }
  
  /// Set camera calib.
  void SetCameraModel(const image_geometry::StereoCameraModel& model);
    
  /// Set camera pose in body.
  void SetSensorPose(const kr::Posed& cam_pose);
  
  /// Triangulated points from last frame.
  const std::vector<gtsam::Point3>& triangulated_points() const {
    return triangulated_points_;
  }
  
  /// Last optimized pose
  const gtsam::Pose3 current_pose() const {
    return current_pose_;
  }
  
private:
      
  struct CandidateEdge {
    unsigned int pose_id;
    unsigned int landmark_id;
    kr::vec2d measurement;
    kr::vec3d estimate;
  };
  
  std::map<unsigned int, CandidateEdge> candidates_;
  
  /// Triangulate a point.
  bool Triangulate(const geometry_msgs::Point& left,
                   const geometry_msgs::Point& right,
                   const kr::Posed &odom_pose, gtsam::Point3 &output_point,
                   kr::mat3d& covariance);
  
  static constexpr int kNumInitPoses = 2;
  
  void PerformUpdate();
    
  gtsam::Symbol PoseKey(int index) const;
  gtsam::Symbol CurPoseKey() const { return PoseKey(meas_index_); }
  gtsam::Symbol PrevPoseKey() const { return PoseKey(meas_index_-1); }
  
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values estimates_;
  gtsam::ISAM2 isam_;
  gtsam::Pose3 current_pose_;
  gtsam::Cal3_S2Stereo::shared_ptr calib_;
  image_geometry::StereoCameraModel model_;
  kr::Posed cam_pose_in_body_;
  
  std::set<unsigned long> current_ids_;
  std::vector<gtsam::Point3> triangulated_points_;
  
  typedef gtsam::GenericProjectionFactor<gtsam::Pose3, 
      gtsam::Point3, gtsam::Cal3_S2> ProjectionFactor;
    
  typedef gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>
    StereoProjectionFactor;
  
  int meas_index_;
  bool initialized_;
  kr::Posed previous_odom_;
};

}  // namespace sam_estimator
}  // namespace galt

#endif  // GALT_SAM_ESTIMATOR_HPP_
