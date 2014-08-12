#include "stereo_vo/stereo_vo.h"
#include "stereo_vo/feature.h"
#include "stereo_vo/key_frame.h"
#include "stereo_vo/utils.h"

#include <sstream>
#include <iostream>

#include <image_geometry/stereo_camera_model.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <kr_math/base_types.hpp>
#include <kr_math/feature.hpp>
#include <kr_math/pose.hpp>
#include <kr_math/SO3.hpp>

#include <Eigen/Geometry>

namespace galt {

namespace stereo_vo {

using image_geometry::StereoCameraModel;

void StereoVo::Initialize(const CvStereoImage &stereo_image,
                          const StereoCameraModel &model) {
  model_ = model;
  // Add the first stereo image as first keyframe
  // At this moment, we use current_pose as input, but inherently will use
  // estimated depth to reinitialize current_pose. Later will be replaced with
  // estimates of other sensors.
  std::vector<Corner> tracked_corners;
  AddKeyFrame(relative_pose(), stereo_image, tracked_corners);

  // Save stereo image and tracked corners for next iteration
  stereo_image_prev_ = stereo_image;
  corners_ = tracked_corners;

  // Create a window for display
  // TODO: replace this with published topic later
  cv::namedWindow("display",
                  CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  init_ = true;
  ROS_INFO_STREAM("StereVo initialized, baseline: " << model_.baseline());
}

void StereoVo::Iterate(const CvStereoImage &stereo_image) {
  std::vector<Corner> tracked_corners;
  // Track corners from previous frame into current frame
  // Remove corresponding features in last key frame only if they are newly
  // initialized
  TrackTemporal(stereo_image_prev_.first, stereo_image.first, corners_,
                tracked_corners, key_frame_prev());

  // Estimate pose using 2D-to-3D correspondences
  // 2D - currently tracked corners
  // 3D - features in last key frame
  relative_pose_ = EstimatePose();
  absolute_pose_ = key_frame_prev().pose().compose(relative_pose_);

  // Check whether to add key frame based on the following criteria
  // 1. Camera has moved some distance away from the last key frame pose
  // 2. The percentage of cells with corners is below some threshold
  // After this step, tracked_corners will contain both old corners and newly
  // added corners
  AddKeyFrame(absolute_pose_, stereo_image, tracked_corners);

  // Do a windowed optimization if window size is reached
  if (key_frames_.size() > static_cast<unsigned>(config_.kf_size)) {
    // Do awesome optimization which will update poses and features in all
    // keyframes
    //    BundleAdjustment();
    if (key_frames_.size() == 20) key_frames_.pop_front();
  }

  // Visualization (optional)
  Display(stereo_image, tracked_corners, key_frame_prev());

  // Save stereo image and tracked corners for next iteration
  stereo_image_prev_ = stereo_image;
  corners_ = tracked_corners;
}

void StereoVo::TrackTemporal(const cv::Mat &image_prev, const cv::Mat &image,
                             const std::vector<Corner> &corners1,
                             std::vector<Corner> &corners2,
                             KeyFrame &key_frame) {
  std::vector<CvPoint2> points1, points2;
  std::vector<uchar> status;
  std::vector<Feature::Id> ids, ids_to_remove;
  for (const auto &c : corners1) {
    points1.push_back(c.p_pixel());
    ids.push_back(c.id());
  }
  status.reserve(points1.size());
  // LK tracker
  OpticalFlow(image_prev, image, points1, points2, status);
  PruneByStatus(status, ids, ids_to_remove);
  PruneByStatus(status, points1);
  PruneByStatus(status, points2);
  status.clear();
  status.reserve(points1.size());
  // Find fundamental matrix
  cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, 1.5, 0.99, status);
  PruneByStatus(status, ids, ids_to_remove);
  PruneByStatus(status, points2);
  ROS_ASSERT_MSG(ids.size() == points2.size(), "Dimension mismatch");
  // Prune features in last key frame
  key_frame.PruneById(ids_to_remove);
  // Reconstruct corners for later use
  auto it_id = ids.cbegin(), it_id_e = ids.cend();
  auto it_pts = points2.cbegin();
  for (; it_id != it_id_e; ++it_id, ++it_pts) {
    // It doesn't matter here if these corners are init or not
    // AddFeatures will set all of them to false
    corners2.emplace_back(*it_id, *it_pts, false);
  }
}

Pose StereoVo::EstimatePose() {
  const size_t N = key_frame_prev().features().size();

  std::vector<CvPoint2> imagePoints;
  std::vector<CvPoint3> worldPoints;
  std::vector<uchar> inliers;

  imagePoints.reserve(N);
  worldPoints.reserve(N);

  for (const Corner &corner : corners_) {
    const auto it = features_.find(corner.id());
    if (it != features_.end()) {
      const Feature &feat = it->second;
      imagePoints.push_back(corner.p_pixel());
      worldPoints.push_back(feat.p_world());
    }
  }

  if (imagePoints.empty()) {
    throw std::runtime_error("EstimatePose called with empty features");
  }

  cv::Mat rvec = cv::Mat(3, 1, CV_64FC1);
  cv::Mat tvec = cv::Mat(3, 1, CV_64FC1);
  const size_t minInliers =
      std::ceil(worldPoints.size() * config_.pnp_ransac_inliers);
  cv::solvePnPRansac(worldPoints, imagePoints,
                     model_.left().fullIntrinsicMatrix(), std::vector<double>(),
                     rvec, tvec, false, 100, config_.pnp_ransac_error,
                     minInliers, inliers, cv::ITERATIVE);

  kr::vec3<scalar_t> r(rvec.at<double>(0, 0), rvec.at<double>(1, 0),
                       rvec.at<double>(2, 0));
  kr::vec3<scalar_t> t(tvec.at<double>(0, 0), tvec.at<double>(1, 0),
                       tvec.at<double>(2, 0));

  return Pose::fromVectors(r, t);
}

void StereoVo::AddKeyFrame(const Pose &pose, const CvStereoImage &stereo_image,
                           std::vector<Corner> &corners) {
  const auto dist = relative_pose().p.norm();
  const auto angles = kr::getRPY(relative_pose().q.matrix());
  const auto yaw = angles[2];
  // const auto filled = detector_.GridFilled(stereo_image.first, corners);
  // Check distance and feature distribution metric
  if ((dist > config_.kf_dist) ||
      (corners.size() < config_.kf_min_filled * config_.shi_max_corners) ||
      (std::abs(yaw) > 45.0 / 180 * M_PI)) {
    std::map<Feature::Id, Feature> features;
    // Add new corners to current corners
    // After this, we will have two types of corner in corners
    // One is tracked from previous key frame, the other is newly added
    detector_.AddFeatures(stereo_image.first, corners);
    // Track corners from left image to right image and triangulate them
    // We will remove corners that are lost during tracking and have bad
    // triangulation score. Features will be created based on the corners left
    TrackSpatial(stereo_image, corners, features);
    
    
    
    if (key_frames_.empty()) {
      //  some hacky initializing code just for better visualization
      double depth = 0;
      for (const std::pair<Feature::Id, Feature> &feat : features) {
        depth += feat.second.p_cam_left().z;
      }
      depth /= features.size();
      relative_pose_.q = kr::quat<scalar_t>(0, 1, 0, 0);
      relative_pose_.p = kr::vec3<scalar_t>(0, 0, depth);
    }
    // Add key frame to queue with current_pose, features and stereo_image
    key_frames_.emplace_back(pose, features, stereo_image);
  }
}

void StereoVo::TrackSpatial(const CvStereoImage &stereo_image,
                            std::vector<Corner> &corners,
                            std::map<Corner::Id, Feature> &features) {
  std::vector<CvPoint2> points1, points2;
  std::vector<uchar> status;
  for (const auto &c : corners) points1.push_back(c.p_pixel());
  // LK tracker
  OpticalFlow(stereo_image.first, stereo_image.second, points1, points2,
              status);
  PruneByStatus(status, points1);
  PruneByStatus(status, points2);
  PruneByStatus(status, corners);
  status.clear();
  // Find fundamental matrix
  cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, 1.5, 0.99, status);
  PruneByStatus(status, points2);
  PruneByStatus(status, corners);
  ROS_ASSERT_MSG(corners.size() == points2.size(), "Dimension mismatch");
  // Now we use corners and points2 to create new features
  auto it_crn = corners.cbegin(), it_crn_e = corners.cend();
  auto it_pts = points2.cbegin();
  for (; it_crn != it_crn_e; ++it_crn, ++it_pts) {
    // Create a feature using corner and pixel_right
    Feature feature(*it_crn, *it_pts);
//    if (feature.triangulate(model_, config_.tri_max_eigenratio)) {
//      features[it_crn->id()] = feature;
//    }
  }
  
  
}

void StereoVo::OpticalFlow(const cv::Mat &image1, const cv::Mat &image2,
                           const std::vector<CvPoint2> &points1,
                           std::vector<CvPoint2> &points2,
                           std::vector<uchar> &status) {
  int win_size = config_.klt_win_size;
  int max_level = config_.klt_max_level;
  static cv::TermCriteria term_criteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 25, 0.01);
  cv::calcOpticalFlowPyrLK(image1, image2, points1, points2, status,
                           cv::noArray(), cv::Size(win_size, win_size),
                           max_level, term_criteria);
}

}  // namespace stereo_vo

}  // namespace galt
