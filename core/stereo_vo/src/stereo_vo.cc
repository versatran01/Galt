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
  AddKeyFrame(absolute_pose(), stereo_image, tracked_corners);

  // Save stereo image and tracked corners for next iteration
  stereo_image_prev_ = stereo_image;
  corners_ = tracked_corners;

  // Create a window for display
  /// @todo: replace this with published topic later
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
  absolute_pose_ = EstimatePose();

  // Check whether to add key frame based on the following criteria
  // 1. Movement exceeds config_.kf_dist_thresh
  // 2. Yaw angle exceeds config_.kf_yaw_thresh
  // 3. Number of features falls below threshold (see ShouldAddKeyFrame)
  // After this step, tracked_corners will contain both old corners and newly
  // added corners
  if (ShouldAddKeyFrame(tracked_corners.size())) {
    AddKeyFrame(absolute_pose(), stereo_image, tracked_corners);
  }

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
                             const std::vector<Corner> &corners_input,
                             std::vector<Corner> &corners_output,
                             KeyFrame &key_frame) {
  std::vector<CvPoint2> points_in, points_tracked;
  std::vector<uchar> status;
  std::vector<Feature::Id> ids, ids_to_remove;

  for (const Corner &corner : corners_input) {
    //  pictures in left image
    points_in.push_back(corner.p_pixel());
    ids.push_back(corner.id());
  }

  //  track and remove mismatches
  OpticalFlow(image_prev, image, points_in, points_tracked, status);

  PruneByStatus(status, ids, ids_to_remove);
  PruneByStatus(status, points_in);
  PruneByStatus(status, points_tracked);
  status.clear();

  // Find fundamental matrix to reject outliers in tracking
  cv::findFundamentalMat(points_in, points_tracked, cv::FM_RANSAC, 1.5, 0.99,
                         status);
  PruneByStatus(status, ids, ids_to_remove);
  PruneByStatus(status, points_tracked);

  ROS_ASSERT_MSG(ids.size() == points_tracked.size(), "Dimension mismatch");

  //  remove singlet observations from previous keyframes as required
  for (auto it_kf = key_frames_.rbegin(); it_kf != key_frames_.rend();
       it_kf++) {

    KeyFrame &kf = *it_kf;
    if (!kf.RemoveById(ids_to_remove)) {
      break;  //  nothing in this keyframe to remove, we can stop checking
    }
  }

  auto it_pts = points_tracked.cbegin();
  for (auto it_id = ids.cbegin(); it_id != ids.cend(); ++it_id, ++it_pts) {
    // It doesn't matter here if these corners are init or not
    // AddFeatures will set all of them to false
    corners_output.emplace_back(*it_id, *it_pts, false);
  }
}

Pose StereoVo::EstimatePose() {
  const size_t N = key_frame_prev().corners().size();

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

bool StereoVo::ShouldAddKeyFrame(size_t num_corners) const {
  //  no keyframes, add one
  if (key_frames_.empty()) {
    return true;
  }

  const KeyFrame &kf_prev = key_frames_.back();
  const Pose diff = absolute_pose().difference(kf_prev.pose());
  if (diff.p.norm() > config_.kf_dist_thresh) {
    //  over distance threshold, add keyframe
    return true;
  }

  const kr::vec3<scalar_t> &angles = kr::getRPY(diff.q.matrix());
  if (std::abs(angles[2] * 180 / M_PI) > config_.kf_yaw_thresh) {
    //  over yaw angle threshold, add keyframe
    return true;
  }

  const size_t min_corners =
      std::ceil(config_.kf_min_filled * config_.shi_max_corners);
  if (num_corners < min_corners) {
    //  insufficent features, add keyframe with new ones
    return true;
  }

  return false;
}

void StereoVo::AddKeyFrame(const Pose &pose, const CvStereoImage &stereo_image,
                           std::vector<Corner> &corners) {
  /// @todo: create observations and add to key frame
  // Detect new corners based on distribution/number of current corners
  // Mark current corners as old corners
  std::vector<Corner> new_corners;
  detector_.AddCorners(stereo_image.first, corners, new_corners);
  // Track new corners from left image to right image and return corresponding
  // points on the right image. Corners will be removed from new corners if they
  // are lost during tracking
  std::vector<CvPoint2> right_points;
  TrackSpatial(stereo_image, new_corners, right_points);
  // Triangulate will change both new_corners and features based on the
  // triangulation results
  Triangulate(new_corners, right_points);
  // Add new corners to corners
  corners.insert(corners.end(), new_corners.begin(), new_corners.end());

  if (key_frames_.empty()) {
    //  first keyframe, calculate depth
    double depth = 0;

    size_t count = 0;
    for (const Corner &corner : corners) {
      auto ite = features_.find(corner.id());
      if (ite != features_.end()) {
        const Feature &feat = ite->second;
        // depth +=
        count++;
      }
    }

    absolute_pose_.q = kr::quat<scalar_t>(0, 1, 0, 0);
    absolute_pose_.p = kr::vec3<scalar_t>(0, 0, depth);
  }
  // Add key frame to queue with current_pose, features and stereo_image
  key_frames_.emplace_back(pose, corners, stereo_image);
}

void StereoVo::Triangulate(std::vector<Corner> &corners,
                           std::vector<CvPoint2> &points) {
  ;
}

void StereoVo::TrackSpatial(const CvStereoImage &stereo_image,
                            std::vector<Corner> &corners,
                            std::vector<CvPoint2> &r_points) {
  // Put pixel of corners into l_points
  std::vector<CvPoint2> l_points;
  for (const Corner &corner : corners) l_points.push_back(corner.p_pixel());
  std::vector<uchar> status;
  // LK tracker
  OpticalFlow(stereo_image.first, stereo_image.second, l_points, r_points,
              status);
  PruneByStatus(status, l_points);
  PruneByStatus(status, r_points);
  PruneByStatus(status, corners);
  status.clear();
  // Find fundamental matrix
  cv::findFundamentalMat(l_points, r_points, cv::FM_RANSAC, 1.5, 0.99, status);
  PruneByStatus(status, r_points);
  PruneByStatus(status, corners);
  // Verify that outputs have the same size
  ROS_ASSERT_MSG(corners.size() == r_points.size(),
                 "TrackSpatial Dimension mismatch");
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

bool StereoVo::TriangulatePoint(const CvPoint2 &left, const CvPoint2 &right,
                                CvPoint3 &output) {
  //  camera model
  const scalar_t lfx = model_.left().fx(), lfy = model_.left().fy();
  const scalar_t lcx = model_.left().cx(), lcy = model_.left().cy();
  const scalar_t rfx = model_.right().fx(), rfy = model_.right().fy();
  const scalar_t rcx = model_.right().cx(), rcy = model_.right().cy();

  Pose poseLeft;   //  identity
  Pose poseRight;  //  shifted right along x
  poseRight.p[0] = model_.baseline();

  //  normalized coordinates
  kr::vec2<scalar_t> lPt((left.x - lcx) / lfx, (left.y - lcy) / lfy);
  kr::vec2<scalar_t> rPt((right.x - rcx) / rfx, (right.y - rcy) / rfy);

  kr::vec3<scalar_t> p3D;
  scalar_t ratio;

  kr::triangulate(poseLeft, lPt, poseRight, rPt, p3D, ratio);

  if (ratio > config_.tri_max_eigenratio) {
    return false;
  }

  //  point is valid, refine it some more
  std::vector<Pose> poses({poseLeft, poseRight});
  std::vector<kr::vec2<scalar_t>> obvs({lPt, rPt});

  if (!kr::refinePoint(poses, obvs, p3D)) {
    return false;
  }

  output.x = p3D[0];
  output.y = p3D[1];
  output.z = p3D[2];
  return true;
}

}  // namespace stereo_vo

}  // namespace galt
