#include "stereo_vo/stereo_vo.h"
#include "stereo_vo/feature.h"
#include "stereo_vo/frame.h"
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
  FramePtr curr_frame = std::make_shared<Frame>(stereo_image);
  AddKeyFrame(curr_frame);

  // Save frame for next iteration
  prev_frame_ = curr_frame;

  // Create a window for display
  /// @todo: replace this with published topic later
  cv::namedWindow("display",
                  CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  init_ = true;
  ROS_INFO_STREAM("StereVo initialized, baseline: " << model_.baseline());
}

void StereoVo::Iterate(const CvStereoImage &stereo_image) {
  // Construct a new frame based on incoming stereo image
  FramePtr curr_frame = std::make_shared<Frame>(stereo_image);
  // Track corners from previous frame into current frame
  // Remove corresponding corners in last key frame only if they are newly
  // initialized
  TrackTemporal(prev_frame_, curr_frame, prev_key_frame());

  // Estimate pose using 2D-to-3D correspondences
  // 2D - currently tracked corners
  // 3D - points triangulated in world frame
  EstimatePose(curr_frame, point3ds_);

  // Check whether to add key frame based on the following criteria
  // 1. Movement exceeds config_.kf_dist_thresh
  // 2. Yaw angle exceeds config_.kf_yaw_thresh
  // 3. Number of features falls below threshold (see ShouldAddKeyFrame)
  // After this step, tracked_corners will contain both old corners and newly
  // added corners
  if (ShouldAddKeyFrame(curr_frame)) {
    AddKeyFrame(curr_frame);
  }

  // Do a windowed optimization with gtsam if window size is reached

  // Visualization (optional)
  Display(curr_frame, prev_key_frame());

  prev_frame_ = curr_frame;
}

void StereoVo::TrackTemporal(const FramePtr &frame1, const FramePtr &frame2,
                             const FramePtr &key_frame) {
  TrackTemporal(frame1->l_image(), frame2->l_image(), frame1->features(),
                frame2->features(), key_frame);
}

void StereoVo::TrackTemporal(const cv::Mat &image1, const cv::Mat &image2,
                             const std::vector<Feature> &features1,
                             std::vector<Feature> &features2,
                             const FramePtr &key_frame) {
  std::vector<CvPoint2> corners2;
  std::vector<uchar> status;
  std::set<Id> ids_to_remove;

  std::vector<Id> ids = ExtractIds(features1);
  std::vector<CvPoint2> corners1 = ExtractCorners(features1);

  // Track and remove mismatches
  OpticalFlow(image1, image2, corners1, corners2, status);
  PruneByStatus(status, ids, ids_to_remove);
  PruneByStatus(status, corners1);
  PruneByStatus(status, corners2);
  status.clear();

  // Find fundamental matrix to reject outliers in tracking
  FindFundamentalMat(corners1, corners2, status);
  PruneByStatus(status, ids, ids_to_remove);
  PruneByStatus(status, corners2);

  // Verify that ids is of the same dimension as points_tracked
  ROS_ASSERT_MSG(ids.size() == corners2.size(),
                 "TrackSpatial Dimension mismatch");

  // Remove singlet observations from previous keyframe as required
  key_frame->RemoveById(ids_to_remove);

  auto it_pts = corners2.cbegin();
  for (auto it_id = ids.cbegin(), ite_id = ids.cend(); it_id != ite_id;
       ++it_id, ++it_pts) {
    features2.emplace_back(*it_id, *it_pts, false);
  }
}

void StereoVo::EstimatePose(const FramePtr &frame,
                            const std::map<Id, Point3d> point3ds) const {
  const size_t n_features = frame->num_feautres();

  std::vector<CvPoint2> image_points;
  std::vector<CvPoint3> world_points;
  std::vector<uchar> inliers;

  image_points.reserve(n_features);
  world_points.reserve(n_features);

  for (const Feature &feature : frame->features()) {
    const auto it = point3ds.find(feature.id());
    if (it != point3ds.end()) {
      const Point3d &point3d = it->second;
      image_points.push_back(feature.p_pixel());
      world_points.push_back(point3d.p_world());
    }
  }

  if (image_points.empty()) {
    throw std::runtime_error("EstimatePose called with empty features");
  }

  static cv::Mat rvec = cv::Mat::zeros(3, 3, CV_64FC1);
  static cv::Mat tvec = cv::Mat::zeros(3, 3, CV_64FC1);

  const size_t minInliers =
      std::ceil(world_points.size() * config_.pnp_ransac_inliers);
  cv::solvePnPRansac(world_points, image_points,
                     model_.left().fullIntrinsicMatrix(), std::vector<double>(),
                     rvec, tvec, false, 100, config_.pnp_ransac_error,
                     minInliers, inliers, cv::ITERATIVE);

  kr::vec3<scalar_t> r(rvec.at<double>(0, 0), rvec.at<double>(1, 0),
                       rvec.at<double>(2, 0));
  kr::vec3<scalar_t> t(tvec.at<double>(0, 0), tvec.at<double>(1, 0),
                       tvec.at<double>(2, 0));

  // This is now absolute pose
  frame->set_pose(Pose::fromVectors(r, t));
}

bool StereoVo::ShouldAddKeyFrame(const FramePtr &frame) const {
  // Not initialized, thus no keyframes, add one
  if (!init_) {
    return true;
  }

  const Pose &diff = frame->pose().difference(prev_key_frame()->pose());
  if (diff.p.norm() > config_.kf_dist_thresh) {
    ROS_INFO("Distance: %f", diff.p.norm());
    //  over distance threshold, add keyframe
    return true;
  }

  const kr::vec3<scalar_t> &angles = kr::getRPY(diff.q.matrix());
  if (std::abs(angles[2] * 180 / M_PI) > config_.kf_yaw_thresh) {
    ROS_INFO("Angle: %f", angles[2]);
    //  over yaw angle threshold, add keyframe
    return true;
  }

  const size_t min_features =
      std::ceil(config_.kf_min_filled * config_.shi_max_corners);
  if (frame->num_feautres() < min_features) {
    ROS_INFO("Corners: %i", (int)frame->num_feautres());
    //  insufficent features, add keyframe with new ones
    return true;
  }

  return false;
}

void StereoVo::AddKeyFrame(FramePtr &frame) {
  // Detect new corners and add to features in current frame
  std::vector<Feature> &features = frame->features();
  size_t num_new_features = detector_.AddFeatures(frame->l_image(), features);
  ROS_INFO("new corners detected: %d", static_cast<int>(num_new_features));
  // Track features into right image
  std::vector<CvPoint2> right_corners;
  TrackSpatial(frame->stereo_image(), features, right_corners);

  if (!init_) {
    //  make up a pose that looks pretty
    Pose init_pose(kr::quat<scalar_t>(0, 1, 0, 0),
                   kr::vec3<scalar_t>(0, 0, 10));
    frame->set_pose(init_pose);
  }

  // Retriangulate in current pose
  Triangulate(frame->pose(), features, right_corners);

  frame->SetKeyFrame();
  // Add key frame to queue with current_pose, features and stereo_image
  key_frames_.push_back(frame);
}

void StereoVo::TrackSpatial(const CvStereoImage &stereo_image,
                            std::vector<Feature> &features,
                            std::vector<CvPoint2> &r_corners) {
  // Put pixel of corners into l_points
  std::vector<CvPoint2> l_corners = ExtractCorners(features);
  std::vector<uchar> status;

  // LK tracker
  OpticalFlow(stereo_image.first, stereo_image.second, l_corners, r_corners,
              status);
  PruneByStatus(status, l_corners);
  PruneByStatus(status, r_corners);
  PruneByStatus(status, features);
  status.clear();

  // Find fundamental matrix
  if (l_corners.empty()) {
    ROS_WARN("OpticalFlow failed to track any features");
    return;
  }
  FindFundamentalMat(l_corners, r_corners, status);
  PruneByStatus(status, r_corners);
  PruneByStatus(status, features);

  // Verify that outputs have the same size
  ROS_ASSERT_MSG(features.size() == r_corners.size(),
                 "TrackSpatial Dimension mismatch");
}

void StereoVo::Triangulate(const Pose &pose, std::vector<Feature> &features,
                           std::vector<CvPoint2> &corners) {
  auto it_corner = corners.begin();
  for (auto it_feature = features.begin(); it_feature != features.end();
       ++it_feature, ++it_corner) {
    // Re-triangulate
    kr::vec3<scalar_t> p3;
    Feature &feature = *it_feature;
    if (!TriangulatePoint(feature.p_pixel(), *it_corner, p3)) {
      // Failed erase from corners
      it_feature = features.erase(it_feature);
      it_corner = corners.erase(it_corner);
      continue;
    }
    // Convert to world frame
    p3 = pose.q.conjugate().matrix() * p3 + pose.p;
    /// @note: I feel like since we are using a map, it's not necessary to check
    /// if that point is in the map or not. Merely index into the map would
    /// either create or update that point, right?
    const Id &id = it_feature->id();
    point3ds_[id] = Point3d(id, CvPoint3(p3[0], p3[1], p3[2]));
  }
}

void StereoVo::OpticalFlow(const cv::Mat &image1, const cv::Mat &image2,
                           const std::vector<CvPoint2> &points1,
                           std::vector<CvPoint2> &points2,
                           std::vector<uchar> &status) {
  if (points1.empty()) {
    //  don't let calc optical flow assert
    ROS_WARN("OpticalFlow() called with no points");
    return;
  }
  int win_size = config_.klt_win_size;
  int max_level = config_.klt_max_level;
  static cv::TermCriteria term_criteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.001);
  cv::calcOpticalFlowPyrLK(image1, image2, points1, points2, status,
                           cv::noArray(), cv::Size(win_size, win_size),
                           max_level, term_criteria);
}

void StereoVo::FindFundamentalMat(const std::vector<CvPoint2> &points1,
                                  const std::vector<CvPoint2> &points2,
                                  std::vector<uchar> &status) {
  cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, 1, 0.99, status);
}

bool StereoVo::TriangulatePoint(const CvPoint2 &left, const CvPoint2 &right,
                                kr::vec3<scalar_t> &p_cam) {
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

  scalar_t ratio;
  kr::triangulate(poseLeft, lPt, poseRight, rPt, p_cam, ratio);

  if (ratio > config_.tri_max_eigenratio) {
    return false;
  }

  //  point is valid, refine it some more
  std::vector<Pose> poses({poseLeft, poseRight});
  std::vector<kr::vec2<scalar_t>> obvs({lPt, rPt});

  if (!kr::refinePoint(poses, obvs, p_cam)) {
    return false;
  }

  /// @note: I moved convert to world coordinates to Triangulate
  //  convert to world coordinates
  //  p3D = pose.q.conjugate().matrix() * p3D + pose.p;

  //  point3d.x = p3D[0];
  //  point3d.y = p3D[1];
  //  point3d.z = p3D[2];
  return true;
}

}  // namespace stereo_vo

}  // namespace galt
