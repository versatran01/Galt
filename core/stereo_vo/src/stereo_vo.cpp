#include "stereo_vo/stereo_vo.h"
#include "stereo_vo/feature.h"
#include "stereo_vo/keyframe.h"
#include "stereo_vo/utils.h"

#include <image_geometry/stereo_camera_model.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <kr_math/base_types.hpp>
#include <kr_math/feature.hpp>
#include <kr_math/pose.hpp>
#include <kr_math/SO3.hpp>

namespace galt {

namespace stereo_vo {

using image_geometry::StereoCameraModel;

void StereoVo::Initialize(const CvStereoImage& stereo_image,
                          const StereoCameraModel& model) {
  ROS_INFO("Initializing stereo_vo");
  // Don't initialize if initial pose is not set
  if (!init_pose()) {
    ROS_INFO("Initial pose not set, stop initialization");
    return;
  }

  model_ = model;
  // Add the first stereo image as first keyframe
  //  FramePtr curr_frame = boost::make_shared<Frame>(stereo_image);
  // AddKeyFrame(curr_frame);
  // AddKeyFrame(iamges, corners);

  // Save frame for next iteration
  // prev_frame_ = curr_frame;

  //  CheckEverything();
  //  ROS_WARN("Passed check everything for initializing");

  init_ = true;
  ROS_INFO_STREAM("StereoVo initialized, baseline: " << model_.baseline());
}

bool StereoVo::ShouldAddKeyFrame() const {
  // Not initialized, thus no keyframes, add one
  if (!init_) {
    return true;
  }

  if (temporal_tracker_.features().size() < 30) {
    //  insufficient features
    return true;
  }

  const KeyFramePtr prev_key_frame = key_frames_.back();
  
  const KrPose &diff = pose().difference(prev_key_frame->pose());
  if (diff.p().norm() > config_.kf_dist_thresh) {
    ROS_INFO("Distance: %f", diff.p().norm());
    //  over distance threshold, add keyframe
    return true;
  }

  //  const kr::vec3<scalar_t> &angles = kr::getRPY(diff.bRw());
  //  if (std::abs(angles[2] * 180 / M_PI) > config_.kf_yaw_thresh) {
  //    ROS_INFO("Angle: %f", angles[2]);
  //    //  over yaw angle threshold, add keyframe
  //    return true;
  //  }

  return false;
}

void StereoVo::AddKeyFrame(const CvStereoImage& stereo_image) {
  const bool first_frame = key_frames_.empty();
  
  //  capture the current pose and create new key frame
  const KrPose current_pose = pose();
  KeyFramePtr ptr = std::make_shared<KeyFrame>(stereo_image);
  ptr->set_pose(current_pose);

  //  add more features using the grid pattern
  const std::vector<Feature>& features = temporal_tracker_.features();
  const std::vector<Feature> new_features =
      detector_.AddFeatures(stereo_image.first, features);

  //  track new features left -> right
  spatial_tracker_.Reset();
  spatial_tracker_.AddFeatures(new_features);
  std::vector<Feature> left_features;
  spatial_tracker_.Track(stereo_image.first, stereo_image.second,
                         left_features);

  //  left_features and right_features now contain matching points
  std::vector<Feature> right_features = spatial_tracker_.features();

  //  create point 3ds for new features
  auto ite_l = left_features.begin();
  auto ite_r = right_features.begin();

  size_t added_count = 0;
  while (ite_l != left_features.end()) {
    const Id id = ite_l->id();
    const CvPoint2& left = ite_l->p_pixel();
    const CvPoint2& right = ite_r->p_pixel();

    Point3d point(id);
    if (point.AddObservation(config_, model_, ptr, left, right)) {
      //  sanity check
      assert(points_.find(id) == points_.end());
      if (first_frame) {
        //  mark first key-frame features as inliers
        point.set_is_inlier(true);
      }
      points_[id] = point;
      ite_l++;
      ite_r++;
      added_count++;
    } else {
      //  bad initial triangulation, toss it out
      ite_l = left_features.erase(ite_l);
      ite_r = right_features.erase(ite_r);
    }
  }
  ROS_INFO("Initialized %lu features", added_count);

#ifdef DEBUG_GUI
  //  display left and right images with matched features
  //  assume same dimensions here
  const int rows = stereo_image.first.rows;
  const int cols = stereo_image.first.cols;
  cv::Mat images(rows, cols * 2, CV_8UC1);

  stereo_image.first.copyTo(images(cv::Rect(0, 0, cols, rows)));
  stereo_image.second.copyTo(images(cv::Rect(cols, 0, cols, rows)));

  for (size_t i = 0; i < left_features.size(); i++) {
    const CvPoint2& left = left_features[i].p_pixel();
    const CvPoint2& right = right_features[i].p_pixel();

    cv::circle(images, left, 3, cv::Scalar(255, 0, 0), 1);
    cv::circle(images, CvPoint2(right.x + cols, right.y), 3,
               cv::Scalar(255, 0, 0), 1);
    cv::line(images, left, CvPoint2(right.x + cols, right.y),
             cv::Scalar(0, 0, 255));
  }
  cv::imshow("stereo_pair", images);
  cv::waitKey(1);
#endif

  //  update old point 3ds with new pose
  for (const Feature& f : features) {
    std::map<Id, Point3d>::iterator ite = points_.find(f.id());
    assert(ite != points_.end());
    Point3d& p3d = ite->second;
    //  add only left p_pixel here
    assert(p3d.is_initialized());
    if (!p3d.is_inlier()) {
      p3d.AddObservation(config_,model_,ptr,f.p_pixel());
    }
  }

  //  add the new left features to the collection of trackables
  temporal_tracker_.AddFeatures(left_features);
  key_frames_.push_back(ptr);
}

// void StereoVo::TrackSpatial(const CvStereoImage &stereo_image,
//                            std::vector<Feature> &features,
//                            std::vector<CvPoint2> &r_corners) {
//  // Put pixel of corners into l_points
//  std::vector<CvPoint2> l_corners = ExtractCorners(features);
//  std::vector<uchar> status;

//  // LK tracker
//  OpticalFlow(stereo_image.first, stereo_image.second, l_corners, r_corners,
//              status);
//  PruneByStatus(status, l_corners);
//  PruneByStatus(status, r_corners);
//  PruneByStatus(status, features);
//  status.clear();

//  // Find fundamental matrix
//  if (l_corners.empty()) {
//    ROS_WARN("OpticalFlow() failed to track any features");
//    return;
//  }
//  FindFundamentalMat(l_corners, r_corners, status);
//  PruneByStatus(status, r_corners);
//  PruneByStatus(status, features);

//  // Verify that outputs have the same size
//  ROS_ASSERT_MSG(features.size() == r_corners.size(),
//                 "TrackSpatial Dimension mismatch");
//}

// void StereoVo::OpticalFlow(const cv::Mat &image1, const cv::Mat &image2,
//                           const std::vector<CvPoint2> &points1,
//                           std::vector<CvPoint2> &points2,
//                           std::vector<uchar> &status) {
//  if (points1.empty()) {
//    ROS_WARN("OpticalFlow() called with no points");
//    return;
//  }
//  int win_size = config_.klt_win_size;
//  int max_level = config_.klt_max_level;
//  static cv::TermCriteria term_criteria(
//      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 10, 0.005);
//  cv::calcOpticalFlowPyrLK(image1, image2, points1, points2, status,
//                           cv::noArray(), cv::Size(win_size, win_size),
//                           max_level, term_criteria);
//}

// void StereoVo::FindFundamentalMat(const std::vector<CvPoint2> &points1,
//                                  const std::vector<CvPoint2> &points2,
//                                  std::vector<uchar> &status) {
//  if (points1.empty()) {
//    ROS_WARN("FindFundamentalMat() called with no points");
//    return;
//  }
//  cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, 0.5, 0.999, status);
//}

void StereoVo::Iterate(const CvStereoImage& stereo_image) {
  if (!prev_left_image_.empty()) {
    //  track from previous frame
    std::vector<Feature> unused;
    temporal_tracker_.Track(prev_left_image_, stereo_image.first, unused);
  }

  /*
  cv::Mat debug_image;
  cv::cvtColor(stereo_image.first, debug_image, CV_GRAY2RGB);
  //  draw the tracked features
  for (const Feature& f : temporal_tracker_.features()) {
    cv::circle(debug_image, f.p_pixel(), 3, cv::Scalar(255, 0, 0), 1);
  }
  cv::imshow("derp derp", debug_image);
  cv::waitKey(1);
  */

  //  ransac PnP
  EstimatePose();

  //  add keyframe: extract features, match, triangulate, insert into map
  if (ShouldAddKeyFrame()) {
    AddKeyFrame(stereo_image);
  }

  /// @todo: do visualization
  Display(stereo_image.first, spatial_tracker_.features(),
          temporal_tracker_.features());

  prev_left_image_ = stereo_image.first;
}

void StereoVo::EstimatePose() {
  std::vector<CvPoint2> pixel_points;
  std::vector<CvPoint3> world_points;

  //  iterate over all presently visible features
  for (const Feature& feature : temporal_tracker_.features()) {
    //  retrieve corresponding 3D point
    std::map<Id, Point3d>::iterator ite_pt = points_.find(feature.id());
    assert(ite_pt != points_.end());
    const Point3d& point3d = ite_pt->second;

    if (point3d.is_initialized()) {// && point3d.is_inlier()) {
      pixel_points.push_back(feature.p_pixel());
      world_points.push_back(point3d.p_world());
    }
  }

  const size_t N = pixel_points.size();
  if (N < 4) {
    //  need at least 4 points for PnP
    ROS_WARN("Insufficient points for PnP - skipping");
    return;
  }

  cv::Mat rvec = cv::Mat::zeros(3, 3, CV_64FC1);
  cv::Mat tvec = cv::Mat::zeros(3, 3, CV_64FC1);

  const size_t min_inliers = std::ceil(N * config_.pnp_ransac_inliers);
  cv::solvePnPRansac(world_points, pixel_points,
                     model_.left().fullIntrinsicMatrix(), std::vector<double>(),
                     rvec, tvec, false, 200, config_.pnp_ransac_error,
                     min_inliers);

  const vec3 r(rvec.at<double>(0, 0), rvec.at<double>(1, 0),
               rvec.at<double>(2, 0));
  const vec3 t(tvec.at<double>(0, 0), tvec.at<double>(1, 0),
               tvec.at<double>(2, 0));
  KrPose new_pose = KrPose::fromVectors(r, t);

  /// @todo: some kind of sanity check here

  //  const bool shenanigans =
  //      new_pose.difference(old_pose).p().norm() > config_.pnp_motion_thresh;
  //  if ((t[0] == 0 && t[1] == 0 && t[2] == 0) || shenanigans) {
  //    ROS_WARN("Probable shenanigans in ransac PnP - trying regular PnP");
  //    cv::solvePnP(world_points, image_points,
  //                 model_.left().fullIntrinsicMatrix(), std::vector<double>(),
  //                 rvec, tvec, false);

  //    r = kr::vec3<scalar_t>(rvec.at<double>(0, 0), rvec.at<double>(1, 0),
  //                           rvec.at<double>(2, 0));
  //    t = kr::vec3<scalar_t>(tvec.at<double>(0, 0), tvec.at<double>(1, 0),
  //                           tvec.at<double>(2, 0));
  //    new_pose = KrPose::fromVectors(r, t);
  //  }

  pose_ = new_pose;
}

/*
void StereoVo::CheckEverything() {
  std::map<Id, int> feat_counts;
  std::set<Id> kf_ids;
  for (const FramePtr &kf_ptr : key_frames_) {
    const Frame &frame = *kf_ptr;
    //  make sure there are no duplicate frame ids
    ROS_ASSERT_MSG(kf_ids.find(frame.id()) == kf_ids.end(),
                   "Duplicate frame IDs found");
    kf_ids.insert(frame.id());

    std::set<Id> point_ids;
    for (const Feature &feature : frame.features()) {
      //  make sure there are no duplicate features in this frame
      ROS_ASSERT_MSG(point_ids.find(feature.id()) == point_ids.end(),
                     "Duplicate feature (%lu) found in frame %lu", feature.id(),
                     frame.id());
      point_ids.insert(feature.id());

      //  features must be triangulated
      ROS_ASSERT_MSG(point3ds_.find(feature.id()) != point3ds_.end(),
                     "Missing triangulation for feature %lu", feature.id());

      if (feat_counts.find(feature.id()) == feat_counts.end()) {
        feat_counts[feature.id()] = 1;
      } else {
        feat_counts[feature.id()]++;
      }
    }
  }
}
*/
/*
void StereoVo::TrackTemporal(const FramePtr &frame1, const FramePtr &frame2,
                             const FramePtr &key_frame) {
  if (frame1->features().empty()) {
    ROS_WARN("No features to track temporally, skipping");
  } else {
    TrackTemporal(frame1->l_image(), frame2->l_image(), frame1->features(),
                  frame2->features(), key_frame);
  }
}
*/
/*
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
  const size_t num_in = corners1.size();
  const size_t num_out = std::count(status.begin(), status.end(), uchar(1));
  PruneByStatus(status, ids, ids_to_remove);
  PruneByStatus(status, corners1);
  PruneByStatus(status, corners2);
  status.clear();

  // Find fundamental matrix to reject outliers in tracking
  FindFundamentalMat(corners1, corners2, status);
  const size_t num_in2 = corners1.size();
  const size_t num_out2 = std::count(status.begin(), status.end(), uchar(1));
  if (num_out2 == 0) {
    ROS_WARN("Fundamental mat found no inliers, ignoring it.");
  } else {
    PruneByStatus(status, ids, ids_to_remove);
    PruneByStatus(status, corners2);
  }

  if (corners2.empty()) {
    ROS_WARN("Corners2 is empty! (%lu, %lu) and (%lu, %lu)", num_in, num_out,
             num_in2, num_out2);
  }

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
*/

/*
void StereoVo::Triangulate(const KrPose &pose, std::vector<Feature> &features,
                           std::vector<CvPoint2> &corners) {
  auto it_corner = corners.begin();
  for (auto it_feature = features.begin(); it_feature != features.end();) {
    // Re-triangulate
    kr::vec3<scalar_t> p3;
    Feature &feature = *it_feature;
    if (!TriangulatePoint(feature.p_pixel(), *it_corner, p3)) {
      // Failed erase from corners
      it_feature = features.erase(it_feature);
      it_corner = corners.erase(it_corner);
    } else {
      // Convert to world frame
      p3 = pose.wRb() * p3 + pose.p();

      ++it_feature;
      ++it_corner;
    }
  }
}
*/

/*
bool StereoVo::TriangulatePoint(const CvPoint2 &left, const CvPoint2 &right,
                                kr::vec3<scalar_t> &p_cam) {
  //  camera model
  const scalar_t lfx = model_.left().fx(), lfy = model_.left().fy();
  const scalar_t lcx = model_.left().cx(), lcy = model_.left().cy();
  const scalar_t rfx = model_.right().fx(), rfy = model_.right().fy();
  const scalar_t rcx = model_.right().cx(), rcy = model_.right().cy();

  KrPose poseLeft;   //  identity
  KrPose poseRight;  //  shifted right along x
  poseRight.p()[0] = model_.baseline();

  //  normalized coordinates
  kr::vec2<scalar_t> lPt((left.x - lcx) / lfx, (left.y - lcy) / lfy);
  kr::vec2<scalar_t> rPt((right.x - rcx) / rfx, (right.y - rcy) / rfy);

  scalar_t ratio;
  kr::triangulate(poseLeft, lPt, poseRight, rPt, p_cam, ratio);

  if (ratio > config_.tri_max_eigenratio) {
    return false;
  }

  //  point is valid, refine it some more
  std::vector<KrPose> poses({poseLeft, poseRight});
  std::vector<kr::vec2<scalar_t>> obvs({lPt, rPt});

  if (!kr::refinePoint(poses, obvs, p_cam)) {
    return false;
  }

  return true;
}
*/

/*
void StereoVo::NukeOutliers() {

  struct PointMetric {
    scalar_t err_left_2;
    int count;

    scalar_t norm_err_left_2() const { return err_left_2 / count; }
  };
  std::map<Id, PointMetric> metrics;

  const auto fx = model_.left().fx();
  const auto fy = model_.left().fy();
  const auto cx = model_.left().cx();
  const auto cy = model_.left().cy();

  //  consider all key frames
  for (const FramePtr &ptr : key_frames_) {
    const Frame &frame = *ptr;
    const KrPose &pose = frame.pose();

    for (const Feature &feat : frame.features()) {
      const Point3d &p3d = point3ds_[feat.id()];
      const CvPoint2 &p2d = feat.p_pixel();

      //  project point
      kr::vec3<scalar_t> p_cam(p3d.p_world().x, p3d.p_world().y,
                               p3d.p_world().z);
      p_cam = pose.transformToBody(p_cam);
      p_cam /= p_cam[2];
      //  apply camera model
      p_cam[0] = fx * p_cam[0] + cx;
      p_cam[1] = fy * p_cam[1] + cy;

      //  reprojection error squared
      auto err2 = (p_cam[0] - p2d.x) * (p_cam[0] - p2d.x) +
                  (p_cam[1] - p2d.y) * (p_cam[1] - p2d.y);

      auto ite = metrics.find(feat.id());
      if (ite == metrics.end()) {
        PointMetric pm;
        pm.err_left_2 = err2;
        pm.count = 1;
        metrics[feat.id()] = pm;
      } else {
        PointMetric &pm = ite->second;
        pm.err_left_2 += err2;
        pm.count++;
      }
    }
  }

  struct Stats {
    scalar_t mean;
    scalar_t mean_squared;
    int count;
    Stats() {
      mean = 0;
      mean_squared = 0;
      count = 0;
    }
    void add(scalar_t x) {
      mean = (mean * count + x) / (count + 1);
      mean_squared = (mean_squared * count + x * x) / (count + 1);
      count++;
    }
    scalar_t variance() const { return mean_squared - mean * mean; }
  };
  std::map<int, Stats> sorted_stats;

  //  calculate mean squared error
  for (const std::pair<Id, PointMetric> &pair : metrics) {

    Stats &stats = sorted_stats[pair.second.count];  //  organize by count

    const auto err2 = pair.second.norm_err_left_2();
    stats.add(err2);
  }

  for (const std::pair<int, Stats> &pair : sorted_stats) {
    ROS_INFO("MSPE,STD (count = %i): %f, %f", pair.first, pair.second.mean,
             std::sqrt(pair.second.variance()));
  }

  //  build a set of features to eliminate
  std::set<Id> removables;
  for (const std::pair<Id, PointMetric> &pair : metrics) {
    if (pair.second.norm_err_left_2() > 100) {
      removables.insert(pair.first);
    }
  }

  if (!removables.empty()) {
    ROS_INFO("Nuking %lu outliers", removables.size());
  }

  //  remove them
  size_t erased=0;
  for (FramePtr ptr : key_frames_) {
    erased += ptr->RemoveById(removables, true);
  }
  if (erased) {
   ROS_INFO("Erased %lu observations", erased);
  }
}
*/

}  // namespace stereo_vo

}  // namespace galt
