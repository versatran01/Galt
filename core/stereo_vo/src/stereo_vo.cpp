#include "stereo_vo/stereo_vo.h"
#include "stereo_vo/feature.h"
#include "stereo_vo/frame.h"
#include "stereo_vo/utils.h"

#include <image_geometry/stereo_camera_model.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"

#include <kr_math/feature.hpp>

namespace galt {
namespace stereo_vo {

using image_geometry::StereoCameraModel;

void StereoVo::Initialize(const CvStereoImage &stereo_image,
                          const StereoCameraModel &model) {
  ROS_INFO("Initializing stereo_vo");

  model_ = model;

  if (!InitializePose()) {
    ROS_WARN_THROTTLE(1, "Failed to initialize first pose");
    return;
  }

  if (!AddKeyFrame(w_T_c(), stereo_image, tracked_features_)) {
    ROS_WARN_THROTTLE(1, "Failed to add the first frame as key frame");
    return;
  }

  prev_stereo_ = stereo_image;

  init_ = true;
  ROS_INFO_STREAM("StereoVo initialized, baseline: " << model_.baseline());
}

void StereoVo::Iterate(const CvStereoImage &stereo_image,
                       const ros::Time &time) {
  TrackTemporal(prev_stereo_.first, stereo_image.first, tracked_features_);

  Display(stereo_image, tracked_features_, key_frames_.back());

  if (ShouldAddKeyFrame()) {
    AddKeyFrame(w_T_c(), stereo_image, tracked_features_);
    PublishStereoFeatures(key_frames_.back(), time);
  }

  prev_stereo_ = stereo_image;
}

bool StereoVo::InitializePose() {
  // For now just initialize to identity, later will use tf2 to listen to a
  // transform
  ROS_INFO("First camera pose initialized to:");
  std::cout << w_T_c_.matrix() << std::endl;
  return true;
}

bool StereoVo::ShouldAddKeyFrame() const {
  // For now simply add a key frame if tracked features is less than 20
  return (tracked_features_.size() < 30);
}

bool StereoVo::AddKeyFrame(const KrPose &pose,
                           const CvStereoImage &stereo_image,
                           std::vector<Feature> &features) {
  // Detect new corners based on current feature distribution
  std::vector<CvPoint2> l_corners =
      detector_.AddFeatures(stereo_image.first, features);
  std::vector<CvPoint2> r_corners;
  // Track new corners from left to right so that we can triangulate them
  // Mis-tracked corners will be removed from this step
  TrackSpatial(stereo_image.first, stereo_image.second, l_corners, r_corners);

  if (l_corners.empty()) {
    ROS_WARN("No new corners detected");
    return false;
  }
  
  // Triangulate and add corners to features
  auto ite_l = l_corners.begin(), ite_r = r_corners.begin();
  while (ite_l != l_corners.end()) {
    const CvPoint2& lp = *ite_l;
    const CvPoint2& rp = *ite_r;
    
    kr::vec3<scalar_t> p_cam;
    kr::mat3<scalar_t> tri_cov;
    if ( TriangulatePoint(lp,rp,p_cam,tri_cov) ) {
      //  triangulation was good, create a fresh feature
      //  first calculate world coordinates
      kr::vec3<scalar_t> p_world = pose.transformFromBody(p_cam);
      
      Feature new_feature(lp);
      new_feature.p_cam = CvPoint3(p_cam[0],p_cam[1],p_cam[2]);
      new_feature.p_world = CvPoint3(p_world[0],p_world[1],p_world[2]);
      new_feature.p_cov_cam = tri_cov;
      //ROS_INFO_STREAM("Triangulated cov: " << tri_cov);
      
      features.push_back(new_feature);
      ite_l++;
      ite_r++;
    } else {
      //  bad triangulatio, do not retain this corner
      ite_l = l_corners.erase(ite_l);
      ite_r = r_corners.erase(ite_r);
    }
  }
  
  features.insert(features.end(), l_corners.cbegin(), l_corners.cend());
  // Add a key frame
  key_frames_.emplace_back(pose, stereo_image, features, r_corners);
  ROS_INFO("new corners added: %lu", l_corners.size());

  return true;
}

void StereoVo::TrackSpatial(const cv::Mat &image1, const cv::Mat &image2,
                            std::vector<CvPoint2> &l_corners,
                            std::vector<CvPoint2> &r_corners) {
  std::vector<uchar> status;
  OpticalFlow(image1, image2, l_corners, r_corners, status);
  PruneByStatus(status, l_corners);
  PruneByStatus(status, r_corners);
  ROS_ASSERT_MSG(l_corners.size() == r_corners.size(),
                 "Corners size mismatch after PruneByStatus1");
  status.clear();
  FindFundamentalMat(l_corners, r_corners, status);
  PruneByStatus(status, l_corners);
  PruneByStatus(status, r_corners);
  ROS_ASSERT_MSG(l_corners.size() == r_corners.size(),
                 "Corners size mismatch after PruneByStatus2");
}

void StereoVo::OpticalFlow(const cv::Mat &image1, const cv::Mat &image2,
                           const std::vector<CvPoint2> &points1,
                           std::vector<CvPoint2> &points2,
                           std::vector<uchar> &status) {
  if (points1.empty()) {
    ROS_WARN("OpticalFlow() called with no points");
    return;
  }
  int win_size = config_.klt_win_size;
  int max_level = config_.klt_max_level;
  const static cv::TermCriteria term_criteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 10, 0.005);
  cv::calcOpticalFlowPyrLK(image1, image2, points1, points2, status,
                           cv::noArray(), cv::Size(win_size, win_size),
                           max_level, term_criteria);
  ROS_ASSERT_MSG(points1.size() == points2.size(),
                 "Corners size mismatch during OpticalFlow()");
}

void StereoVo::FindFundamentalMat(const std::vector<CvPoint2> &points1,
                                  const std::vector<CvPoint2> &points2,
                                  std::vector<uchar> &status) {
  if (points1.empty()) {
    ROS_WARN("FindFundamentalMat() called with no points");
    return;
  }
  cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, 0.5, 0.9999, status);
}

void StereoVo::TrackTemporal(const cv::Mat &image1, const cv::Mat &image2,
                             std::vector<Feature> &features) {
  std::vector<uchar> status;
  std::vector<CvPoint2> corners2;
  std::vector<CvPoint2> corners1;
  ExtractCorners(features, corners1);

  // Track and remove mismatches
  OpticalFlow(image1, image2, corners1, corners2, status);
  PruneByStatus(status, features);
  PruneByStatus(status, corners1);
  PruneByStatus(status, corners2);
  ROS_ASSERT_MSG(corners1.size() == corners2.size(),
                 "Corners size mismatch in OpticalFlow()");
  status.clear();

  // Find fundamental matrix to reject outliers in tracking
  FindFundamentalMat(corners1, corners2, status);
  PruneByStatus(status, features);
  PruneByStatus(status, corners2);
  ROS_ASSERT_MSG(features.size() == corners2.size(),
                 "Corners size mismatch in FindFundamentalMat()");

  // Modify tracked features position
  std::transform(features.begin(), features.end(), corners2.begin(),
                 features.begin(), [](Feature &f, const CvPoint2 &c) {
    f.px = c;
    return f;
  });
}

/*
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

//  const KrPose &diff = pose().difference(prev_key_frame->pose());
//  if (diff.p().norm() > config_.kf_dist_thresh) {
//    ROS_INFO("Distance: %f", diff.p().norm());
//    //  over distance threshold, add keyframe
//    return true;
//  }

  //  const kr::vec3<scalar_t> &angles = kr::getRPY(diff.bRw());
  //  if (std::abs(angles[2] * 180 / M_PI) > config_.kf_yaw_thresh) {
  //    ROS_INFO("Angle: %f", angles[2]);
  //    //  over yaw angle threshold, add keyframe
  //    return true;
  //  }

  return false;
}
*/

/*
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
      //p3d.AddObservation(config_,model_,ptr,f.p_pixel());
    }
  }

  //  add the new left features to the collection of trackables
  temporal_tracker_.AddFeatures(left_features);
  key_frames_.push_back(ptr);
}
*/

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

/*
void StereoVo::Iterate(const CvStereoImage& stereo_image) {
  if (!prev_left_image_.empty()) {
    //  track from previous frame
    std::vector<Feature> unused;
    temporal_tracker_.set_window_size(cv::Size(config_.klt_win_size,
                                              config_.klt_win_size));
    temporal_tracker_.set_ransac_thresh(config_.klt_ransac_thresh);
    temporal_tracker_.set_max_levels(config_.klt_max_level);
    temporal_tracker_.Track(prev_left_image_, stereo_image.first, unused);
  }
  */

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
/*
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
*/

/*
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

  std::vector<int> inliers;
  const size_t min_inliers = std::ceil(N * config_.pnp_ransac_inliers);
  cv::solvePnPRansac(world_points, pixel_points,
                     model_.left().fullIntrinsicMatrix(), std::vector<double>(),
                     rvec, tvec, false, 300, config_.pnp_ransac_error,
                     min_inliers, inliers);

  //  throw away outliers
  std::set<Id> erased_ids = temporal_tracker_.RetainInliers(inliers);
  for (const Id& id : erased_ids) {
    points_.erase(id);
  }

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
*/

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

bool StereoVo::TriangulatePoint(const CvPoint2& lp,
                                const CvPoint2& rp,
                                kr::vec3<scalar_t> &p3d,
                                kr::mat3<scalar_t> &sigma) {
  //  convert to normalized coordinates
  const double lfx = model_.left().fx(), lfy = model_.left().fy();
  const double lcx = model_.left().cx(), lcy = model_.left().cy();
  const double rfx = model_.right().fx(), rfy = model_.right().fy();
  const double rcx = model_.right().cx(), rcy = model_.right().cy();
  
  const kr::vec2d v_left((lp.x-lcx)/lfx, (lp.y-lcy)/lfy);
  const kr::vec2d v_right((rp.x-rcx)/rfx, (rp.y-rcy)/rfy);
  
  //  triangulate in the frame of the left camera
  const kr::Posed left_pose;
  const kr::Posed right_pose(kr::quatd(1,0,0,0),
                             kr::vec3d(model_.baseline(),0,0));
  kr::vec3d position;
  double ratio;
  const double depth = kr::triangulate(left_pose,v_left,
                                       right_pose,v_right,position,ratio);
  /// @todo: make the ratio an option
  if (depth < 0 || ratio > 1e4) {
    return false;
  }
  p3d = position.cast<scalar_t>();
  
  //  calculate covariance on depth and 3D position
  const double depth_sigma = 
      kr::triangulationDepthSigma(depth, v_left, right_pose.p(), lfx);
  const auto P = 
      kr::triangulationCovariance(v_left, 1.0 / lfx, depth, depth_sigma);
  sigma = P.cast<scalar_t>();
  return true;
}

void StereoVo::PublishStereoFeatures(const KeyFrame &keyframe,
                                     const ros::Time &time) const {
  StereoFeaturesStamped stereo_features;
  stereo_features.header.frame_id = "stereo_vo";
  stereo_features.header.stamp = time;
  stereo_features.image_id = keyframe.id();
  const std::vector<Feature> &features = keyframe.features();
  auto fit = std::find_if(features.cbegin(), features.cend(),
                          [](const Feature &f) { return f.fresh; });
  for (auto fit_beg = features.cbegin(); fit_beg != fit; ++fit_beg) {
    FeatureMsg l_feature;
    const Feature &f = *fit_beg;
    l_feature.id = f.id;
    l_feature.point.x = f.px.x;
    l_feature.point.y = f.px.y;
    l_feature.fresh = f.fresh;
    stereo_features.left.push_back(l_feature);
  }

  for (const CvPoint2 &c : keyframe.r_corners()) {
    const Feature &f = *fit++;
    FeatureMsg l_feature;
    FeatureMsg r_feature;
    l_feature.id = r_feature.id = f.id;
    l_feature.fresh = r_feature.fresh = f.fresh;
    l_feature.point.x = f.px.x;
    l_feature.point.y = f.px.y;
    r_feature.point.x = c.x;
    r_feature.point.y = c.y;
    stereo_features.left.push_back(l_feature);
    stereo_features.right.push_back(r_feature);
  }

  pub_features_.publish(stereo_features);
}

}  // namespace stereo_vo
}  // namespace galt
