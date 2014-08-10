#include "stereo_vo/stereo_vo.h"
#include "stereo_vo/feature.h"
#include "stereo_vo/key_frame.h"
#include "stereo_vo/utils.h"

#include <sstream>
#include <iostream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <kr_math/base_types.hpp>
#include <kr_math/feature.hpp>
#include <kr_math/pose.hpp>

#include <Eigen/Geometry>

namespace galt {

namespace stereo_vo {

void StereoVo::Initialize(const CvStereoImage &stereo_image,
                          const StereoCameraModel &model) {
  model_ = model;
  // Add the first stereo image as first keyframe
  // At this moment, we use current_pose as input, but inherently will use
  // estimated depth to reinitialize current_pose.
  // Later will replace this with estimates of other sensors.
  std::vector<Feature> tracked_features;
  AddKeyFrame(current_pose(), stereo_image, tracked_features);

  // Save stereo image and tracked features for next iteration
  stereo_image_prev_ = stereo_image;
  features_ = tracked_features;

  // Create a window for display
  // TODO: replace this with published topic later
  cv::namedWindow("display",
                  CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  init_ = true;
  ROS_INFO_STREAM("StereVo initialized, baseline: " << model_.baseline());
}

void StereoVo::Iterate(const CvStereoImage &stereo_image) {

  std::vector<Feature> tracked_features;
  std::set<Feature::Id> removables;

  // TrackTemporal(l_image_prev_, l_image, removables);

  // remove features here...

  auto relative_pose = EstimatePose();
  current_pose_ = current_pose_.compose(relative_pose);

  // This will check if it's necessary to add new keyframes and add it if true
  AddKeyFrame(current_pose(), stereo_image, tracked_features);

  // Windowed optimization here
  // Check if key_frame_.size() == N

  // Visualization (optional)
  Display(stereo_image, tracked_features, key_frames_.back());

  // Save stereo image and tracked features for next iteration
  stereo_image_prev_ = stereo_image;
  features_ = tracked_features;
}

Pose StereoVo::EstimatePose() {
  if (!features_.empty()) {

    std::vector<CvPoint2> imagePoints;
    std::vector<CvPoint3> worldPoints;
    std::vector<uchar> inliers;

    imagePoints.reserve(features_.size());
    worldPoints.reserve(features_.size());

    for (const Feature &feat : features_) {
      imagePoints.push_back(feat.p_pixel_left());
      worldPoints.push_back(feat.p_cam_left());
    }

    cv::Mat rvec = cv::Mat(3, 1, CV_64FC1);
    cv::Mat tvec = cv::Mat(3, 1, CV_64FC1);
    const size_t minInliers =
        std::ceil(worldPoints.size() * config_.pnp_ransac_inliers);
    cv::solvePnPRansac(
        worldPoints, imagePoints, model_.left().fullIntrinsicMatrix(),
        std::vector<double>(), rvec, tvec, false, 100, config_.pnp_ransac_error,
        minInliers, inliers, cv::ITERATIVE);

    //  convert rotation to quaternion
    kr::vec3<scalar_t> r(rvec.at<double>(0, 0), rvec.at<double>(1, 0),
                         rvec.at<double>(2, 0));
    kr::vec3<scalar_t> t(tvec.at<double>(0, 0), tvec.at<double>(1, 0),
                         tvec.at<double>(2, 0));

    auto pose = Pose::fromVectors(r, t);
    return pose;
  }

  throw std::runtime_error("EstimatePose called with empty features");
  return Pose();
}

void StereoVo::AddKeyFrame(const Pose &pose, const CvStereoImage &stereo_image,
                           std::vector<Feature> features) {
  bool should_add_key_frame = false;
  if (key_frames_.empty()) {
    should_add_key_frame = true;
  } else {
    const auto &last_key_frame = key_frames_.back();
    //  check distance metric to see if new keyframe is required
    const double distance = (last_key_frame.pose().p - current_pose_.p).norm();
    if (distance > config_.kf_dist) {
      should_add_key_frame = true;
    }
    //  do another check on feature distribution then set should_add_key_frame
  }

  if (should_add_key_frame) {
    //  TODO: add a keyframe here eventually
  }
}

/*void StereoVo::TriangulateFeatures(bool set_ready) {
  kr::vec2<scalar_t> lPt, rPt;
  Pose poseLeft;   //  identity
  Pose poseRight;  //  shifted right along x
  poseRight.p[0] = model_.baseline();

  //  initialize new features
  const scalar_t lfx = model_.left().fx(), lfy = model_.left().fy();
  const scalar_t lcx = model_.left().cx(), lcy = model_.left().cy();
  const scalar_t rfx = model_.right().fx(), rfy = model_.right().fy();
  const scalar_t rcx = model_.right().cx(), rcy = model_.right().cy();

  for (auto I = features_.begin(); I != features_.end();) {
    Feature &feature = *I;
    bool failed = false;

    if (!feature.triangulated()) {
      //  new feature requires triangulation
      lPt[0] = feature.p_pixel_left().x;
      lPt[1] = feature.p_pixel_left().y;
      rPt[0] = feature.p_pixel_right().x;
      rPt[1] = feature.p_pixel_right().y;

      lPt[0] = (lPt[0] - lcx) / lfx;
      lPt[1] = (lPt[1] - lcy) / lfy;
      rPt[0] = (rPt[0] - rcx) / rfx;
      rPt[1] = (rPt[1] - rcy) / rfy;

      kr::vec3<scalar_t> p3D;
      scalar_t ratio;

      //  triangulate 3d position
      kr::triangulate(poseLeft, lPt, poseRight, rPt, p3D, ratio);

      if (ratio > 1e5) {
        //  bad, reject this feature
        failed = true;
      } else {
        //  valid, refine the point
        std::vector<Pose> poses({poseLeft, poseRight});
        std::vector<kr::vec2<scalar_t>> obvs({lPt, rPt});

        if (kr::refinePoint(poses, obvs, p3D)) {
          //  convert to world coordinates
          p3D = current_pose_.q.conjugate() * p3D + current_pose_.p;
          feature.set_p_world(CvPoint3(p3D[0], p3D[1], p3D[2]));
          feature.set_triangulated(true);
          //feature.set_ready(set_ready);  //  set ready if indicated by iterate
        } else {
          //  failed to converge
          failed = true;
        }
      }
    }

    if (failed) {
      I = features_.erase(I);
    } else {
      I++;
    }
  }
}*/

// void TrackFeatures(const cv::Mat &image1, const cv::Mat &image2,

//                   std::function<void(Feature *, const CvPoint2 &)>
// update_func,
//                   const int win_size, const int max_level) {
////  static cv::TermCriteria term_criteria(
////      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 25, 0.01);
////  std::vector<uchar> status;
////  CvCorners2 corners1;
////  corners1.reserve(features.size());
////  // Put pixel left into features1
////  for (const auto &feature : features) {
////    corners1.push_back(feature.p_pixel_left());
////  }
////  CvCorners2 corners2;
////  // Do optical flow
////  cv::calcOpticalFlowPyrLK(image1, image2, corners1, corners2, status,
////                           cv::noArray(), cv::Size(win_size, win_size),
////                           max_level, term_criteria);
////  // TODO: improve this part later
////  PruneByStatus(status, features);
////  PruneByStatus(status, corners1);
////  PruneByStatus(status, corners2);
////  status.clear();
////  // Do find fundamental matrix
////  cv::findFundamentalMat(corners1, corners2, cv::FM_RANSAC, 1.5, 0.99,
/// status);
////  PruneByStatus(status, features);
////  PruneByStatus(status, corners1);
////  PruneByStatus(status, corners2);
////  // Update features
////  ROS_ASSERT_MSG((features.size() == corners1.size()) &&
////                     (features.size() == corners2.size()),
////                 "Dimension mismatch");
////  auto it_cnr2 = corners2.cbegin();
////  auto it_cnr2_e = corners2.cend();
////  auto it_feat = features.begin();
////  for (; it_cnr2 != it_cnr2_e; it_cnr2++, it_feat++) {
////    update_func(&(*it_feat), *it_cnr2);
////  }
//}

}  // namespace stereo_vo

}  // namespace galt
