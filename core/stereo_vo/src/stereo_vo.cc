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

void StereoVo::Initialize(const cv::Mat &l_image, const cv::Mat &r_image,
                          const StereoCameraModel &model) {
  model_ = model;
  // Detect features for the first time
  detector_.AddFeatures(l_image, features_);
  // Save current images to previous images
  l_image_prev_ = l_image;
  r_image_prev_ = r_image;
  // Create a window for display
  cv::namedWindow("display",
                  CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  init_ = true;
  std::cout << "StereoVo initialized, baseline: " << model_.baseline()
            << std::endl;
}

void StereoVo::Iterate(const cv::Mat &l_image, const cv::Mat &r_image) {
  // Update features for this iteration, assign pixel_next -> pixel_left
  UpdateFeatures(features_);
  // Track features from prev left into prev right, assign pixel_right
  TrackFeatures(l_image_prev_, r_image_prev_, features_,
                &Feature::set_p_pixel_right, config_.win_size,
                config_.max_level);
  // Track features from prev left into next left, assign pixel_next
  TrackFeatures(l_image_prev_, l_image, features_, &Feature::set_p_pixel_next,
                config_.win_size, config_.max_level);
  // Triangulate points with features in stereo images
  TriangulateFeatures();
  // Estimate pose using PnP
  EstimatePose();
  // Add new features for tracking later
  detector_.AddFeatures(l_image, features_);
  // Check if a new keyframe is necessary and add
  AddKeyFrame();
  // Visualization (optional)
  Display(l_image_prev_, l_image, r_image_prev_, r_image, features_);
  // Save previous left image for tracking later, and right image for display
  l_image_prev_ = l_image;
  r_image_prev_ = r_image;
}

void StereoVo::EstimatePose() {
  if (!features_.empty()) {

    std::vector<CvPoint2> imagePoints;
    std::vector<CvPoint3> worldPoints;
    std::vector<uchar> inliers;

    imagePoints.reserve(features_.size());
    worldPoints.reserve(features_.size());

    for (const Feature &feat : features_) {
      imagePoints.push_back(feat.p_pixel_left());
      worldPoints.push_back(feat.p_world());
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

    auto pose = kr::Pose<scalar_t>::fromVectors(r, t);

    //  left-multiply by the keyframe pose to get world pose
    current_pose_ = pose;
  } else {
    ROS_WARN("EstimatePose() called but no features available");
  }
}

bool StereoVo::AddKeyFrame() {
  bool should_add_key_frame = false;
  if (key_frames_.empty()) {
    should_add_key_frame = true;
  } else {
    const KeyFrame &last_key_frame = key_frames_.back();
    //  check distance metric to see if new keyframe is required
    const double distance = (last_key_frame.pose().p - current_pose_.p).norm();
    if (distance > config_.keyframe_dist_thresh) {
      should_add_key_frame = true;
    }
  }

  if (should_add_key_frame) {
    //  TODO: add a keyframe here eventually
  }
  return should_add_key_frame;
}



/*void KeyFrame::Update(const cv::Mat &l_image, const cv::Mat &r_image,
                      const StereoVoConfig &config,
                      const StereoCameraModel &model,
                      const kr::Pose<scalar_t> &pose, bool init) {
  // Collect relevant options
  int num_features = config.num_features;

  std::vector<CvPoint2> l_features, r_features;
  std::vector<uchar> status;
  // Find features
  cv::goodFeaturesToTrack(l_image, l_features, num_features, 0.01, 10);
  if (l_features.empty()) {
    ROS_WARN("No new features found");
    return;  //  no new features
  }

  TrackFeatures(l_image, r_image, l_features, r_features, status, config);
  PruneByStatus(status, l_features);
  PruneByStatus(status, r_features);

  //  initialize new features
  const scalar_t lfx = model.left().fx(), lfy = model.left().fy();
  const scalar_t lcx = model.left().cx(), lcy = model.left().cy();
  const scalar_t rfx = model.right().fx(), rfy = model.right().fy();
  const scalar_t rcx = model.right().cx(), rcy = model.right().cy();

  features_.clear();
  features_.reserve(l_features.size());
  for (size_t i = 0; i < l_features.size(); i++) {
    Feature feat;
    feat.left = l_features[i];
    feat.next = feat.left;
    feat.right = r_features[i];

    //  undo K matrix
    feat.left_coord.x = (feat.left.x - lcx) / lfx;
    feat.left_coord.y = (feat.left.y - lcy) / lfy;
    feat.right_coord.x = (feat.right.x - rcx) / rfx;
    feat.right_coord.y = (feat.right.y - rcy) / rfy;

    features_.push_back(feat);
  }

  const scalar_t meanDepth = Triangulate(model);
  l_image_ = l_image;
  r_image_ = r_image;
  if (!init) {
    pose_ = pose;
  } else {
    pose_ = kr::Pose<scalar_t>(kr::quat<scalar_t>(0, 1, 0, 0),
                               kr::vec3<scalar_t>(0, 0, meanDepth));
  }
}*/

void StereoVo::TriangulateFeatures() {
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
}

void TrackFeatures(const cv::Mat &image1, const cv::Mat &image2,
                   Features &features,
                   std::function<void(Feature *, const CvPoint2 &)> update_func,
                   const int win_size, const int max_level) {
  static cv::TermCriteria term_criteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 25, 0.01);
  std::vector<uchar> status;
  CvCorners2 corners1;
  // Put pixel left into features1
  for (const auto &feature : features) {
    corners1.push_back(feature.p_pixel_left());
  }
  CvCorners2 corners2;
  // Do optical flow
  cv::calcOpticalFlowPyrLK(image1, image2, corners1, corners2, status,
                           cv::noArray(), cv::Size(win_size, win_size),
                           max_level, term_criteria);
  // TODO: improve this part later
  PruneByStatus(status, features);
  PruneByStatus(status, corners1);
  PruneByStatus(status, corners2);
  status.clear();
  // Do find fundamental matrix
  cv::findFundamentalMat(corners1, corners2, cv::FM_RANSAC, 1.5, 0.99, status);
  PruneByStatus(status, features);
  PruneByStatus(status, corners1);
  PruneByStatus(status, corners2);
  // Update features
  ROS_ASSERT_MSG((features.size() == corners1.size()) &&
                     (features.size() == corners2.size()),
                 "Dimension mismatch");
  auto it_cnr2 = corners2.cbegin();
  auto it_cnr2_e = corners2.cend();
  auto it_feat = features.begin();
  for(; it_cnr2 != it_cnr2_e; it_cnr2++, it_feat++) {
    update_func(&(*it_feat), *it_cnr2);
  }
}

}  // namespace stereo_vo

}  // namespace galt
