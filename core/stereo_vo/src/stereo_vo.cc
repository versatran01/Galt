#include "stereo_vo/stereo_vo.h"

#include <sstream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <kr_math/base_types.hpp>
#include <kr_math/feature.hpp>
#include <kr_math/pose.hpp>

namespace galt {

namespace stereo_vo {

struct CvColor {
  cv::Scalar blue = cv::Scalar(255, 0, 0);
  cv::Scalar green = cv::Scalar(0, 255, 0);
  cv::Scalar red = cv::Scalar(0, 0, 255);
} cv_color;

StereoVo::StereoVo() {}

void StereoVo::Initialize(const cv::Mat &l_image, const cv::Mat &r_image,
                          const StereoCameraModel &model) {
  model_ = model;
  key_frame_.Update(l_image, r_image, config_, model);

  std::cout << "StereoVo initialized, baseline: " << model_.baseline()
            << std::endl;

  // Create a window for display
  cv::namedWindow("two_frame",
                  CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  init_ = true;
  return;
}

void StereoVo::Iterate(const cv::Mat &l_image, const cv::Mat &r_image) {
  // Track features across frames
  std::vector<cv::Point2f> new_features, l_features;
  std::vector<uchar> status;

  for (const Feature& feat : key_frame_.features_) {
    l_features.push_back(feat.left);
  }
  
  if (!l_features.empty()) {
    cv::TermCriteria term_criteria(
        cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 25, 0.001);
    cv::calcOpticalFlowPyrLK(key_frame_.l_image_, l_image, l_features,
                             new_features, status, cv::noArray(),
                             cv::Size(11, 11), 3, term_criteria);
  }
  
  /*for (auto p : new_features) {
    
  }*/
  
  PruneByStatus(status, key_frame_.features_);
  
  // Display images
  Display(l_image, r_image, new_features);
  // Save the new images
  if (new_features.size() < 90) {
    key_frame_.Update(l_image, r_image, config_, model_);
  }
}

void StereoVo::Display(const cv::Mat &l_image, const cv::Mat &r_image,
                       const std::vector<CvPoint2> &new_features) {
  int n_rows = l_image.rows;
  int n_cols = l_image.cols;
  static cv::Mat two_frame(2 * n_rows, 2 * n_cols, CV_8UC1);

  // Copy 2 frames 4 images on to one image
  key_frame_.l_image_.copyTo(two_frame(cv::Rect(0, 0, n_cols, n_rows)));
  key_frame_.r_image_.copyTo(two_frame(cv::Rect(n_cols, 0, n_cols, n_rows)));
  l_image.copyTo(two_frame(cv::Rect(0, n_rows, n_cols, n_rows)));
  r_image.copyTo(two_frame(cv::Rect(n_cols, n_rows, n_cols, n_rows)));

  // Convert to color
  cv::Mat two_frame_color;
  cv::cvtColor(two_frame, two_frame_color, CV_GRAY2BGR);

  // Draw triangulated features on key frame
  for (const auto &feature : key_frame_.features_) {
    auto r_p = feature.right + CvPoint2(n_cols, 0);
    cv::circle(two_frame_color, feature.left, 3, cv_color.blue, 2);
    cv::circle(two_frame_color, r_p, 3, cv_color.green, 2);
    cv::line(two_frame_color, feature.left, r_p, cv_color.green, 1);
  }

  // Draw tracked features on new frame
  auto it_new = new_features.cbegin(), e_new = new_features.cend();
  auto it_feat = key_frame_.features_.cbegin();
  for(; it_new != e_new; ++it_new, ++it_feat) {
    auto n_p = *it_new + CvPoint2(0, n_rows);
    cv::circle(two_frame_color, n_p, 3, cv_color.red, 2);
    cv::line(two_frame_color, it_feat->left, n_p, cv_color.red, 1);
  }

  // Add text annotation
  double offset_x = 10.0, offset_y = 30.0;
  auto font = cv::FONT_HERSHEY_SIMPLEX;
  double scale = 1.0, thickness = 2.0;
  // Which frame?
  cv::putText(two_frame_color, "key frame", CvPoint2(offset_x, offset_y),
              font, scale, cv_color.blue, thickness);
  cv::putText(two_frame_color, "current frame",
              CvPoint2(n_cols + offset_x, n_rows + offset_y), font, scale,
              cv_color.blue, thickness);
  // How many matching features?
  std::ostringstream ss;
  ss << key_frame_.features_.size();
  cv::putText(two_frame_color, ss.str(),
              CvPoint2(offset_x, n_rows - offset_y / 2), font, scale,
              cv_color.blue, thickness);

  // Display image
  cv::imshow("two_frame", two_frame_color);
  cv::waitKey(1);
}

void KeyFrame::Update(const cv::Mat &l_image, const cv::Mat &r_image,
                      const StereoVoDynConfig &config, const StereoCameraModel &model) {
  // Collect relevant options
  int max_iter = 25;
  double epsilon = 0.01;
  int win_size = config.win_size;
  int max_level = config.max_level;
  int num_features = config.num_features;

  std::vector<cv::Point2f> l_features, r_features;
  std::vector<uchar> status;
  // Find features
  cv::goodFeaturesToTrack(l_image, l_features, num_features, 0.01, 10);
  if (l_features.empty()) {
    return; //  no new features
  }
  // Optical flow
  cv::TermCriteria term_criteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, max_iter, epsilon);
  cv::calcOpticalFlowPyrLK(l_image, r_image, l_features, r_features, status,
                           cv::noArray(), cv::Size(win_size, win_size),
                           max_level, term_criteria);
  
  PruneByStatus(status,l_features);
  PruneByStatus(status,r_features);
  
  // Fundamental matrix (reject outliers via RANSAC)
  status.clear();
  cv::findFundamentalMat(l_features, r_features, cv::FM_RANSAC, 1, 0.99,
                         status);
  PruneByStatus(status, l_features);
  PruneByStatus(status, r_features);
  
  //  initialize new features
  const scalar_t lfx = model.left().fx(), lfy = model.left().fy();
  const scalar_t lcx = model.left().cx(), lcy = model.left().cy();
  const scalar_t rfx = model.right().fx(), rfy = model.right().fy();
  const scalar_t rcx = model.right().cx(), rcy = model.right().cy();
  
  features_.clear();
  features_.reserve(l_features.size());
  for (size_t i=0; i < l_features.size(); i++) {
    Feature feat;
    feat.left = l_features[i];
    feat.right = r_features[i];
    
    feat.left_coord.x = (feat.left.x - lcx) / lfx; //  undo K matrix
    feat.left_coord.y = (feat.left.y - lcy) / lfy;
    feat.right_coord.x = (feat.right.x - rcx) / rfx;
    feat.right_coord.y = (feat.right.y - rcy) / rfy;
    
    features_.push_back(feat);
  }

  Triangulate(model);
  l_image_ = l_image;
  r_image_ = r_image;
}

void KeyFrame::Triangulate(const StereoCameraModel& model) {
      
  kr::vec2<scalar_t> lPt, rPt;
  kr::Pose<scalar_t> poseLeft;  //  identity
  kr::Pose<scalar_t> poseRight;
  poseRight.p[0] = model.baseline();
  
  for (auto itr = features_.begin(); itr != features_.end();) {
    
    lPt[0] = itr->left.x;
    lPt[1] = itr->left.y;
    rPt[0] = itr->right.x;
    rPt[1] = itr->right.y;
    
    kr::vec3<scalar_t> p3D;
    scalar_t ratio;
    
    kr::triangulate(poseLeft,lPt,poseRight,rPt,p3D,ratio);
    
    bool failed = false;
    if (ratio > 1e4) {
      //  bad, reject this feature
      failed = true;
    } else {
      //  valid, refine the point
      std::vector<kr::Pose<scalar_t>> poses({poseLeft,poseRight});
      std::vector<kr::vec2<scalar_t>> obvs({lPt,rPt});
      
      if (kr::refinePoint(poses,obvs,p3D)) {
        itr->point = cv::Point3f(p3D[0],p3D[1],p3D[2]);
      } else {
        //  failed to converge
        failed = true;
      }
    }
    
    if (failed) {
      //  erase feature
      itr = features_.erase(itr);
    } else {
      itr->triangulated = true;      
      itr++;
    }
  }
}

std::vector<cv::Point2f> ExtractByStatus(
    const std::vector<cv::Point2f> &features,
    const std::vector<uchar> &status) {
  std::vector<cv::Point2f> new_features;
  for (unsigned i = 0; i != status.size(); ++i) {
    if (status[i]) {
      new_features.push_back(features[i]);
    }
  }
  return new_features;
}

}  // namespace stereo_vo

}  // namespace galt
