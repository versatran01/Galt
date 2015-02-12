#include "spectral_meter/tracker.h"

#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace galt {
namespace spectral_meter {

Tracker::Tracker(const ros::NodeHandle& pnh) : pnh_(pnh), cfg_server_(pnh) {
  cfg_server_.setCallback(boost::bind(&Tracker::configCallback, this, _1, _2));
  detector_ = cv::FeatureDetector::create("GFTT");
  detector_->set("minDistance", config_.min_distance);
  detector_->set("nfeatures", config_.num_features);
  printCvAlgorithmParams(detector_);
}

void Tracker::step(const cv::Mat& image) {
  cv::Mat display;
  cv::Mat image_gray;

  if (image.type() != CV_8UC1) {
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
  } else {
    image_gray = image;
  }

  if (isInit()) {
    prev_points_ = curr_points_;
    curr_points_.clear();
    ROS_INFO("Tracking features: %d", (int)prev_points_.size());
    ROS_ASSERT_MSG(!prev_points_.empty(), "No tracked features");
    // optical flow
    std::vector<uchar> status;
    const cv::TermCriteria term_criteria(
        cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 10, 0.005);
    cv::calcOpticalFlowPyrLK(prev_image_, image_gray, prev_points_,
                             curr_points_, status, cv::noArray(),
                             cv::Size(config_.win_size, config_.win_size),
                             config_.max_level, term_criteria);
    pruneByStatus(status, prev_points_);
    pruneByStatus(status, curr_points_);
    status.clear();
    cv::findFundamentalMat(prev_points_, curr_points_, status, cv::FM_RANSAC, 1,
                           0.99);
    pruneByStatus(status, prev_points_);
    pruneByStatus(status, curr_points_);
    drawTrackes(prev_image_, prev_points_, image_gray, curr_points_,
                std::vector<uchar>(), display);
  }

  if (needMoreFeatures()) {
    // Create a mask based on these features
    cv::Mat mask = cv::Mat::ones(image.rows, image.cols, CV_8UC1);
    for (const cv::Point2f& p : prev_points_) {
      cv::circle(mask, p, config_.min_distance, cv::Scalar(0), -1);
    }
    // Detect new keypoints
    std::vector<cv::KeyPoint> keypoints;
    detector_->detect(image_gray, keypoints, mask);
    if (display.empty()) {
      display = image;
    }
    drawKeypoints1(display, keypoints, display);
    ROS_INFO("Number of new points: %d", (int)keypoints.size());
    // Add newly detected keypoints to prev_features
    for (const cv::KeyPoint& keypoint : keypoints) {
      curr_points_.push_back(keypoint.pt);
    }
    ROS_ASSERT_MSG(prev_points_.size() <= curr_points_.size(), "oops");
  }
  prev_image_ = image_gray;

  cv::imshow("track", display);
  cv::waitKey(1);
}

cv::Point2f Tracker::calcOffset() const {
  float x{0.0}, y{0.0};
  for (size_t i = 0; i < prev_points_.size(); ++i) {
    x += curr_points_[i].x - prev_points_[i].x;
    y += curr_points_[i].y - prev_points_[i].y;
  }
  x /= prev_points_.size();
  y /= prev_points_.size();
  return {x, y};
}

void Tracker::configCallback(Config& config, int level) {
  if (level < 0) {
    ROS_INFO("%s: Initializaing reconfigure server",
             pnh_.getNamespace().c_str());
  }
  config_ = config;
}

void printCvAlgorithmParams(cv::Algorithm* algo) {
  std::vector<std::string> params;
  algo->getParams(params);
  std::cout << algo->name() << std::endl;
  for (const std::string& param_name : params) {
    const auto type = algo->paramType(param_name);
    const auto help_text = algo->paramHelp(param_name);
    std::string type_text;
    std::cout << " parameter - value: ";
    switch (type) {
      case cv::Param::BOOLEAN:
        type_text = "bool";
        std::cout << std::boolalpha << algo->getBool(param_name);
        break;
      case cv::Param::INT:
        type_text = "int";
        std::cout << algo->getInt(param_name);
        break;
      case cv::Param::REAL:
        type_text = "real (double)";
        std::cout << algo->getDouble(param_name);
        break;
      case cv::Param::STRING:
        type_text = "string";
        std::cout << algo->getString(param_name);
        break;
      case cv::Param::MAT:
        type_text = "Mat";
        break;
      case cv::Param::ALGORITHM:
        type_text = "Algorithm";
        break;
      case cv::Param::MAT_VECTOR:
        type_text = "Mat vector";
        break;
    }
    std::cout << ", name: " << param_name << ", type: " << type_text
              << std::endl;
  }
}

void drawKeypoints1(const cv::Mat& image,
                    const std::vector<cv::KeyPoint>& keypoints,
                    cv::Mat& image_out) {
  cv::drawKeypoints(image, keypoints, image_out, CV_RGB(0, 255, 0),
                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
}

void drawTrackes(const cv::Mat& image1,
                 const std::vector<cv::Point2f>& corners1,
                 const cv::Mat& image2,
                 const std::vector<cv::Point2f>& corners2,
                 const std::vector<uchar>& matches, cv::Mat& image_track) {
  double alpha = 0.2;
  double beta = (1.0 - alpha);
  cv::addWeighted(image1, alpha, image2, beta, 0.0, image_track);
  // Process matches
  std::vector<uchar> matches_;
  if (matches.empty()) {
    matches_ = std::vector<uchar>(corners2.size(), 1);
  } else {
    matches_ = matches;
  }
  for (size_t i = 0; i < matches_.size(); ++i) {
    cv::circle(image_track, corners2[i], 3, CV_RGB(0, 0, 255), -1, CV_AA);
    cv::line(image_track, corners1[i], corners2[i], CV_RGB(255, 0, 0), 2);
  }
}

}  // namespace spectral_meter
}  // namespace galt
