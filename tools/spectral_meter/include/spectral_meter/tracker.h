#ifndef SPECTRAL_METER_TRACKER_H_
#define SPECTRAL_METER_TRACKER_H_

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <dynamic_reconfigure/server.h>

#include <spectral_meter/OpticalFlowDynConfig.h>

namespace galt {
namespace spectral_meter {

class Tracker {
 public:
  using Config = ::spectral_meter::OpticalFlowDynConfig;
  using Ptr = boost::shared_ptr<Tracker>;
  explicit Tracker(const ros::NodeHandle& pnh);

  bool isInit() const { return !prev_image_.empty(); }
  void step(const cv::Mat& image);
  cv::Point2f calcOffset() const;
  void configCallback(Config& config, int level);

  const std::vector<cv::Point2f> prev_points() const { return prev_points_; }
  const std::vector<cv::Point2f> curr_points() const { return curr_points_; }

 private:
  bool needMoreFeatures() {
    return curr_points_.size() < config_.num_features * config_.min_features_k;
  }

  ros::NodeHandle pnh_;
  dynamic_reconfigure::Server<Config> cfg_server_;
  cv::Mat prev_image_;
  std::vector<cv::Point2f> prev_points_, curr_points_;
  cv::Ptr<cv::FeatureDetector> detector_;
  Config config_;
};

void printCvAlgorithmParams(cv::Algorithm* algo);

template <typename T, typename U>
void pruneByStatus(const std::vector<U>& status, std::vector<T>& objects) {
  ROS_ASSERT_MSG(status.size() == objects.size(),
                 "status and object size mismatch");
  ROS_ASSERT_MSG(!status.empty(), "nothing to prune");
  auto it_obj = objects.begin();
  for (const auto& s : status) {
    if (s) {
      it_obj++;
    } else {
      it_obj = objects.erase(it_obj);
    }
  }
}

void drawKeypoints1(const cv::Mat& image,
                    const std::vector<cv::KeyPoint>& keypoints,
                    cv::Mat& image_out);

void drawTrackes(const cv::Mat& image1,
                 const std::vector<cv::Point2f>& corners1,
                 const cv::Mat& image2,
                 const std::vector<cv::Point2f>& corners2,
                 const std::vector<uchar>& matches, cv::Mat& image_track);

}  // namespace spectral_meter
}  // namespace galt

#endif  // SPECTRAL_METER_TRACKER_H_
