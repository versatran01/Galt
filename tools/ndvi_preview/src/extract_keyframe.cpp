#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <sstream>

class KeyframeExtractor {
public:
  KeyframeExtractor(const ros::NodeHandle &pnh, const std::string &camera_ns)
      : pnh_(pnh), num_keyframes_(0) {
    pnh_.param("scale", scale_, 0.5);
    pnh_.param("num_corners", num_corners_, 700);
    pnh_.param("min_corners_ratio", min_corners_ratio_, 0.6);
    pnh_.param("min_distance", min_distance_, 15);
    pnh_.param("win_size", win_size_, 21);
    pnh_.param("max_level", max_level_, 4);
    pnh_.param<std::string>("out_dir", out_dir_, "/tmp");
    camera_ = camera_ns;
    if (camera_.front() == '/') {
      camera_.erase(0, 1);
    }
  }

  void ProcessImage(const sensor_msgs::Image::ConstPtr &image_msg) {
    cv::Mat image_scaled;
    cv::resize(cv_bridge::toCvShare(image_msg)->image, image_scaled,
               cv::Size(0, 0), scale_, scale_);
    Track(image_scaled);
  }

private:
  template <typename T, typename U>
  void pruneByStatus(const std::vector<U> &status, std::vector<T> &objects) {
    ROS_ASSERT_MSG(status.size() == objects.size(),
                   "status and object size mismatch");
    ROS_ASSERT_MSG(!status.empty(), "nothing to prune");
    auto it_obj = objects.begin();
    for (const auto &s : status) {
      if (s) {
        it_obj++;
      } else {
        it_obj = objects.erase(it_obj);
      }
    }
  }

  void Track(const cv::Mat &image) {
    cv::Mat gray;
    cv::Mat disp;

    // Ensure grayscale iamge
    if (image.type() != CV_8UC1) {
      cv::cvtColor(image, gray, CV_BGR2GRAY);
    } else {
      gray = image;
    }
    cv::cvtColor(gray, disp, CV_GRAY2BGR);

    // Track
    if (!prev_image_.empty()) {
      std::vector<uchar> status;
      std::vector<cv::Point2f> curr_points;
      const cv::TermCriteria term_criteria(
          cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 10, 0.005);
      cv::calcOpticalFlowPyrLK(prev_image_, gray, tracked_points_, curr_points,
                               status, cv::noArray(),
                               cv::Size(win_size_, win_size_), max_level_,
                               term_criteria);
      pruneByStatus(status, tracked_points_);
      pruneByStatus(status, curr_points);
      status.clear();
      cv::findFundamentalMat(tracked_points_, curr_points, status,
                             cv::FM_RANSAC, 1, 0.99);
      pruneByStatus(status, tracked_points_);
      pruneByStatus(status, curr_points);
      ROS_ASSERT_MSG(curr_points.size() == tracked_points_.size(),
                     "Track mismatch");
      DrawPoints(disp, curr_points, cv::Scalar(0, 255, 0));
      DrawTracks(disp, tracked_points_, curr_points, cv::Scalar(255, 0, 0));
      tracked_points_ = curr_points;
    }

    // Extract
    if (tracked_points_.size() < num_corners_ * min_corners_ratio_) {
      // Create a mask based on tracked features
      cv::Mat mask = cv::Mat::ones(gray.rows, gray.cols, CV_8UC1);
      for (const auto &p : tracked_points_) {
        cv::circle(mask, p, min_distance_, cv::Scalar(0), -1);
      }
      // Detect new corners and append to tracked points
      std::vector<cv::Point2f> corners;
      cv::goodFeaturesToTrack(gray, corners, num_corners_, 0.01, min_distance_,
                              mask);
      // Draw newly detected points with red
      DrawPoints(disp, corners, cv::Scalar(0, 0, 255));
      // Add to tracked corners
      tracked_points_.insert(tracked_points_.end(), corners.begin(),
                             corners.end());
      num_keyframes_ += 1;
      // Save to disk
      std::ostringstream ss;
      ss << std::setw(4) << std::setfill('0') << num_keyframes_;
      const std::string filename = camera_ + "_" + ss.str();
      cv::imwrite(out_dir_ + "/" + filename + ".png", image);
    }
    cv::putText(disp, std::to_string(num_keyframes_), {30, 30},
                cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 255), 2, CV_AA);
    prev_image_ = gray;

    cv::imshow("track", disp);
    cv::waitKey(1);
  }

  void DrawTracks(cv::Mat &image, const std::vector<cv::Point2f> &points1,
                  const std::vector<cv::Point2f> &points2,
                  const cv::Scalar &color) {
    for (size_t i = 0; i < points1.size(); ++i) {
      cv::line(image, points1[i], points2[i], color, 2);
    }
  }

  void DrawPoints(cv::Mat &image, const std::vector<cv::Point2f> &points,
                  const cv::Scalar &color) {
    for (const cv::Point2f &p : points) {
      cv::circle(image, p, 2, color, -1);
    }
  }

  ros::NodeHandle pnh_;
  double scale_;
  cv::Mat prev_image_;
  std::vector<cv::Point2f> tracked_points_;
  int num_corners_;
  int min_distance_;
  double min_corners_ratio_;
  int win_size_;
  int max_level_;
  int num_keyframes_;
  std::string out_dir_;
  std::string camera_;
};

//  _                _
// | |__   __ _  ___| | __
// | '_ \ / _` |/ __| |/ /
// | | | | (_| | (__|   <
// |_| |_|\__,_|\___|_|\_\
//

int main(int argc, char **argv) {
  ros::init(argc, argv, "extract_and_rectify");
  ros::NodeHandle pnh("~");

  std::string in_bag_path;
  if (!pnh.getParam("in_bag", in_bag_path)) {
    ROS_ERROR("No input bag file specified.");
    return -1;
  }

  int queue_size;
  pnh.param("queue_size", queue_size, 20);

  std::string camera_ns;
  pnh.param<std::string>("camera", camera_ns, "/spectral_670");
  std::string image;
  pnh.param<std::string>("image", image, "image_rect");
  const std::string image_topic = camera_ns + "/" + image;

  // Open rosbag
  rosbag::Bag in_bag(in_bag_path);
  rosbag::View view(in_bag, rosbag::TopicQuery(image_topic));

  ROS_INFO("Opened bag: %s", in_bag.getFileName().c_str());

  KeyframeExtractor extractor(pnh, camera_ns);

  for (auto it = view.begin(), it_end = view.end(); it != it_end; ++it) {
    const rosbag::MessageInstance &m = *it;

    // Image topic
    if (m.getTopic() == image_topic || ("/" + m.getTopic() == image_topic)) {
      sensor_msgs::ImageConstPtr image_msg =
          m.instantiate<sensor_msgs::Image>();
      if (image_msg)
        extractor.ProcessImage(image_msg);
    }
  }
}
