#include "flir_gige/calibrator.h"
#include <sstream>
#include <cmath>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace flir_gige {

using std::cout;
using std::endl;

Calibrator::Calibrator(const ros::NodeHandle &nh) : nh_{nh}, it_{nh} {
  image_sub_ = it_.subscribe("image_raw", 1, &Calibrator::ImageCallback, this);
  server_.setCallback(
      boost::bind(&Calibrator::ReconfigureCallback, this, _1, _2));
  // Get parameters for calibration
  nh_.param<double>("depth", depth_, 1.0);
  nh_.param<double>("dist", dist_, 0.17);
  nh_.param<int>("roi_width", roi_width_, 64);
  nh_.param<int>("roi_height", roi_height_, 32);
  nh_.param<int>("thresh", thresh_, 200);
  nh_.param<double>("sigma", sigma_, 1.0);
  nh_.param<double>("min_area", min_area_, 100);
  nh_.param<double>("max_area", max_area_, 225);
  nh_.param<double>("max_width_height_distortion", max_width_height_distortion_,
                    0.5);
  nh_.param<double>("max_circular_distortion", max_circular_distortion_, 0.5);
  // Create a window
  cv::namedWindow("calib");
}

void Calibrator::ImageCallback(const sensor_msgs::ImageConstPtr &image) {
  static cv::Rect roi;
  static std::vector<double> pixel_dists;
  cv::Mat image_gray, image_color;

  GetImage(image, image_gray, image_color);
  if (!init_roi_) {
    roi = CalculateRoi(image_gray.cols, image_gray.rows);
  }
  DrawRoi(image_color, roi);
  // Threshold the image
  cv::Mat image_bw;
  cv::threshold(image_gray(roi), image_bw, thresh_, 255, cv::THRESH_TOZERO);
  cv::imshow("thresh", image_bw);

  // Blur the image
  cv::Mat image_gauss;
  cv::Size ksize;  // Gaussian kernerl size.
  cv::GaussianBlur(image_bw, image_gauss, ksize, sigma_);
  cv::imshow("gauss", image_gauss);

  // Find all contours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(image_gauss.clone(), contours, CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_NONE);
  cv::Mat image_contour = cv::Mat::zeros(image_gauss.size(), CV_8UC3);
  for (unsigned i = 0; i < contours.size(); i++) {
    cv::drawContours(image_contour, contours, i, cv::Scalar(0, 0, 255), 2);
  }
  cv::imshow("contour", image_contour);

  std::vector<Point2> points;
  // Identify all the blobs in the image
  cv::Mat image_bound;
  cv::cvtColor(image_gauss, image_bound, CV_GRAY2RGB);
  for (const auto &ctr : contours) {
    double area = cv::contourArea(ctr);     // Blob area
    cv::Rect rect = cv::boundingRect(ctr);  // Bounding box
    // Calculate moments
    cv::Moments mu = cv::moments(ctr);
    Point2 mc = Point2(mu.m10 / mu.m00, mu.m01 / mu.m00);
    // Test with size
    if (area >= min_area_ && area <= max_area_) {
      double width_height_distortion = std::abs(
          1 - std::min(static_cast<double>(rect.width) / rect.height,
                       static_cast<double>(rect.height) / rect.width));
      double circular_distortion_height =
          std::abs(1 - (area / (CV_PI * std::pow(rect.height / 2, 2))));
      double circular_distortion_width =
          std::abs(1 - (area / (CV_PI * std::pow(rect.width / 2, 2))));
      if (width_height_distortion < max_width_height_distortion_ &&
          circular_distortion_height < max_circular_distortion_ &&
          circular_distortion_width < max_circular_distortion_) {
        points.push_back(mc);
        cv::circle(image_bound, mc, 1, cv::Scalar(0, 255, 0), 2);
        cv::rectangle(image_bound, rect, cv::Scalar(255, 0, 0), 2);
        std::ostringstream ss;
        ss << area;
        cv::putText(image_bound, ss.str(), mc, cv::FONT_HERSHEY_PLAIN, 1,
                    cv::Scalar(255, 255, 0), 2);
      }
    }
  }
  // If found two points
  unsigned n_points = 20;
  if (points.size() == 2 && (pixel_dists.size() < n_points)) {
    double pixel_dist = CalculateDistance(points[0], points[1]);
    pixel_dists.push_back(pixel_dist);
    ROS_INFO_STREAM(points[0].x << " " << points[1].x);
  } else if (pixel_dists.size() == n_points) {
    double d_avg =
        std::accumulate(pixel_dists.cbegin(), pixel_dists.cend(), 0.0) /
        pixel_dists.size();
    double f = d_avg * depth_ / dist_;
    ROS_INFO("pixel distance: %f, depth: %f, distance: %f, focal length: %f",
             d_avg, depth_, dist_, f);
    ros::shutdown();
  }
  cv::imshow("bound", image_bound);

  cv::imshow("calib", image_color);
  cv::waitKey(5);
}

double Calibrator::CalculateDistance(const Point2 &p1, const Point2 &p2) {
  return {std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2))};
}

void Calibrator::ReconfigureCallback(CalibConfig &config, int level) {
  if (level < 0) {
    return;
  }
  roi_width_ = config.roi_width;
  roi_height_ = config.roi_height;
  thresh_ = config.thresh;
  sigma_ = config.sigma;
  min_area_ = config.min_area;
  max_area_ = config.max_area;
  init_roi_ = false;
}

void Calibrator::GetImage(const sensor_msgs::ImageConstPtr &image,
                          cv::Mat &image_gray, cv::Mat &image_color) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image, image->encoding);
  if (cv_ptr->image.type() == CV_8UC1) {
    image_gray = cv_ptr->image;
    cv::cvtColor(image_gray, image_color, CV_GRAY2RGB);
  } else if (cv_ptr->image.type() == CV_8UC3) {
    image_color = cv_ptr->image;
    cv::cvtColor(image_color, image_gray, CV_BGR2GRAY);
  }
}

cv::Rect Calibrator::CalculateRoi(int width, int height) {
  int x = (width - roi_width_) / 2;
  int y = (height - roi_height_) / 2;
  return {x, y, roi_width_, roi_height_};
}

void Calibrator::DrawRoi(cv::Mat &image_color, const cv::Rect &roi) {
  cv::rectangle(image_color, roi, cv::Scalar(255, 0, 0), 2);
}

}  // namespace flir_gige
