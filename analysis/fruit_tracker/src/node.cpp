#include <fruit_tracker/node.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace galt {
namespace fruit_tracker {

Node::Node(const ros::NodeHandle& pnh) : pnh_(pnh), it_(pnh_) {
  
  //  subscribe to image & camera info
  sub_image_.subscribe(it_, "image", kROSQueueSize);
  sub_cam_info_.subscribe(pnh_, "camera_info", kROSQueueSize);
  
  sync_image_ = std::make_shared<Synchronizer>(TimeSyncPolicy(kROSQueueSize),
                                               sub_image_,sub_cam_info_);
  sync_image_->registerCallback(
        boost::bind(&Node::imageCallback, this, _1, _2));
}

Node::~Node() {}

void Node::imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
                         const sensor_msgs::CameraInfoConstPtr& info_msg) {
  
  cv_bridge::CvImageConstPtr bridgedImg = 
      cv_bridge::toCvShare(image_msg, "bgr8");
  if (!bridgedImg || bridgedImg->image.empty()) {
    ROS_ERROR("Failed to convert image to bgr8");
    return;
  }
  
  //  input image
  cv::Mat input = bridgedImg->image;
  
  //  convert to HSV
  cv::Mat hsv_input;
  cv::cvtColor(input,hsv_input,cv::COLOR_BGR2HSV);
  //  extract separate channels from HSV
  std::vector<cv::Mat> channels(3);
  cv::split(hsv_input,channels);
  //cv::imshow("h",channels[0]);
  //cv::imshow("s",channels[1]);
  //cv::imshow("v",channels[2]);
  
  //  threshold the h-channel (blacks only)
  cv::threshold(channels[0],channels[0],0.1*255,255,cv::THRESH_BINARY_INV);
  //  threshold the v-channel (whites only)
  cv::threshold(channels[2],channels[2],0.95*255,255,cv::THRESH_BINARY_INV);
  
  //  make the detection mask by adding them together...
  cv::Mat detect(hsv_input.rows,hsv_input.cols,CV_8UC1);
  for (int i=0; i < detect.rows; i++) {
    for (int j=0; j < detect.cols; j++) {
      detect.at<uchar>(i,j) = (channels[0].at<uchar>(i,j) && 
          channels[2].at<uchar>(i,j)) * 255;
    }
  }
  //  box blur it
  cv::blur(detect,detect,cv::Size(3,3));
  cv::Mat contour_image;
  detect.copyTo(contour_image);
   
  //  do connected components
  std::vector <std::vector<cv::Point>> contours;
  cv::findContours(contour_image,contours,cv::RETR_CCOMP,cv::CHAIN_APPROX_NONE);
  //  throw away using area threshold
  for (const std::vector<cv::Point>& contour : contours) {
    const double area = std::abs(cv::contourArea(contour));
    
    if (area > 100) {
      //  find the moments
      cv::Moments moment = cv::moments(contour);
      double inv_m00 = 1 / moment.m00;
      cv::Point2d center = cv::Point2d(moment.m10 * inv_m00, 
                                       moment.m01 * inv_m00);
      const double radius = std::sqrt(area / M_PI)*1.2;
      
      //  draw onto input image
      cv::circle(input,center,static_cast<int>(radius),cv::Scalar(0,0,255),2);
    }
  }
  
  cv::imshow("bgr", input);
  //cv::imshow("hsv", hsv_input);
  cv::imshow("detect", detect);
  cv::waitKey(1);
}

} //  fruit_tracker
} //  galt
