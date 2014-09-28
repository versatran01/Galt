/*
 * circle_tracker_node.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 28/6/2014
 *      Author: gareth
 */

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_msgs/Header.h>

#include <image_transport/image_transport.h>

#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <dynamic_reconfigure/server.h>
#include <circle_tracker/CircleTrackerConfig.h>

#include "circle_tracker/Circle.h"
#include "circle_tracker/Circles.h"

#include "elliptical_fit.hpp"

using namespace std;

ros::NodeHandlePtr nh;  //  private node handle
ros::Publisher circPub;

std::shared_ptr<dynamic_reconfigure::Server<circle_tracker::CircleTrackerConfig>> drServer;

//  global parameters
bool displayGui;

const double lowCannyThresh = 0.0;
double highCannyThresh;
int pointCountThresh;
double areaThreshold;
double circThresh;
double blurSigma;

struct Circle {
  cv::Point2f center;
  double radius;
  double score;
};

//  user changed parameters
void dynamicParamsCallback(circle_tracker::CircleTrackerConfig& config,
                           uint32_t level) {
  highCannyThresh = config.high_canny_threshold;
  pointCountThresh = config.point_count_threshold;
  areaThreshold = config.area_threshold;
  circThresh = config.circle_threshold;
  blurSigma = config.blur_sigma;
}

//  based on optical_flow_circle.cpp
void camera_callback(const sensor_msgs::Image::ConstPtr &img,
                     const sensor_msgs::CameraInfo::ConstPtr &camInfo) {

  cv_bridge::CvImageConstPtr bridgedImagePtr;
  try {
    bridgedImagePtr = cv_bridge::toCvShare(img,"mono8");
    
    if (!bridgedImagePtr) {
      ROS_ERROR("cv_bridge failed to convert image to mono8");
      return;
    }
  }
  catch (std::exception& e) {
    ROS_WARN("Warning: toCvShare threw an exception: %s", e.what());
    return;
  }

  const cv_bridge::CvImage& bridgedImage = *bridgedImagePtr;

  //  extract camera info
  cv::Mat F(3,3,CV_64F);
  memset(F.ptr(),0,sizeof(double)*9);

  F.at<double>(0,0) = camInfo->K[0];
  F.at<double>(0,2) = camInfo->K[2];
  F.at<double>(1,1) = camInfo->K[4];
  F.at<double>(1,2) = camInfo->K[5];
  F.at<double>(2,2) = 1;

  double focal = (camInfo->K[0] + camInfo->K[4]) / 2.0;

  cv::Mat distCoeffs(camInfo->D.size(), 1, CV_64F, const_cast<double*>(camInfo->D.data()));

  //  blur to reduce noise
  cv::Mat blurredImage;
  if (blurSigma < 0.5) {
    blurredImage = bridgedImage.image;
  } else {
    cv::GaussianBlur(bridgedImage.image,blurredImage,cv::Size(0,0),blurSigma);
  }

  //  find edges
  cv::Mat edgeImage;
  cv::Canny(blurredImage,edgeImage,lowCannyThresh,highCannyThresh);

  //  find contours
  std::vector<std::vector<cv::Point>> contours;

  cv::Mat contourImage;
  if (displayGui) {
    contourImage = cv::Mat(edgeImage.size(),CV_8UC3, cv::Scalar(0,0,0));  //  draw debug contours here
  }

  cv::findContours(edgeImage,contours,CV_RETR_CCOMP,CV_CHAIN_APPROX_NONE);

  std::vector <Circle> matches;
  int idx=0;

  for (const std::vector<cv::Point>& contour : contours)
  {
    if (contour.size() > static_cast<size_t>(pointCountThresh))
    {
      //  undistort the contour
      std::vector<cv::Point2f> dist;
      std::vector<cv::Point2f> undist;
      std::vector<cv::Point2d> undist_d;
      for (const cv::Point& p : contour) {
        dist.push_back(p);  //  convert to cv::Point2f
      }
      cv::undistortPoints(dist,undist,F,distCoeffs);

      for (const cv::Point2d& p : undist) {
        undist_d.push_back(cv::Point2d(p.x,p.y));
      }

      if (undist.size() > static_cast<size_t>(pointCountThresh))
      {
        //  apply area threshold (in pixels)
        const double A = std::abs( cv::contourArea(undist) );
        if (A*focal*focal > areaThreshold) {

          const double C = cv::arcLength(undist,true);  //  circumference
          const double score = 4*M_PI*A / (C*C);

          //  determine circular-ity (1 for a real circle)
          if (score > circThresh) {

            Circle circ;

            //  calculate moments
            cv::Moments moment = cv::moments(undist);
            double inv_m00 = 1 / moment.m00;
            circ.center = cv::Point2d(moment.m10 * inv_m00, moment.m01 * inv_m00);
            circ.radius = std::sqrt(A / M_PI);
            circ.score = score;
            matches.push_back(circ);

            if (displayGui) {
              cv::drawContours(contourImage,contours,idx,cv::Scalar(0,0,255));
            }
          }
        }
      }
    }

    idx++;
  }

  circle_tracker::Circles circlesMsg;
  circlesMsg.header = img->header;
  if (!matches.empty())
  {
    //  sort circles from best to worst
    std::sort(matches.begin(), matches.end(), [](const Circle& a, const Circle& b) -> bool {
      return std::abs(a.score - 1.0) < std::abs(b.score - 1.0);
    });

    for (const Circle& match : matches) {
      circle_tracker::Circle message;

      message.centerX = match.center.x;
      message.centerY = match.center.y;
      message.radius = match.radius;
      message.score = match.score;

      circlesMsg.circles.push_back(message);
    }
  }
  circPub.publish(circlesMsg);  //  publish even when empty

  if (displayGui) {
    cv::imshow("input_image", bridgedImage.image);
    //cv::imshow("edge_image", edgeImage);
    cv::imshow("contour_image", contourImage);
    cv::waitKey(1);
  }
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "circle_tracker_node");
  nh = ros::NodeHandlePtr( new ros::NodeHandle("~") );

  //  load parameters
  nh->param("high_canny_threshold", highCannyThresh, 120.0);
  nh->param("point_count_threshold", pointCountThresh, 6);
  nh->param("area_threshold", areaThreshold, 50.0);
  nh->param("circle_threshold", circThresh, 0.70);
  nh->param("blur_sigma", blurSigma, 0.0);
  nh->param("display_gui", displayGui, false);

  circPub = nh->advertise<circle_tracker::Circles>("circles", 1);

  //  subscribe to image feed
  image_transport::ImageTransport it(*nh);
  image_transport::CameraSubscriber camSub;

  camSub = it.subscribeCamera("image_raw",1,&camera_callback);

  //  reconfigure server
  drServer = std::make_shared<dynamic_reconfigure::Server<circle_tracker::CircleTrackerConfig>>();
  
  dynamic_reconfigure::Server<circle_tracker::CircleTrackerConfig>::CallbackType cb;
  cb = boost::bind(dynamicParamsCallback, _1, _2);
  drServer->setCallback(cb);

  ros::spin();
  return 0;
}
