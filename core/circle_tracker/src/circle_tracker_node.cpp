/*
 * circle_tracker_node.cpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
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

#include "circle_tracker/Circle.h"
#include "circle_tracker/Circles.h"

#include "elliptical_fit.hpp"

using namespace std;

ros::NodeHandlePtr nh;  //  private node handle
ros::Publisher circPub;

//  global parameters
bool displayGui;

double lowCannyThresh = 0.0;
double highCannyThresh;
int pointCountThresh;
double areaThreshold;
double circThresh;

struct Circle {
  cv::Point2f center;
  double radius;
  double score;
};

//  based on optical_flow_circle.cpp
void camera_callback(const sensor_msgs::Image::ConstPtr &img,
                     const sensor_msgs::CameraInfo::ConstPtr &camInfo) {
  
  cv_bridge::CvImageConstPtr bridgedImagePtr = cv_bridge::toCvShare(img,"mono8");
  if (!bridgedImagePtr) {
    ROS_ERROR("Failed to convert image with cv_bridge");
    ros::shutdown();
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
  
  cv::Mat distCoeffs(camInfo->D.size(), 1, CV_64F, const_cast<double*>(camInfo->D.data()));
  
  //  blur to reduce noise
  cv::Mat blurredImage;
  blurredImage = bridgedImage.image;
  //cv::GaussianBlur(bridgedImage.image,blurredImage,cv::Size(2,2),0);
  
  //  find edges
  cv::Mat edgeImage;
  //cv::threshold(blurredImage,edgeImage,15.0,255.0,CV_THRESH_BINARY);
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
        //  apply area threshold
        const double A = std::abs( cv::contourArea(undist) );
        if (A > areaThreshold) {
          
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
    cv::imshow("edge_image", edgeImage);
    cv::imshow("contour_image", contourImage);
    cv::waitKey(1);
  }
}
             
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "circle_tracker_node");
  nh = ros::NodeHandlePtr( new ros::NodeHandle("~") );

  //  load parameters
  nh->param(string("high_canny_threshold"), highCannyThresh, 120.0);
  nh->param(string("point_count_threshold"), pointCountThresh, 6);
  nh->param(string("area_threshold"), areaThreshold, 0.0005);
  nh->param(string("circle_threshold"), circThresh, 0.70);
  nh->param(string("display_gui"), displayGui, false);
  
  circPub = nh->advertise<circle_tracker::Circles>("circles", 1);
  
  //  subscribe to image feed
  image_transport::ImageTransport it(*nh);
  image_transport::CameraSubscriber camSub;
  
  camSub = it.subscribeCamera("image_raw",1,&camera_callback);
  
  ros::spin();
  return 0;
}
