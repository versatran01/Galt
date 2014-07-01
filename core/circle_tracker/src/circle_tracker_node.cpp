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
#include "elliptical_fit.hpp"

ros::NodeHandlePtr nh;  //  private node handle
ros::Publisher circPub;

//  global parameters
bool displayGui = true;

double lowCannyThresh = 0.0;
double highCannyThresh = 120.0;
int pointCountThresh = 6;
double areaThreshold = 0.001;
double circThresh = 0.85;

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
  cv::GaussianBlur(bridgedImage.image,blurredImage,cv::Size(7,7),0);
  
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
                  
      //cv::drawContours(contourImage,contours,idx,cv::Scalar(rand()&255,rand()&255,rand()&255));
      
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
              cv::drawContours(contourImage,contours,idx,cv::Scalar(0,255,0));
            }
          }
        }
      }
    }
    
    idx++;
  }
  
  //  take best circle
  if (!matches.empty())
  {
    std::sort(matches.begin(), matches.end(), [](const Circle& a, const Circle& b) -> bool {
      return std::abs(a.score - 1.0) < std::abs(b.score - 1.0);
    });
    
    Circle result = matches.front();

    //  publish
    circle_tracker::Circle message;
    message.header = img->header; //  repeat timestamp and seq #
    message.centerX = result.center.x;
    message.centerY = result.center.y;
    message.radius = result.radius;
    message.score = result.score;
    
    circPub.publish(message);
  }
  
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

  circPub = nh->advertise<circle_tracker::Circle>("circle", 1);
  
  //  subscribe to image feed
  image_transport::ImageTransport it(*nh);
  image_transport::CameraSubscriber camSub;
  
  camSub = it.subscribeCamera("image_raw",1,&camera_callback);
  
  ros::spin();
  return 0;
}
