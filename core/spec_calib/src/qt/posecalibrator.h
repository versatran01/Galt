/*
 * posecalibrator.h
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 11/7/2014
 *      Author: gareth
 */

#ifndef POSECALIBRATOR_H
#define POSECALIBRATOR_H

#include <QObject>
#include <QThread>
#include <memory>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>

#include <circle_tracker/Circles.h>
#include <spectral/SpectrometerPose.hpp>

#include <monocular_pose_estimator/PixelArray.h>

#include <kr_math/pose.hpp>

class PoseCalibrator : public QObject
{
  Q_OBJECT
public:
  struct Circle {
    cv::Point2d uv;   //  pixels
    cv::Point2d xy;   //  normalized
    double radiusPix; //  pixels
    double radiusCam; //  normalized
    
    //  compare with radius
    bool operator < (const Circle& b) const { return radiusPix < b.radiusPix; }
  };
  
  explicit PoseCalibrator(QObject *parent=0);
  virtual ~PoseCalibrator();
  
  const cv::Mat& lastImage() const;
  
  size_t observationCount() const;
  
  bool canCalibrate() const;
  
  void calibrate();
    
  bool hasCalibration() const;
  
  const galt::SpectrometerPose& getSpectrometerPose() const;
  
  Circle projectWithPose(const kr::Pose<double>& pose);
  
signals:
  void receivedMessage();
  
public slots:
  
private:
  std::shared_ptr<image_transport::ImageTransport> imgTransport_;
  
  //  ROS subscribers
  static constexpr uint32_t kROSQueueSize = 10;
  
  image_transport::SubscriberFilter subImage_;
  message_filters::Subscriber <sensor_msgs::CameraInfo> subCamInfo_;
  message_filters::Subscriber <circle_tracker::Circles> subCircles_;
  message_filters::Subscriber <geometry_msgs::PoseStamped> subPose_;
  message_filters::Subscriber <monocular_pose_estimator::PixelArray> subPixels_;
  
  //  time sync policy
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
      sensor_msgs::CameraInfo, circle_tracker::Circles,
      geometry_msgs::PoseStamped, monocular_pose_estimator::PixelArray> TimeSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<TimeSyncPolicy>> sync_;
  
  //  ROS callback
  void syncCallback(const sensor_msgs::ImageConstPtr&,
                    const sensor_msgs::CameraInfoConstPtr&,
                    const circle_tracker::CirclesConstPtr&,
                    const geometry_msgs::PoseStampedConstPtr&poseStamped,
                    const monocular_pose_estimator::PixelArrayConstPtr&);
  
  double depthThreshold_;
  double pixelThreshold_;
  sensor_msgs::CameraInfoConstPtr camInfo_;
  
  struct Observation {
    Circle circle;
    cv::Point3d p;  //  camera coordinates
    double depth;
    
    //  compare using depth
    bool operator < (const Observation& b) const { return depth < b.depth; }
    
    //  distance along line
    double calcDistance(const kr::vec3d& l0, const kr::vec3d& ln) const {
      const kr::vec3d del = kr::vec3d(p.x,p.y,p.z) - l0;
      return del[0]*ln[0] + del[1]*ln[1] + del[2]*ln[2];
    }
  };
  
  //  add new observation to current set
  bool addObservation(const kr::Pose<double> &pose, const Circle& circle);
  
  cv::Mat image_;
  
  //  measurements collected
  std::vector <Observation> observations_;
  
  //  calibration
  galt::SpectrometerPose pose_;
  bool hasCalibration_;
};

#endif // POSECALIBRATOR_H
