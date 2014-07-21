/*
 * SpectrumCalibrator.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 17/7/2014
 *      Author: gareth
 */

#include "spectrumcalibrator.h"
#include <cv_bridge/cv_bridge.h>
#include "utilities.hpp"
#include <kr_math/pose.hpp>
#include <kr_math/SO3.hpp>

SpectrumCalibrator::SpectrumCalibrator(QObject *parent,
                                       const galt::SpectrometerPose &specPose,
                                       const galt::FilterProfile &filterProfile)
    : QObject(parent), specPose_(specPose), filterProfile_(filterProfile) {
  ros::NodeHandle nh("~");

  //  TODO: Add some error checking mechanism to this
  //  for now we just hope it works
  imgTransport_ = std::make_shared<image_transport::ImageTransport>(nh);
  subImage_.subscribe(*imgTransport_, "image", kROSQueueSize);
  subCamInfo_.subscribe(nh, "camera_info", kROSQueueSize);
  subPose_.subscribe(nh, "pose_tags", kROSQueueSize);
  subSpectrum_.subscribe(nh, "spectrum", kROSQueueSize);

  sync_ = std::make_shared<message_filters::Synchronizer<TimeSyncPolicy>>(
      TimeSyncPolicy(kROSQueueSize), subImage_, subCamInfo_, subPose_,
      subSpectrum_);
  sync_->registerCallback(
      boost::bind(&SpectrumCalibrator::syncCallback, this, _1, _2, _3, _4));

  ROS_INFO("Subscribing to ~image, ~camera_info, ~pose_tags and ~spectrum");
}

const cv::Mat &SpectrumCalibrator::lastImage() const { return image_; }

const galt::Spectrum &SpectrumCalibrator::lastSpectrum() const {
  return spectrum_;
}

const galt::FilterProfile &SpectrumCalibrator::getFilterProfile() const {
  return filterProfile_;
}

const galt::Spectrum &SpectrumCalibrator::getPredictedSpectrum() const {
  return predictedSpectrum_;
}

void SpectrumCalibrator::syncCallback(
    const sensor_msgs::ImageConstPtr &img,
    const sensor_msgs::CameraInfoConstPtr &info,
    const geometry_msgs::PoseStampedConstPtr &poseStamped,
    const ocean_optics::SpectrumConstPtr &spec) {
  //  lazy: regardless of input format, convert to rgb8 then mono
  cv_bridge::CvImageConstPtr bridgedImagePtr =
      cv_bridge::toCvCopy(img, "mono8");
  if (!bridgedImagePtr) {
    ROS_ERROR("Failed to convert image to mono8 with cv_bridge");
    return;
  }
  cv::Mat monoImage = bridgedImagePtr->image;

  //  convert to intensity
  cv::Mat rgbImage;
  cv::cvtColor(monoImage, rgbImage, CV_GRAY2RGB);
  image_ = rgbImage;
  cameraInfo_ = *info;

  //  spectrometer measurement
  spectrum_ = galt::Spectrum(spec->wavelengths, spec->spectrum);
  
  //  apply the filter profile to get prediction
  galt::Spectrum specPred = spectrum_;
  specPred.multiply(filterProfile_.getSpectrum());
  predictedSpectrum_ = specPred;
  
  //  project the spectrometer view into the image
  kr::Pose<double> pose(poseStamped->pose);
  
  //  AprilTags produce pose of camera in tag frame, so reverse it
  pose = pose.inverse();
  
  kr::vec3d o = pose.p;
  kr::vec3d n = pose.q.matrix() * kr::vec3d(0,0,1);
  
  //  TODO: refactor this...
  const double d = specPose_.distanceToPlane(o,n);
  kr::vec3d p_cam = specPose_.getPosition() + specPose_.getDirection()*d;
  
  p_cam[0] /= p_cam[2];
  p_cam[1] /= p_cam[2]; //  project
  
  //  distort
  cv::Point2d dist = distortPoint(info->D, cv::Point2d(p_cam[0],p_cam[1]));
  
  //  camera intrinsics
  double fx = info->K[0];
  double cx = info->K[2];
  double fy = info->K[4];
  double cy = info->K[5];
    
  dist.x = dist.x*fx + cx;
  dist.y = dist.y*fy + cy;
  double radius = std::tan(specPose_.getFov()/2) * d * 0.5 * (fx+fy) / p_cam[2];
    
  if (radius > 0.0) {
    //  draw circle on image
    cv::circle(image_, dist, radius, cv::Scalar(0,255,0), 3);
  } else {
    ROS_WARN("Negative radius! Bad calibration?");
  }
  
  if (true) { //  logic here...
    addObservation(pose, predictedSpectrum_);
  }
  
  emit receivedMessage();
}

void SpectrumCalibrator::addObservation(const kr::Pose<double>& pose, 
                                        const galt::Spectrum& spectrum) {
  
  //  TODO: some code duplication here, correct/clean this up after gallo
  //  pose = orientation of plane in camera frame
  //  spectrum = spectrum passing through filter
  
  kr::vec3d o = pose.p;
  kr::vec3d n = pose.q.matrix() * kr::vec3d(0,0,1);
  
  const kr::vec3d v = specPose_.getDirection();
  
  //  find a pair of perpendicular axes
  kr::vec3d u;
  u[0] = 1;
  u[1] = 1;
  u[2] = (-v[0]*u[0] - v[1]*u[1]) / v[2];
  u /= u.norm();
  //kr::vec3d w = v.cross(u);
  
  const double fx = cameraInfo_.K[0];
  const double cx = cameraInfo_.K[2];
  const double fy = cameraInfo_.K[4];
  const double cy = cameraInfo_.K[5];
  
  //  generate vectors 'a' around the conic section
  const double cos = std::cos(specPose_.getFov() / 2);
  const double sin = std::sin(specPose_.getFov() / 2);
  for (int i=0; i < 10; i++) {  
    kr::vec3d a = v*cos + u*sin;
    //  rotate
    a = kr::rodriguesExp<double>(v * i / 10.0 * 2 * M_PI) * a;
    
    //  project to plane
    galt::SpectrometerPose sp(specPose_.getPosition(), a, 0, 0);
    const double d = sp.distanceToPlane(o,n);
    
    a = sp.getPosition() + a*d;
    
    //  project back to image
    a[0] /= a[2];
    a[1] /= a[2];
    
    cv::Point2d dist = distortPoint(cameraInfo_.D, cv::Point2d(a[0],a[1]));
    
    dist.x = fx*dist.x + cx;
    dist.y = fy*dist.y + cy;
    
    cv::circle(image_, dist, 5, cv::Scalar(255,0,0), 2);
  }
}

