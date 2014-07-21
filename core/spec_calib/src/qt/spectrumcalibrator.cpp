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

//  TODO: This code is a mess, clean it up after gallo
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
    addObservation(kr::vec2d(dist.x,dist.y), radius, predictedSpectrum_, monoImage);
  }
  
  emit receivedMessage();
}

void SpectrumCalibrator::addObservation(const kr::vec2d &point, double radius, 
                                        const galt::Spectrum& spectrum, 
                                        const cv::Mat& monoImage) {
  
  //  sample from the image in the circle
  
  int min_x = std::max( std::floor(point[0] - radius), 0.0 );
  int min_y = std::max( std::floor(point[1] - radius), 0.0 );
  int max_x = std::min( static_cast<int>(std::ceil(point[0] + radius)), image_.cols );
  int max_y = std::min( static_cast<int>(std::ceil(point[1] + radius)), image_.rows );
 
  std::vector<double> pixels;
  
  for (int y=min_y; y < max_y; y++) {
    for (int x=min_x; x < max_x; x++) {
      double r2 = (y-min_y)*(y-min_y) + (x-min_x)*(x-min_x);
      if (r2 < radius*radius*0.9) {
        // point is in circle, sample
        const double I = monoImage.at<uchar>(y,x) * (1 / 255.0);
        pixels.push_back(static_cast<double>(I));
      }
    }
  }
  
  //  calculate total
  double total=0.0;
  double total2=0.0;  //  squared
  for (const double& I : pixels) {
    total += I;
    total2 += I*I;
  }
  const double mean = total / pixels.size();
  const double mean_sqr = total2 / pixels.size();
  const double var = mean_sqr - mean*mean;
  
  //  integrate over the spectrum
  
}

