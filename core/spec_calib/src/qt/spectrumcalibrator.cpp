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

SpectrumCalibrator::SpectrumCalibrator(QObject *parent, const galt::SpectrometerPose &specPose, const galt::Spectrum &filterProfile) :
  QObject(parent), specPose_(specPose), filterProfile_(filterProfile)
{
  ros::NodeHandle nh("~");
  
  //  TODO: Add some error checking mechanism to this
  //  for now we just hope it works
  imgTransport_ = std::make_shared<image_transport::ImageTransport>(nh);
  subImage_.subscribe(*imgTransport_, "image", kROSQueueSize);
  subCamInfo_.subscribe(nh, "camera_info", kROSQueueSize);
  subPose_.subscribe(nh, "pose_tags", kROSQueueSize);
  subSpectrum_.subscribe(nh, "spectrum", kROSQueueSize);
  
  sync_ = std::make_shared<message_filters::Synchronizer<TimeSyncPolicy>>(TimeSyncPolicy(kROSQueueSize),
                                                                          subImage_,
                                                                          subCamInfo_,
                                                                          subPose_,
                                                                          subSpectrum_);
  sync_->registerCallback(boost::bind(&SpectrumCalibrator::syncCallback, this, _1, _2, _3, _4));
  
  ROS_INFO("Subscribing to ~image, ~camera_info, ~pose_tags and ~spectrum");
}

const cv::Mat& SpectrumCalibrator::lastImage() const {
  return image_; 
}

const galt::Spectrum& SpectrumCalibrator::lastSpectrum() const {
  return spectrum_;
}

const galt::Spectrum& SpectrumCalibrator::getFilterProfile() const {
  return filterProfile_;
}

void SpectrumCalibrator::syncCallback(const sensor_msgs::ImageConstPtr& img,
                  const sensor_msgs::CameraInfoConstPtr &info, 
                  const geometry_msgs::PoseStampedConstPtr &poseStamped,
                  const ocean_optics::SpectrumConstPtr& spec)
{  
  //  lazy: regardless of input format, convert to rgb8 then mono
  cv_bridge::CvImageConstPtr bridgedImagePtr = cv_bridge::toCvCopy(img, "mono8");
  if (!bridgedImagePtr) {
    ROS_ERROR("Failed to convert image to mono8 with cv_bridge");
    return;
  }  
  cv::Mat monoImage = bridgedImagePtr->image;
    
  //  convert to intensity
  cv::Mat rgbImage;
  cv::cvtColor(monoImage, rgbImage, CV_GRAY2RGB);
  image_ = rgbImage;  
  
  //  spectrometer measurement
  spectrum_ = galt::Spectrum(spec->wavelengths, spec->spectrum);
  
  
  
  emit receivedMessage();
}
