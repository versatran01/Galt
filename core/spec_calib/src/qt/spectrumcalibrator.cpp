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

SpectrumCalibrator::SpectrumCalibrator(QObject *parent, const ros::NodeHandlePtr &nhp) :
  QObject(parent), nhp_(nhp), hasPose_(false)
{
  if (!nhp) {
    throw std::invalid_argument("Node handle pointer cannot be null");
  }
  
  //  TODO: Add some error checking mechanism to this
  //  for now we just hope it works
  imgTransport_ = std::make_shared<image_transport::ImageTransport>(*nhp);
  subImage_.subscribe(*imgTransport_, "image", kROSQueueSize);
  subCamInfo_.subscribe(*nhp, "camera_info", kROSQueueSize);
  subPose_.subscribe(*nhp, "pose_tags", kROSQueueSize);
  subSpectrum_.subscribe(*nhp, "spectrum", kROSQueueSize);
  
  sync_ = std::make_shared<message_filters::Synchronizer<TimeSyncPolicy>>(TimeSyncPolicy(kROSQueueSize),
                                                                          subImage_,
                                                                          subCamInfo_,
                                                                          subPose_,
                                                                          subSpectrum_);
  sync_->registerCallback(boost::bind(&SpectrumCalibrator::syncCallback, this, _1, _2, _3, _4));
}

void SpectrumCalibrator::setSpectrometerPose(const galt::SpectrometerPose& pose) {
  pose_ = pose;
  hasPose_ = true;  //  todo: this is ugly, find a better way...
}

void SpectrumCalibrator::syncCallback(const sensor_msgs::ImageConstPtr& img,
                  const sensor_msgs::CameraInfoConstPtr &info, 
                  const geometry_msgs::PoseStampedConstPtr &poseStamped,
                  const ocean_optics::SpectrumConstPtr& spec)
{
  cv_bridge::CvImageConstPtr bridgedImagePtr = cv_bridge::toCvCopy(img,"rgb8");
  if (!bridgedImagePtr) {
    ROS_ERROR("Failed to convert image to rgb8 with cv_bridge");
    return;
  }  
  cv::Mat image = bridgedImagePtr->image;
  
  if (hasPose_) {
    
    
    
  }
  
  emit receivedMessage();
}
