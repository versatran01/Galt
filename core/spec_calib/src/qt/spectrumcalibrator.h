/*
 * SpectrumCalibrator.h
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 17/7/2014
 *      Author: gareth
 */

#ifndef SPECTRUMCALIBRATOR_H
#define SPECTRUMCALIBRATOR_H

#include <QObject>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <ocean_optics/Spectrum.h>

#include <spectral/SpectrometerPose.hpp>
#include <spectral/Spectrum.hpp>
#include <spectral/FilterProfile.hpp>

#include <kr_math/pose.hpp>

class SpectrumCalibrator : public QObject {
  Q_OBJECT
public:
  explicit SpectrumCalibrator(QObject *parent,
                              const galt::SpectrometerPose &specPose,
                              const galt::FilterProfile &filterProfile);

  const cv::Mat &lastImage() const;

  const galt::Spectrum &lastSpectrum() const;

  const galt::FilterProfile &getFilterProfile() const;

  const galt::Spectrum &getPredictedSpectrum() const;
  
signals:
  void receivedMessage();

public slots:

private:
  std::shared_ptr<image_transport::ImageTransport> imgTransport_;
  cv::Mat image_;
  sensor_msgs::CameraInfo cameraInfo_;

  galt::Spectrum spectrum_;
  galt::SpectrometerPose specPose_;
  galt::FilterProfile filterProfile_;
  galt::Spectrum predictedSpectrum_;

  //  ROS subscribers
  static constexpr uint32_t kROSQueueSize = 100;

  image_transport::SubscriberFilter subImage_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> subCamInfo_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> subPose_;
  message_filters::Subscriber<ocean_optics::Spectrum> subSpectrum_;

  //  time sync policy
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::CameraInfo, geometry_msgs::PoseStamped,
      ocean_optics::Spectrum> TimeSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<TimeSyncPolicy>> sync_;

  //  synchronized callback
  void syncCallback(const sensor_msgs::ImageConstPtr &img,
                    const sensor_msgs::CameraInfoConstPtr &info,
                    const geometry_msgs::PoseStampedConstPtr &poseStamped,
                    const ocean_optics::SpectrumConstPtr &spec);
  
  void addObservation(const kr::vec2d& point, double radius, const galt::Spectrum& spectrum, const cv::Mat &monoImage);
};

#endif // SPECTRUMCALIBRATOR_H
