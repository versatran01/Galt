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
#include <spectral/CameraCalibration.hpp>
#include <spectral/SpectrometerCalibration.hpp>

#include <kr_math/pose.hpp>

class SpectrumCalibrator : public QObject {
  Q_OBJECT
public:
  
  //  both camera and spectrometer measurement
  struct Observation {
    double intensityCam;    
    galt::SpectrometerCalibration::Sample sample;
  };
  
  explicit SpectrumCalibrator(QObject *parent,
                              const std::string &cameraSerial,
                              const galt::SpectrometerPose &specPose,
                              const galt::FilterProfile &filterProfile);

  bool hasMeasurement() const;
  
  bool hasSourceSpectrum() const;
  
  bool hasSpectrum() const;
  
  bool hasCalibration() const;
  
  bool canCalibrate() const;
  
  void setSource();
  
  void setCurrentReflectance(double currentReflectance);
  
  void collectSample();
  
  void calibrate();

  const std::string& getCameraSerial() const;
  
  const kr::Pose<double>& getPose() const;
  
  const cv::Mat &getUserImage() const;

  const galt::Spectrum &getSpectrum() const;

  const galt::Spectrum &getSourceSpectrum() const;
  
  const galt::FilterProfile &getFilterProfile() const;
  
  const std::vector<Observation>& getObservations() const;
  
  const galt::CameraCalibration& getCameraCalibration() const;
  
signals:
  void receivedMessage();
  void receivedSpectrum();

public slots:

private:
  
  void calcSampleRegion(kr::vec2d& center, double& radius) const;
  
  std::shared_ptr<image_transport::ImageTransport> imgTransport_;
  
  //  measurements  
  cv::Mat monoImage_, rgbImage_;
  sensor_msgs::CameraInfo cameraInfo_;
  kr::Pose<double> camPose_;
  galt::Spectrum spectrum_;
  bool hasSpectrum_;
  kr::vec2d specCenter_;
  double specRadius_;
  bool hasMeasurement_;
  
  //  settings
  std::string cameraSerial_;
  galt::SpectrometerPose specPose_;
  galt::FilterProfile filterProfile_;
  
  //  source spectrum
  galt::Spectrum specSource_;
  double currentReflectance_;
  bool hasSource_;

  //  measurements  
  std::vector<Observation> observations_;
  
  //  resulting calibration
  galt::CameraCalibration cameraCalibration_;
  bool hasCalibration_;
    
  //  ROS subscribers
  static constexpr uint32_t kROSQueueSize = 10;

  image_transport::SubscriberFilter subImage_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> subCamInfo_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> subPose_;
  //message_filters::Subscriber<ocean_optics::Spectrum> subSpectrum_;

  ros::Subscriber subSpectrum_;
  
  //  time sync policy
  typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image, sensor_msgs::CameraInfo, geometry_msgs::PoseStamped> TimeSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<TimeSyncPolicy>> sync_;

  //  synchronized callback
  void syncCallback(const sensor_msgs::ImageConstPtr &img,
                    const sensor_msgs::CameraInfoConstPtr &info,
                    const geometry_msgs::PoseStampedConstPtr &poseStamped);
  
  void spectrumCallback(const ocean_optics::SpectrumConstPtr &spec);
  
  void addObservation();
};

#endif // SPECTRUMCALIBRATOR_H
