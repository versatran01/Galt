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

#include <QDateTime>

#include "spectrumcalibrator.h"
#include <cv_bridge/cv_bridge.h>
#include "utilities.hpp"
#include <kr_math/pose.hpp>
#include <kr_math/SO3.hpp>

#include <Eigen/Dense>

//  least squares fit of form y = mx + b
static bool leastSquaresFit(const std::vector<double>& x,
                            const std::vector<double>& y,
                            double& m,
                            double& b) {
  
  assert(x.size() == y.size() && x.size() > 0);
  
  Eigen::Matrix<double,Eigen::Dynamic,2> X(x.size(), 2);
  Eigen::Matrix<double,Eigen::Dynamic,1> Y(y.size(), 1);
  
  for (size_t i=0; i < x.size(); i++) {
    X(i,0) = x[i];
    X(i,1) = 1;
    Y(i,0) = y[i];
  }
  
  Eigen::Matrix<double,2,2> H = X.transpose() * X;
  Eigen::Matrix<double,2,2> Hinv;
  
  bool invertible;
  H.computeInverseWithCheck(Hinv,invertible);
  if (!invertible) {
    return false;
  }
  
  Eigen::Matrix<double,2,1> sol = Hinv * X.transpose() * Y;
  m = sol[0];
  b = sol[1];
  return true;
}

SpectrumCalibrator::SpectrumCalibrator(QObject *parent, const std::string &cameraSerial,
                                       const galt::SpectrometerPose &specPose,
                                       const galt::FilterProfile &filterProfile)
    : QObject(parent), cameraSerial_(cameraSerial), specPose_(specPose), 
      filterProfile_(filterProfile) {
  
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
  
  hasMeasurement_ = false;
  currentReflectance_ = 0.0;
  hasSource_ = false;
  hasCalibration_ = false;
}

bool SpectrumCalibrator::hasMeasurement() const { return hasMeasurement_; }

bool SpectrumCalibrator::hasSourceSpectrum() const { return hasSource_; }

bool SpectrumCalibrator::hasCalibration() const { return hasCalibration_; } 

bool SpectrumCalibrator::canCalibrate() const { return observations_.size() >= 2; }

void SpectrumCalibrator::setSource() {
  specSource_ = spectrum_;
  hasSource_ = true;
}

void SpectrumCalibrator::setCurrentReflectance(double currentReflectance) {
  currentReflectance_ = currentReflectance;
}

void SpectrumCalibrator::collectSample() {
  
  //  collect a sample and generate observations
  if (hasMeasurement_ && hasSource_) {
    addObservation();
  }
}

//  TODO: better error handling here too
void SpectrumCalibrator::calibrate() {
 if (!canCalibrate()) {
   throw std::runtime_error("More observations required before calibrating");
 }
 
 ROS_INFO("Calibrating..."); 
  
 // camera first
 std::vector<double> X,Y;
 
 for (const Observation& O : observations_) {
   X.push_back(O.intensityCam);
   Y.push_back(O.reflectance);
 }
 
 double camM,camB;
 if (!leastSquaresFit(X,Y,camM,camB)) {
   ROS_ERROR("Failed to solve linear system (more samples required?)");
   return;
 }
 
 // save the calibration
 ros::NodeHandle nh("~");
 int exposure;
 if (!nh.hasParam("camera_exposure")) {
   ROS_ERROR("No camera exposure specified!");
   return;
 }
 nh.param("camera_exposure", exposure, 0);
 
 cameraCalibration_.cameraSerial = cameraSerial_;
 cameraCalibration_.cameraExposure = exposure;
 
 // get the date from QT
 QDateTime dt = QDateTime::currentDateTime();
 QString str = dt.toString(Qt::ISODate);
 const std::string formattedDate(str.toAscii());
 
 cameraCalibration_.calibrationDate = formattedDate;
 cameraCalibration_.intercept = camB;
 cameraCalibration_.slope = camM;
 cameraCalibration_.filterProfile = filterProfile_;
 cameraCalibration_.spectrometerPose = specPose_;
 hasCalibration_ = true;
}

const std::string& SpectrumCalibrator::getCameraSerial() const { return cameraSerial_; }

const cv::Mat &SpectrumCalibrator::getUserImage() const { return rgbImage_; }

const galt::Spectrum &SpectrumCalibrator::getSpectrum() const {
  return spectrum_;
}

const galt::Spectrum &SpectrumCalibrator::getSourceSpectrum() const {
  return specSource_;
}

const galt::FilterProfile &SpectrumCalibrator::getFilterProfile() const {
  return filterProfile_;
}

const std::vector<SpectrumCalibrator::Observation>& SpectrumCalibrator::getObservations() const {
  return observations_;
}

const galt::CameraCalibration& SpectrumCalibrator::getCameraCalibration() const {
  return cameraCalibration_;
}

void SpectrumCalibrator::calcSampleRegion(kr::vec2d& center, double& radius) const {
 
  const kr::vec3d o = camPose_.p;
  const kr::vec3d n = camPose_.q.matrix() * kr::vec3d(0,0,1);
  
  //  TODO: refactor this...
  const double d = specPose_.distanceToPlane(o,n);
  kr::vec3d p_cam = specPose_.getPosition() + specPose_.getDirection()*d;
  
  p_cam[0] /= p_cam[2];
  p_cam[1] /= p_cam[2]; //  project
  
  //  distort
  cv::Point2d dist = distortPoint(cameraInfo_.D, cv::Point2d(p_cam[0],p_cam[1]));
  //cv::Point2d dist = cv::Point2d(p_cam[0], p_cam[1]);
  
  //  camera intrinsics
  double fx = cameraInfo_.K[0];
  double cx = cameraInfo_.K[2];
  double fy = cameraInfo_.K[4];
  double cy = cameraInfo_.K[5];
    
  center[0] = dist.x*fx + cx;
  center[1] = dist.y*fy + cy;
  radius = std::tan(specPose_.getFov()/2) * d * 0.5 * (fx+fy) / p_cam[2];
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
  monoImage_ = bridgedImagePtr->image;

  //  convert to intensity
  cv::cvtColor(monoImage_, rgbImage_, CV_GRAY2RGB);
  cameraInfo_ = *info;

  //  spectrometer measurement
  spectrum_ = galt::Spectrum(spec->wavelengths, spec->spectrum);
  
  //  project the spectrometer view into the image
  kr::Pose<double> pose(poseStamped->pose);
  
  //  AprilTags produce pose of camera in tag frame, so reverse it
  camPose_ = pose.inverse();  
  hasMeasurement_ = true;
  
  //  calculate circle for user image update
  calcSampleRegion(specCenter_, specRadius_);
    
  if (specRadius_ > 0.0) {
    //  draw circle on image
    cv::circle(rgbImage_, cv::Point2d(specCenter_[0],specCenter_[1]), 
        specRadius_, cv::Scalar(0,255,0), 3);
    
    //ROS_INFO("Circle: %f, %f, %f", specCenter_[0], specCenter_[1], specRadius_);
  } else {
    ROS_WARN("Radius is negative");
  }
  
  emit receivedMessage();
}

void SpectrumCalibrator::addObservation() {
  
  //  sample from the image in the circle
  const kr::vec2d& point = specCenter_;
  const double& radius = specRadius_;
  
  int min_x = std::max( std::floor(point[0] - radius), 0.0 );
  int min_y = std::max( std::floor(point[1] - radius), 0.0 );
  int max_x = std::min( static_cast<int>(std::ceil(point[0] + radius)), monoImage_.cols );
  int max_y = std::min( static_cast<int>(std::ceil(point[1] + radius)), monoImage_.rows );
 
  std::vector<double> pixels;
  
  for (int y=min_y; y < max_y; y++) {
    for (int x=min_x; x < max_x; x++) {
      double r2 = (y-min_y)*(y-min_y) + (x-min_x)*(x-min_x);
      if (r2 < radius*radius*0.9) {
        // point is in circle, sample
        const double I = monoImage_.at<uchar>(y,x) * (1 / 255.0);
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
  const double mean = total / pixels.size();  //  mean intensity over the patch
  
  //const double mean_sqr = total2 / pixels.size();
  //const double var = mean_sqr - mean*mean;
  
  Observation O;
  O.intensityCam = mean;
  O.spectrum = spectrum_;
  O.reflectance = currentReflectance_;
  observations_.push_back(O);
}

