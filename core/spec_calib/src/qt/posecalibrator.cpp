/*
 * posecalibrator.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 11/7/2014
 *      Author: gareth
 */

#include "posecalibrator.h"

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <cv_bridge/cv_bridge.h>

//  TODO: copied from CameraIntx, refactor this into utility class later
template <typename Scalar>
cv::Point_<Scalar> distortPoint(const std::vector<Scalar>& coeffs, const cv::Point_<Scalar>& src)
{
  Scalar k[8] = {0,0,0,0,0,0,0,0};
  for (size_t i=0; i < coeffs.size(); i++) {
    k[i] = coeffs[i];
  }
  
  const Scalar xx = src.x*src.x;
  const Scalar yy = src.y*src.y;
  const Scalar xy = src.x*src.y;
  
  const Scalar r2 = xx + yy;
  const Scalar rad = (1 + r2*(k[0] + r2*(k[1] + k[4]*r2))) / (1 + r2*(k[5] + r2*(k[6] + k[7]*r2)));
  
  //  distort
  const Scalar x = src.x*rad + 2*k[2]*xy + k[3]*(r2 + 2*xx);
  const Scalar y = src.y*rad + k[2]*(r2 + 2*yy) + 2*k[3]*xy;
  
  cv::Point_<Scalar> dst;
  dst.x = x;
  dst.y = y;
  return dst;
}

PoseCalibrator::PoseCalibrator(QObject *parent, const ros::NodeHandlePtr &nhp) :
  QObject(parent), l0_(0,0,0), ln_(0,0,0), fov_(0), err_(0), hasCalibration_(false)
{
  if (!nhp) {
    throw std::invalid_argument("Node handle pointer cannot be null");
  }
  
  //  load options
  nhp->param("depth_threshold", depthThreshold_, 0.1);
  nhp->param("pixel_threshold", pixelThreshold_, 3.0);
  
  imgTransport_ = std::make_shared<image_transport::ImageTransport>(*nhp);
  subImage_.subscribe(*imgTransport_, "image", kROSQueueSize);
  subCamInfo_.subscribe(*nhp, "camera_info", kROSQueueSize);
  subCircles_.subscribe(*nhp, "circles", kROSQueueSize);
  subPose_.subscribe(*nhp, "pose", kROSQueueSize);
  subPixels_.subscribe(*nhp, "pixels", kROSQueueSize);
  
  //  subscribe to synchronized topics
  sync_ = std::make_shared<message_filters::Synchronizer<TimeSyncPolicy>>(TimeSyncPolicy(kROSQueueSize), 
                                                                          subImage_, 
                                                                          subCamInfo_, 
                                                                          subCircles_,
                                                                          subPose_,
                                                                          subPixels_);
  sync_->registerCallback( boost::bind(&PoseCalibrator::syncCallback, this, _1, _2, _3, _4, _5) );  
  
  ROS_INFO("Subscribing to ~image, ~camera_info, ~circles, ~pose and ~pixels");
}

PoseCalibrator::~PoseCalibrator() {
}

const cv::Mat& PoseCalibrator::lastImage() const {
  return image_;
}

size_t PoseCalibrator::observationCount() const {
  return observations_.size();
}

kr::vec3<double> PoseCalibrator::getLineOrigin() const {
  return l0_;
}

kr::vec3<double> PoseCalibrator::getLineNormal() const {
  return ln_;
}

double PoseCalibrator::getError() const {
  return err_;
}

bool PoseCalibrator::canCalibrate() const {
  return observationCount() >= 2;
}

void PoseCalibrator::calibrate() {
  if (!canCalibrate()) {
    throw std::runtime_error("calibrate() called before sufficient points collected");
  }
  
  //  sort observations by depth
  std::sort(observations_.begin(), observations_.end());
  
  //  take all observations and fit them to a line w/ least squares
  std::vector<cv::Point3f> camPoints;
  for (const Observation& o : observations_) {
    camPoints.push_back(cv::Point3f(o.p.x,o.p.y,o.p.z));
  }
  
  std::vector<float> line(6, 0.0f);
  cv::fitLine(camPoints,line,CV_DIST_L2,0,0.01,0.01);
  
  ln_ = kr::vec3<double>(line[0],line[1],line[2]);
  l0_ = kr::vec3<double>(line[3],line[4],line[5]);
  
  //  calculate error in calibration
  err_ = 0.0;
  for (const Observation& o : observations_) {
    kr::vec3<double> del = l0_ - kr::vec3<double>(o.p.x,o.p.y,o.p.z);
    kr::vec3<double> perp = (del - ln_ * (del.transpose() * ln_));
    err_ += perp[0]*perp[0] + perp[1]*perp[1] + perp[2]*perp[2];
  }
  err_ = std::sqrt(err_);
  
  //  calculate spectrometer position in camera coordinates
  double lambda = -(l0_[0]*ln_[0] + l0_[1]*ln_[1] + l0_[2]*ln_[2]);
  kr::vec3<double> specPos = l0_ + ln_*lambda;
  
  ROS_INFO("Spectrometer position: %f, %f, %f", specPos[0], specPos[1], specPos[2]);
  
  l0_ = specPos;  //  <- pick spectrometer centre as the new point on the line
  hasCalibration_=true;
}

bool PoseCalibrator::hasCalibration() const {
  return hasCalibration_;
}

std::pair<PoseCalibrator::Circle, bool> PoseCalibrator::projectWithPose(const kr::Pose<double>& pose) {
  if (!hasCalibration()) {
    throw std::runtime_error("projectWithPose() called before calibration completed");
  }
  
  //  determine plane position
  kr::vec3<double> o = pose.p;
  kr::vec3<double> n = pose.q.matrix() * kr::vec3<double>(0,0,1);
  
  //  intersect to get position in camera
  kr::vec3<double> del = o - l0_;
  double d = (del[0]*n[0] + del[1]*n[1] + del[2]*n[2]);
  d /= (ln_[0]*n[0] + ln_[1]*n[1] + ln_[2]*n[2]);

  kr::vec3<double> p_cam = l0_ + ln_*d;
  //ROS_INFO("Projected point: %f, %f, %f\n", p_cam[0], p_cam[1], p_cam[2]);
  
  if (p_cam[2] < 0.0f) {
    ROS_WARN("Got a shit solution yo!");
    return std::pair<Circle,bool>(Circle(),false);
  }
  
  double fx = camInfo_->K[0];
  double cx = camInfo_->K[2];
  double fy = camInfo_->K[4];
  double cy = camInfo_->K[5];
  
  Circle circle;
  
  cv::Point2d proj;
  proj.x = p_cam[0] / p_cam[2];
  proj.y = p_cam[1] / p_cam[2];
  circle.xy = proj;
  circle.radiusCam = 2*std::tan(fov_/2) * p_cam[2];
    
  proj = distortPoint(camInfo_->D, proj);
  proj.x = fx*proj.x + cx;
  proj.y = fy*proj.y + cy;
  circle.uv = proj;
  circle.radiusPix = circle.radiusCam * (fx+fy)/2;
  
  return std::pair<Circle, bool>(circle, true);
}

void PoseCalibrator::syncCallback(const sensor_msgs::ImageConstPtr& img,
                  const sensor_msgs::CameraInfoConstPtr &info, 
                  const circle_tracker::CirclesConstPtr &circ, 
                  const geometry_msgs::PoseWithCovarianceStampedConstPtr &poseWithCov,
                  const monocular_pose_estimator::PixelArrayConstPtr& pixels)
{
  //  convert image to OpenCV format
  cv_bridge::CvImageConstPtr bridgedImagePtr = cv_bridge::toCvCopy(img,"rgb8");
  if (!bridgedImagePtr) {
    ROS_ERROR("Failed to convert image to rgb8 with cv_bridge");
    return;
  }  
  cv::Mat image = bridgedImagePtr->image;
  camInfo_ = info;
  
  //  camera intrinsics
  double fx = info->K[0];
  double cx = info->K[2];
  double fy = info->K[4];
  double cy = info->K[5];
  double f = (fx+fy)/2;
  std::vector<double> distCoeffs = info->D;
  
  //  check all circles against LEDs
  std::vector<cv::Point2d> leds;
  for (const monocular_pose_estimator::Pixel& pix : pixels->pixels) {
    leds.push_back( cv::Point2d(pix.u,pix.v) );
  }
  
  std::vector<circle_tracker::Circle> circlesIn = circ->circles;
  std::vector<Circle> circlesCulled;
  
  for (auto I = circlesIn.begin(); I != circlesIn.end(); I++) {
    cv::Point2d pc(I->centerX, I->centerY);
    
    //  convert to pixel coordinates
    pc = distortPoint(distCoeffs, pc);
    pc.x = fx*pc.x + cx;
    pc.y = fy*pc.y + cy;
    
    bool collision = false;
    for (const cv::Point2d& pl : leds) {
      double dist = std::sqrt((pc.x-pl.x)*(pc.x-pl.x) + (pc.y-pl.y)*(pc.y-pl.y));
      if (dist < I->radius*f) {
        collision = true;
        break;
      }
    }
    
    //  discard cases where ambiguity between LED & Laser exists
    if (!collision) {
      Circle C;
      C.uv = pc;
      C.xy = cv::Point2d(I->centerX, I->centerY);
      C.radiusPix = I->radius*f;
      C.radiusCam = I->radius;
      circlesCulled.push_back(C);
    }
  }
  
  //  sort remaining circles by area, lowest to highest, picking the largest
  kr::Pose<double> pose(poseWithCov->pose.pose);
  if (!circlesCulled.empty())
  {
    std::sort(circlesCulled.begin(), circlesCulled.end());
    Circle circle = circlesCulled.back();
    
    //  add to observations
    addObservation(pose,circle);
       
    cv::circle(image, circle.uv, circle.radiusPix, cv::Scalar(255,0,0), 3);
  }
  
  //  highlight the LEDs
  for (const monocular_pose_estimator::Pixel& pix : pixels->pixels) {
    cv::Point2d p(pix.u, pix.v);
    cv::circle(image, p, 7.0, cv::Scalar(0,0,255), 2);
  }
  
  if (hasCalibration()) {
    std::pair<Circle,bool> output = projectWithPose(pose);
    if (output.second) {
      cv::circle(image, output.first.uv, output.first.radiusPix, cv::Scalar(0,255,0), 2);
    }
  }
  
  image_ = image;
  
  //  notify listeners
  emit receivedMessage();
}

bool PoseCalibrator::addObservation(const kr::Pose<double> &pose, const Circle& circle) {
    
  kr::vec3<double> v; //  vector to feature, camera frame
  v = kr::vec3<double>(circle.xy.x, circle.xy.y, 1.0);
  v /= v.norm();
  
  //  plane values, in camera frame
  kr::vec3<double> o = pose.p;  //translation();
  kr::vec3<double> n = pose.q.matrix() * kr::vec3<double>(0,0,1);
   
  //  intersect for depth
  double d = (o[0]*n[0] + o[1]*n[1] + o[2]*n[2]) / (v[0]*n[0] + v[1]*n[1] + v[2]*n[2]);
  v *= d;
  
  if (v[2] < 0.0) {
    return false; //  reject positive signed solutions
  }
    
  auto compare = [=](const Observation& obvs) -> bool {
    if (std::abs(obvs.depth - d) < depthThreshold_) { //  depth threshold
      return true;
    }
    
    double delx = obvs.circle.uv.x - circle.uv.x;
    double dely = obvs.circle.uv.y - circle.uv.y;
    if (delx*delx + dely*dely < pixelThreshold_*pixelThreshold_) {
      return true;
    }
      
    return false;
  };
 
  if (!std::any_of(observations_.begin(), observations_.end(), compare)) {
    Observation obvs;
    obvs.circle = circle;
    obvs.depth = d;
    obvs.p = cv::Point3d(v[0],v[1],v[2]);
    observations_.push_back(obvs);
    
    ROS_INFO("Adding observation: %f, %f, %f", v[0], v[1], v[2]);
    return true;
  }
  
  return false;
}
