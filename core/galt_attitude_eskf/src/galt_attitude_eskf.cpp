/*
 * galt_attitude_eskf.cpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 17/6/2014
 *		  Author: gareth
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <error_handling.hpp> //  galt error handling

#include "AttitudeESKF.hpp"

using namespace std;
using namespace Eigen;

AttitudeESKF eskf;
ros::Publisher pubImu;
ros::Publisher pubBias;
ros::Publisher pubAlpha;
ros::Publisher pubField;
ros::Publisher pubPredField;

boost::shared_ptr<tf::TransformBroadcaster> tfBroadcaster;  //  for broadcasting body frame
std::string bodyFrameName;

enum {
  MagNeedsCalibration=0,
  MagCalibrated=1,
} topicMode = MagNeedsCalibration;

#define BIN_DIM  (15)

struct SampleBin {
  Vector3d field;
  quat q;
};

std::map< std::tuple<int,int>, SampleBin > sampleBins;

Vector3d magBias = Vector3d::Zero();
Vector3d magScale = Vector3d::Ones();

//  (world) = Rz * Ry * Rx (body)
template <typename T>
Eigen::Matrix<T,3,1> getRPY(const Eigen::Matrix<T,3,3>& R)
{	
	Eigen::Matrix<T,3,1> rpy;

	T sth = -R(2,0);
	if (sth > 1.0) {
		sth = 1.0;
	} else if (sth < -1.0) {
		sth = -1.0;
	}

	const T theta = std::asin(sth);
	const T cth = std::sqrt(1.0 - sth*sth);

	T phi, psi;
	if (cth < 1e-6)
	{
		phi = std::atan2(R(0,1), R(1,1));
		psi = 0.0;
	}
	else
	{
		phi = std::atan2(R(2,1), R(2,2));
		psi = std::atan2(R(1,0), R(0,0));
	}

	rpy[0] = phi;   //  x
	rpy[1] = theta; //  y
	rpy[2] = psi;   //  z
	return rpy;
}

/**
 *  @brief X rotation matrix
 *  @param angle Angle in radians
 */
template <typename T>
static Eigen::Matrix<T,3,3> rotation_x(T angle)
{
    const T c = std::cos(angle);
    const T s = std::sin(angle);
    
    Eigen::Matrix<T,3,3> R;
    R.setZero();
    
    R(0,0) = 1.0f;
    R(1,1) = c; R(1,2) = -s;
    R(2,1) = s; R(2,2) = c;
    
    return R;
}

/**
 *  @brief Y rotation matrix
 *  @param angle Angle in radians
 */
template <typename T>
static Eigen::Matrix<T,3,3> rotation_y(T angle)
{
    const T c = std::cos(angle);
    const T s = std::sin(angle);
    
    Eigen::Matrix<T,3,3> R;
    R.setZero();
    
    R(1,1) = 1.0;
    R(0,0) = c;  R(0,2) = s;
    R(2,0) = -s; R(2,2) = c;
    
    return R;
}

/**
 * @brief geo_to_eigen Convert geometry_msg Vector3 to Eigen Vector3d
 * @param vec
 * @return Vector3d
 */
Eigen::Vector3d geo_to_eigen(const geometry_msgs::Vector3& vec) {
  Eigen::Vector3d v3d;
  v3d[0] = vec.x;
  v3d[1] = vec.y;
  v3d[2] = vec.z;
  return v3d;
}

void imu_callback(const sensor_msgs::ImuConstPtr& imu, const sensor_msgs::MagneticFieldConstPtr& field)
{  
  Vector3d wm;  //  measured angular rate
  Vector3d am;  //  measured acceleration
  Vector3d mm;  //  measured magnetic field
  
  wm = geo_to_eigen(imu->angular_velocity);
  am = geo_to_eigen(imu->linear_acceleration);
  mm = geo_to_eigen(field->magnetic_field);
  
  if (topicMode == MagNeedsCalibration) {
    eskf.setUsesMagnetometer(false);
  } 
  else if (topicMode == MagCalibrated) {
    for (int i=0; i < 3; i++) {
      mm[i] -= magBias[i];
      mm[i] /= magScale[i];
    }
    eskf.setUsesMagnetometer(true);
    
    quat Q = eskf.getQuat();
    Matrix3d R = Q.to_matrix().cast<double>();
    Vector3d rpy = getRPY(R);
    
    //log_i("heading: %f\n", rpy[2]);
    
    //  debug: calc raw heading
    Matrix3d trans = rotation_y(rpy[1]) * rotation_x(rpy[0]);    
    Vector3d mw = trans * mm;
    
    double ang = std::atan2(mw[1],mw[0]);
    //log_i("basic heading: %f\n", ang);
  }
  
  eskf.predict(wm,imu->header.stamp.toSec());
  eskf.update(am,mm);   //  mm is ignored when magnetometer uncalibrated
 
  quat Q = eskf.getQuat();                            //  updated quaternion
  AttitudeESKF::vec3 w = eskf.getAngularVelocity();   //  bias subtracted 
  
  if (topicMode == MagNeedsCalibration)
  {
    Vector3d nmag = mm;
    nmag /= nmag.norm(); // normalize and discretize
    
    float theta = std::acos(nmag[2]);
    float psi = std::atan2(nmag[1], nmag[0]);
    
    int i = std::floor( (theta / M_PI) * BIN_DIM );
    int j = std::floor( (psi + M_PI) * BIN_DIM / (2 * M_PI) );
    
    SampleBin bin;
    bin.field = mm;
    bin.q = Q;
    
    sampleBins[std::tuple<int,int>(i,j)] = bin;
    
    log_i("%lu bins are filled", sampleBins.size() );
    if ( sampleBins.size() > std::floor(BIN_DIM*BIN_DIM*0.7) ) 
    {
      log_i("Collected enough bins to calibrate");
      
      vector<Vector3d> samples;
      
      Vector3d max,min;
      min[0] = min[1] = min[2] = std::numeric_limits<double>::infinity();
      max[0] = max[1] = max[2] = -min[0];
      
      double mean_rad = 0.0;
      double mean_rad_sqr = 0.0;
      
      for (auto i = sampleBins.begin(); i != sampleBins.end(); i++)
      {
        samples.push_back(i->second.field);
        
        const double r = i->second.field.norm();
        mean_rad += r;
        mean_rad_sqr += r*r;
      }
      mean_rad /= sampleBins.size();      //  mean radius
      mean_rad_sqr /= sampleBins.size();
            
      //  standard deviation
      const float std_dev = std::sqrt(mean_rad_sqr - mean_rad*mean_rad);
      
      //  calculate bias term
      for (Vector3d& s : samples) {
        for (int j=0; j < 3; j++) {
          max[j] = std::max(max[j], s[j]);
          min[j] = std::min(min[j], s[j]);
        }
      }
      
      //  estimate of soft bias
      Vector3d bias = (max + min) / 2;               
      
      //  reject outliers
      auto j = sampleBins.begin();
      for (auto i = samples.begin(); i != samples.end();)
      {
        const Vector3d s = *i - bias;
        
        if (std::abs(s.norm() - mean_rad) > std_dev*2) {
          i = samples.erase(i);
          j = sampleBins.erase(j);
        } else {          
          i++;
          j++;
        }
      }
            
      log_i("%lu samples left after discarding outliers", samples.size());
      log_i("bias: %f, %f, %f", bias[0], bias[1], bias[2]);
            
      //  attempt to determine scale factors via GN-NLS
      Vector3d scl;
      scl.setOnes();
      
      int num_iter = 0;
      while (num_iter++ < 10)
      {
        MatrixXd J(samples.size(), 6);
        VectorXd r(samples.size());
        
        for (size_t i=0; i < samples.size(); i++)
        {
          double x = (samples[i][0] - bias[0]) / (scl[0]);
          double y = (samples[i][1] - bias[1]) / (scl[1]);
          double z = (samples[i][2] - bias[2]) / (scl[2]);
          
          double rad2 = x*x + y*y + z*z;
          r[i] = mean_rad*mean_rad - rad2;
          
          J(i,0) = -2 * x*x / scl[0];
          J(i,1) = -2 * y*y / scl[1];
          J(i,2) = -2 * z*z / scl[2];
          
          J(i,3) = -2 * x / scl[0];
          J(i,4) = -2 * y / scl[1];
          J(i,5) = -2 * z / scl[2];
        }
        
        Matrix<double,6,6> H = J.transpose() * J;
        for (int i=0; i < H.rows(); i++) {
          H(i,i) *= 1.01;
        }
        
        bool invertible;
        decltype(H) Hinv;
        
        auto LU = H.fullPivLu();
        invertible = LU.isInvertible();
        Hinv = LU.inverse();
                
        Matrix<double,6,1> update = Hinv * J.transpose() * r;
        
        bias += update.block<3,1>(3,0);
        scl += update.block<3,1>(0,0);
        
        if (!invertible) {
          log_e("Failed to optimize");
          return;
        }
      }
      
      //bias.setZero();
      //scl.setOnes();
      
      magBias = bias;
      magScale = scl;
      
      log_i("Adjusted scale: %f, %f, %f", scl[0], scl[1], scl[2]);
      log_i("Adjusted bias: %f, %f, %f", bias[0], bias[1], bias[2]);
      
      double hAvg = 0.0;
      double vAvg = 0.0;
      long count=0;
      
      //  determine the inertial field
      for (auto i = sampleBins.begin(); i != sampleBins.end(); i++) 
      {  
        const SampleBin& bin = i->second;
        
        Matrix3d wRb = bin.q.to_matrix().cast<double>();
        
        Vector3d acc = wRb.transpose().block<3,1>(0,2); //  body frame gravity vector
        if (acc[2] < -0.8f) 
        { 
          //  close to the vertical, use this sample
          Vector3d rpy = getRPY(wRb);
          
          //  rotate back to level
          Matrix3d trans = rotation_y(rpy[1]) * rotation_x(rpy[0]);
          Vector3d f = bin.field;
          
          for (int j=0; j < 3; j++) {
            f[j] -= bias[j];
            f[j] /= scl[j];
          }
          
          Vector3d mw = trans * f;
          
          double horz = std::sqrt(mw[0]*mw[0] + mw[1]*mw[1]);
          double vert = mw[2];
          
          hAvg += horz;
          vAvg += vert;
          count++;
          
          log_i("H: %f, V: %f", horz, vert);
        } 
      }
      
      hAvg /= count;
      vAvg /= count;
      
      Eigen::Matrix<double,3,1> vRef;
      vRef.setZero();
      vRef[0] = hAvg;
      vRef[2] = vAvg;
      
      eskf.setMagneticReference(vRef);
      
      topicMode = MagCalibrated;
    }
  }
  
  //  publish IMU topic
  sensor_msgs::Imu filtImu;
  filtImu.header.stamp = ros::Time::now();
  
  filtImu.linear_acceleration = imu->linear_acceleration;
  filtImu.linear_acceleration_covariance = imu->linear_acceleration_covariance;
  filtImu.angular_velocity_covariance = imu->angular_velocity_covariance;
  
  filtImu.angular_velocity.x = w[0];
  filtImu.angular_velocity.y = w[1];
  filtImu.angular_velocity.z = w[2];
  
  filtImu.orientation.w = Q.a();
  filtImu.orientation.x = Q.b();
  filtImu.orientation.y = Q.c();
  filtImu.orientation.z = Q.d();
  
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      filtImu.orientation_covariance[i*3 + j] = eskf.getCovariance()(i,j);
    }
  }
  
  pubImu.publish(filtImu);
  
  //  publish bias
  geometry_msgs::Vector3Stamped bias;
  bias.header.stamp = filtImu.header.stamp;
  bias.vector.x = eskf.getGyroBias()[0];
  bias.vector.y = eskf.getGyroBias()[1];
  bias.vector.z = eskf.getGyroBias()[2];
  pubBias.publish(bias);
  
  //  publish compensated magnetic field
  sensor_msgs::MagneticField fieldOut;
  fieldOut.header.stamp = filtImu.header.stamp;
  fieldOut.magnetic_field.x = mm[0];
  fieldOut.magnetic_field.y = mm[1];
  fieldOut.magnetic_field.z = mm[2];
  pubField.publish(fieldOut);
  
  //  publish predicted magnetic field
  sensor_msgs::MagneticField predFieldOut;
  predFieldOut.header.stamp = filtImu.header.stamp;
  predFieldOut.magnetic_field.x = eskf.getPredictedField()[0];
  predFieldOut.magnetic_field.y = eskf.getPredictedField()[1];
  predFieldOut.magnetic_field.z = eskf.getPredictedField()[2];
  pubPredField.publish(predFieldOut);
  
  //  broadcast frame
  tf::Transform transform;
  transform.setRotation( tf::Quaternion(Q.b(), Q.c(), Q.d(), Q.a()) );
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  
  tfBroadcaster->sendTransform( tf::StampedTransform(transform, filtImu.header.stamp, "fixedFrame", bodyFrameName) );
}

//void field_callback(const sensor_msgs::MagneticFieldConstPtr& field)
//{
//  log_i("%f, %f, %f", field->magnetic_field.x, field->magnetic_field.y, field->magnetic_field.z);
//}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "galt_attitude_eskf");
  ros::NodeHandle nh("~");

  std::string imu_topic, field_topic;
  nh.param("imu_topic", imu_topic, std::string("/imu/imu"));
  nh.param("field_topic", field_topic, std::string("/imu/magnetic_field"));

  log_i("Subscribing to %s", imu_topic.c_str());
  log_i("Subscribing to %s", field_topic.c_str());

  //  subscribe to indicated topics
  message_filters::Subscriber<sensor_msgs::Imu> imuSub(nh, imu_topic, 1);
  message_filters::Subscriber<sensor_msgs::MagneticField> fieldSub(nh, field_topic, 1);

  message_filters::TimeSynchronizer<sensor_msgs::Imu, sensor_msgs::MagneticField> sync(imuSub, fieldSub, 2);
  sync.registerCallback(boost::bind(&imu_callback, _1, _2));
  
  //  filtered IMU output
  pubImu = nh.advertise<sensor_msgs::Imu>("filtered_imu", 1);
  pubBias = nh.advertise<geometry_msgs::Vector3Stamped>("bias", 1);
  pubField = nh.advertise<sensor_msgs::MagneticField>("adjusted_field", 1);
  pubPredField = nh.advertise<sensor_msgs::MagneticField>("predicted_field", 1);
  
  tfBroadcaster = boost::shared_ptr<tf::TransformBroadcaster>( new tf::TransformBroadcaster() );
  bodyFrameName = ros::this_node::getName() + "/bodyFrame";
  
  AttitudeESKF::VarSettings var;
  var.accel[0] = var.accel[1] = var.accel[2] = 1.0;
  var.gyro[0] = var.gyro[1] = var.gyro[2] = 0.001;
  var.mag[0] = var.mag[1] = var.mag[2] = 0.3;
  
  eskf.setVariances(var);
  eskf.setUsesMagnetometer(false);
  
  ros::spin();
  return 0;
}
