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

boost::shared_ptr<tf::TransformBroadcaster> tfBroadcaster;  //  for broadcasting body frame
std::string bodyFrameName;

enum {
  MagNeedsCalibration=0,
  MagCalibrated=1,
} topicMode = MagNeedsCalibration;

#define BIN_DIM  (15)

std::map< std::tuple<int,int>, Vector3d > sampleBins;

Vector3d magBias = Vector3d::Zero();

void imu_callback(const sensor_msgs::ImuConstPtr& imu, const sensor_msgs::MagneticFieldConstPtr& field)
{  
  Vector3d wm;  //  measured angular rate
  Vector3d am;  //  measured acceleration
  Vector3d mm;  //  measured magnetic field
  
  wm[0] = imu->angular_velocity.x;
  wm[1] = imu->angular_velocity.y;
  wm[2] = imu->angular_velocity.z;
  
  am[0] = imu->linear_acceleration.x;
  am[1] = imu->linear_acceleration.y;
  am[2] = imu->linear_acceleration.z;
  
  mm[0] = field->magnetic_field.x;
  mm[1] = field->magnetic_field.y;
  mm[2] = field->magnetic_field.z;
  
  eskf.predict(wm,imu->header.stamp.toSec());
  eskf.update(am,mm);
 
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
    
    sampleBins[std::tuple<int,int>(i,j)] = mm;
    
    log_i("%lu bins are filled", sampleBins.size() );
    if ( sampleBins.size() > std::floor(BIN_DIM*BIN_DIM*0.7) ) {
      
      log_i("Collected enough bins to calibrate");
      
      vector<Vector3d> samples;
      
      Vector3d max,min;
      min[0] = min[1] = min[2] = std::numeric_limits<double>::infinity();
      max[0] = max[1] = max[2] = -min[0];
      
      double mean_rad = 0.0;
      double mean_rad_sqr = 0.0;
      
      for (auto i = sampleBins.begin(); i != sampleBins.end(); i++)
      {
        samples.push_back(i->second);
        
        double r = i->second.norm();
        mean_rad += r;
        mean_rad_sqr += r*r;
      }
      mean_rad /= sampleBins.size();      //  mean radius
      mean_rad_sqr /= sampleBins.size();
            
      //  standard deviation
      float std_dev = std::sqrt(mean_rad_sqr - mean_rad*mean_rad);
      
      //  reject outliers
      for (auto i = samples.begin(); i != samples.end();)
      {
        if (std::abs(i->norm() - mean_rad) > std_dev*2) {
          i = samples.erase(i);
        } else {
          
          for (int j=0; j < 3; j++) {
            max[j] = std::max(max[j], (*i)[j]);
            min[j] = std::min(min[j], (*i)[j]);
          }
          
          i++;
        }
      }
      
      Vector3d bias = (max + min) / 2;    //  estimate of soft bias      
      
      log_i("%lu samples left after discarding outliers", samples.size());
      log_i("bias: %f, %f, %f", bias[0], bias[1], bias[2]);
      
      magBias = bias;
      
      //  attempt to determine scale factors via optimization
      Vector3d scl;
      scl.setOnes();
      
      int num_iter = 0;
      while (num_iter++ < 10)
      {
        MatrixXd J(samples.size(), 3);
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
        }
        
        Matrix3d H = J.transpose() * J;
        for (int i=0; i < 3; i++) {
          H(i,i) *= 1.01;
        }
        
        bool invertible;
        Matrix3d Hinv;
        H.computeInverseWithCheck(Hinv,invertible);
        
        Vector3d update = Hinv * J.transpose() * r;
        scl += update;
        
        if (!invertible) {
          log_e("Failed to optimize");
          return;
        }
      }
      
      log_i("Adjusted scale: %f, %f, %f", scl[0], scl[1], scl[2]);
      
      topicMode = MagCalibrated;
    }
  }
  else if (topicMode == MagCalibrated) {
   // mm -= magBias;
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
  
  //  publish mag bias
  sensor_msgs::MagneticField fieldOut;
  fieldOut.header.stamp = filtImu.header.stamp;
  fieldOut.magnetic_field.x = mm[0] - magBias[0];
  fieldOut.magnetic_field.y = mm[1] - magBias[1];
  fieldOut.magnetic_field.z = mm[2] - magBias[2];
  pubField.publish(fieldOut);
  
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
  
  //ros::Subscriber imuSub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 1, &imu_callback);
  //ros::Subscriber fieldSub = nh.subscribe<sensor_msgs::MagneticField>(field_topic, 1, &field_callback);
    
  message_filters::TimeSynchronizer<sensor_msgs::Imu, sensor_msgs::MagneticField> sync(imuSub, fieldSub, 2);
  sync.registerCallback(boost::bind(&imu_callback, _1, _2));
  
  //  filtered IMU output
  pubImu = nh.advertise<sensor_msgs::Imu>("filtered_imu", 1);
  pubBias = nh.advertise<geometry_msgs::Vector3Stamped>("bias", 1);
  pubField = nh.advertise<sensor_msgs::MagneticField>("adjusted_field", 1);
  
  tfBroadcaster = boost::shared_ptr<tf::TransformBroadcaster>( new tf::TransformBroadcaster() );
  bodyFrameName = ros::this_node::getName() + "/bodyFrame";
  
  AttitudeESKF::VarSettings var;
  var.accel[0] = var.accel[1] = var.accel[2] = 1.0;
  var.gyro[0] = var.gyro[1] = var.gyro[2] = 0.001;
  var.mag[0] = var.mag[1] = var.mag[2] = 0.1;
  eskf.setVariances(var);
  eskf.setUsesMagnetometer(false);
  
  ros::spin();
  return 0;
}
