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

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <error_handling.hpp> //  galt error handling

#include "AttitudeESKF.hpp"

using namespace std;
using namespace Eigen;

AttitudeESKF eskf;
ros::Publisher pubImu;
ros::Publisher pubAccel;
ros::Publisher pubBias;
ros::Publisher pubAlpha;

boost::shared_ptr<tf::TransformBroadcaster> tfBroadcaster;  //  for broadcasting body frame
std::string bodyFrameName;

void imu_callback(const sensor_msgs::ImuConstPtr& imu)
{  
  Vector3d wm;
  Vector3d am;
  
  wm[0] = imu->angular_velocity.x;
  wm[1] = imu->angular_velocity.y;
  wm[2] = imu->angular_velocity.z;
  
  am[0] = imu->linear_acceleration.x;
  am[1] = imu->linear_acceleration.y;
  am[2] = imu->linear_acceleration.z;
  
  eskf.predict(wm,imu->header.stamp.toSec());
  eskf.update(am);
 
  quat Q = eskf.getQuat(); //  updated quaternion
    
  //  publish IMU topic
  sensor_msgs::Imu filtImu;
  filtImu.header.stamp = ros::Time::now();
  filtImu.linear_acceleration = imu->linear_acceleration;
  filtImu.linear_acceleration_covariance = imu->linear_acceleration_covariance;
  filtImu.angular_velocity = imu->angular_velocity;
  filtImu.angular_velocity_covariance = imu->angular_velocity_covariance;
  
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
  
  //  publish predicted acceleration
  geometry_msgs::Vector3Stamped ap;
  ap.header.stamp = filtImu.header.stamp;
  ap.vector.x = 0.0;//eskf.predAccel_[0];
  ap.vector.y = 0.0;//eskf.predAccel_[1];
  ap.vector.z = 0.0;//eskf.predAccel_[2];
  pubAccel.publish(ap);
  
  //  publish bias
  geometry_msgs::Vector3Stamped bias;
  bias.header.stamp = filtImu.header.stamp;
  bias.vector.x = eskf.getGyroBias()[0];
  bias.vector.y = eskf.getGyroBias()[1];
  bias.vector.z = eskf.getGyroBias()[2];
  pubBias.publish(bias);
  
  //  broadcast frame
  tf::Transform transform;
  transform.setRotation( tf::Quaternion(Q.b(), Q.c(), Q.d(), Q.a()) );
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  
  tfBroadcaster->sendTransform( tf::StampedTransform(transform, filtImu.header.stamp, "fixedFrame", bodyFrameName) );
}

void field_callback(const sensor_msgs::MagneticFieldConstPtr& field)
{
  //log_i("%f, %f, %f", field->magnetic_field.x, field->magnetic_field.y, field->magnetic_field.z);
}

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
  ros::Subscriber imuSub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 1, &imu_callback);
  ros::Subscriber fieldSub = nh.subscribe<sensor_msgs::MagneticField>(field_topic, 1, &field_callback);
    
  //  filtered IMU output
  pubImu = nh.advertise<sensor_msgs::Imu>("filtered_imu", 1);
  pubAccel = nh.advertise<geometry_msgs::Vector3Stamped>("predicted_accel", 1);
  pubBias = nh.advertise<geometry_msgs::Vector3Stamped>("bias", 1);
  
  tfBroadcaster = boost::shared_ptr<tf::TransformBroadcaster>( new tf::TransformBroadcaster() );
  bodyFrameName = ros::this_node::getName() + "/bodyFrame";
  
  AttitudeESKF::VarSettings var;
  var.accel[0] = var.accel[1] = var.accel[2] = 1.0;
  var.gyro[0] = var.gyro[1] = var.gyro[2] = 0.001;
  var.gyroBias[0] = var.gyroBias[1] = var.gyroBias[2] = 0.001;
  var.mag[0] = var.mag[1] = var.mag[2] = 0.01;
  eskf.setVariances(var);
  
  ros::spin();
  return 0;
}
