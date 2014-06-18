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

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <error_handling.hpp> //  galt error handling

#include "AttitudeESKF.hpp"

using namespace std;
using namespace Eigen;

AttitudeESKF eskf;
ros::Publisher pubImu;
ros::Publisher pubAccel;

boost::shared_ptr<tf::TransformBroadcaster> tfBroadcaster;  //  for broadcasting body frame
std::string bodyFrameName;

void imu_callback(const sensor_msgs::ImuConstPtr& imu)
{  
  Vector3f wm;
  Vector3f am;
  
  wm[0] = imu->angular_velocity.x;
  wm[1] = imu->angular_velocity.y;
  wm[2] = imu->angular_velocity.z;
  
  am[0] = imu->linear_acceleration.x;
  am[1] = imu->linear_acceleration.y;
  am[2] = imu->linear_acceleration.z;
  
  eskf.predict(wm,imu->header.stamp.toSec());
  eskf.update(am);
  
  quat Q = eskf.getState(); //  updated quaternion
    
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
      filtImu.orientation_covariance[i*3 + j] = eskf.m_P(i,j);
    }
  }
  
  pubImu.publish(filtImu);
  
  //  publish predicted acceleration
  geometry_msgs::Vector3 ap;
  ap.x = eskf.predAccel_[0];
  ap.y = eskf.predAccel_[1];
  ap.z = eskf.predAccel_[2];
  pubAccel.publish(ap);
  
  //  broadcast frame
  tf::Transform transform;
  transform.setRotation( tf::Quaternion(Q.b(), Q.c(), Q.d(), Q.a()) );
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  
  tfBroadcaster->sendTransform( tf::StampedTransform(transform, filtImu.header.stamp, "fixedFrame", bodyFrameName) );
}

void field_callback(const sensor_msgs::MagneticFieldConstPtr& field)
{

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
  pubAccel = nh.advertise<geometry_msgs::Vector3>("predicted_accel", 1);
  
  tfBroadcaster = boost::shared_ptr<tf::TransformBroadcaster>( new tf::TransformBroadcaster() );
  bodyFrameName = ros::this_node::getName() + "/bodyFrame";
  
  ros::spin();
  return 0;
}
