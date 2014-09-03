#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include "quadrotor_ukf.h"

#include <kr_math/SO3.hpp>

ros::Publisher pub_ukf_odom;
std::string frame_id;
std::string child_frame_id;

QuadrotorUKF quadrotor_ukf;

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
  //static int calLimit = 100;
  //static int calCnt = 0;

  kr::vec3d am, wm;
  tf::vectorMsgToEigen(msg->linear_acceleration, am);
  tf::vectorMsgToEigen(msg->angular_velocity, wm);
    
  // Assemble control input
  QuadrotorUKF::InputVec u;
  u.block<3,1>(0,0) = am;
  u.block<3,1>(3,0) = wm;

  if (quadrotor_ukf.ProcessUpdate(u, msg->header.stamp))  // Process Update
  {
    nav_msgs::Odometry odom_ukf;
    odom_ukf.header.stamp = quadrotor_ukf.GetStateTime(); //  copies from header
    odom_ukf.header.frame_id = frame_id;
    odom_ukf.child_frame_id = child_frame_id;
    QuadrotorUKF::StateVec x = quadrotor_ukf.GetState();
    const kr::vec3d pos = x.block<3,1>(0,0);
    const kr::vec3d vel = x.block<3,1>(3,0);
    
    tf::pointEigenToMsg(pos, odom_ukf.pose.pose.position);

    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(x(6), Eigen::Vector3d::UnitZ()) * //  roll
        Eigen::AngleAxisd(x(7), Eigen::Vector3d::UnitY()) * //  pitch
        Eigen::AngleAxisd(x(8), Eigen::Vector3d::UnitX());  //  yaw
    
    tf::quaternionEigenToMsg(q, odom_ukf.pose.pose.orientation);
    tf::vectorEigenToMsg(vel, odom_ukf.twist.twist.linear);    
    tf::vectorEigenToMsg(wm, odom_ukf.twist.twist.angular);
     
    QuadrotorUKF::StateCov P = quadrotor_ukf.GetStateCovariance();
    for (int j = 0; j < 6; j++) {
      for (int i = 0; i < 6; i++) {
        odom_ukf.pose.covariance[i + j * 6] =
            P((i < 3) ? i : i + 3, (j < 3) ? j : j + 3);
      }
    }
    
    // Publish Msg
    pub_ukf_odom.publish(odom_ukf);
  }
}

void gps_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  // Get yaw 
  kr::quatd wQb;
  tf::quaternionMsgToEigen(msg->pose.pose.orientation, wQb);
  const kr::vec3d rpy = kr::getRPY(wQb.matrix());

  // Assemble measurement
  QuadrotorUKF::MeasGPSVec z;
  z(0) = msg->pose.pose.position.x;
  z(1) = msg->pose.pose.position.y;
  z(2) = msg->pose.pose.position.z;
  z(3) = msg->twist.twist.linear.x;
  z(4) = msg->twist.twist.linear.y;
  z(5) = msg->twist.twist.linear.z;
  z(6) = rpy[2];
  
  QuadrotorUKF::StateVec x = quadrotor_ukf.GetState();
  
  static ros::Publisher pPub = ros::NodeHandle("~").advertise<geometry_msgs::PoseStamped>("debug_pose",1);
  geometry_msgs::PoseStamped pPose;
  pPose.header.stamp = msg->header.stamp;
  pPose.header.frame_id = frame_id;
  pPose.pose = msg->pose.pose;
  pPub.publish(pPose);
  
  static ros::Publisher vxPub = ros::NodeHandle("~").advertise<geometry_msgs::Vector3>("vel_x",1);
  geometry_msgs::Vector3 vx;
  vx.x = x(3);
  vx.y = x(4);
  vx.z = x(5);
  vxPub.publish(vx);
  
  static ros::Publisher vzPub = ros::NodeHandle("~").advertise<geometry_msgs::Vector3>("vel_z",1);
  vzPub.publish(msg->twist.twist.linear);
  
  // Assemble measurement covariance
  QuadrotorUKF::MeasGPSCov RnGPS;
  RnGPS.setZero();
  
  RnGPS(0,0) = msg->pose.covariance[0 + 0 * 6];
  RnGPS(1,1) = msg->pose.covariance[1 + 1 * 6];
  RnGPS(2,2) = msg->pose.covariance[2 + 2 * 6];
  RnGPS(3,3) = msg->twist.covariance[0 + 0 * 6];
  RnGPS(4,4) = msg->twist.covariance[1 + 1 * 6];
  RnGPS(5,5) = msg->twist.covariance[2 + 2 * 6];
  RnGPS(6,6) = msg->pose.covariance[5 + 5 * 6];   //  element(5,5), rot_z
  
  // Measurement update
  quadrotor_ukf.MeasurementUpdateGPS(z, RnGPS, msg->header.stamp);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "quadrotor_ukf");
  ros::NodeHandle n("~");

  // UKF Parameters
  double alpha = 0.1;
  double beta = 2.0;
  double kappa = 0.0;

  // Noise standard devs
  double stdAcc[3] = {0, 0, 0};
  double stdW[3] = {0, 0, 0};
  double stdAccBias[3] = {0, 0, 0};

  n.param("world_frame_id", frame_id, std::string("imu"));
  n.param("body_frame_id", child_frame_id, std::string("world"));

  n.param("alpha", alpha, 0.1);
  n.param("beta", beta, 2.0);
  n.param("kappa", kappa, 0.0);

  n.param("noise_std/process/acc/x", stdAcc[0], 0.5);
  n.param("noise_std/process/acc/y", stdAcc[1], 0.5);
  n.param("noise_std/process/acc/z", stdAcc[2], 0.5);
  n.param("noise_std/process/w/x", stdW[0], 0.01);
  n.param("noise_std/process/w/y", stdW[1], 0.01);
  n.param("noise_std/process/w/z", stdW[2], 0.01);
  n.param("noise_std/process/acc_bias/x", stdAccBias[0], 1.0);
  n.param("noise_std/process/acc_bias/y", stdAccBias[1], 1.0);
  n.param("noise_std/process/acc_bias/z", stdAccBias[2], 1.0);

  // Fixed process noise
  QuadrotorUKF::ProcNoiseCov Rv;
  Rv.setIdentity();
  Rv(0, 0) = stdAcc[0] * stdAcc[0];
  Rv(1, 1) = stdAcc[1] * stdAcc[1];
  Rv(2, 2) = stdAcc[2] * stdAcc[2];
  Rv(3, 3) = stdW[0] * stdW[0];
  Rv(4, 4) = stdW[1] * stdW[1];
  Rv(5, 5) = stdW[2] * stdW[2];
  Rv(6, 6) = stdAccBias[0] * stdAccBias[0];
  Rv(7, 7) = stdAccBias[1] * stdAccBias[1];
  Rv(8, 8) = stdAccBias[2] * stdAccBias[2];

  // Initialize UKF
  quadrotor_ukf.SetParameters(alpha, beta, kappa);
  quadrotor_ukf.SetImuCovariance(Rv);

  ros::Subscriber subImu = n.subscribe("imu", 1, imu_callback);
  ros::Subscriber subGPS = n.subscribe("gps_odom", 1, gps_callback);
  
  pub_ukf_odom = n.advertise<nav_msgs::Odometry>("odometry", 1);

  ros::spin();

  return 0;
}
