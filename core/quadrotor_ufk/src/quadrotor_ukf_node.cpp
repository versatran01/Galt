#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include "quadrotor_ukf.h"

#include <kr_math/SO3.hpp>

ros::Publisher pub_ukf_odom;
std::string frame_id;
std::string child_frame_id;

QuadrotorUKF quadrotor_ukf;

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
  
  static int calLimit = 100;
  static int calCnt = 0;
  static Eigen::Vector3d ag = Eigen::Vector3d::Zero();

  //  TODO: correct the underlying node so everything is in units of m/s^2
  const double accX = msg->linear_acceleration.x * QuadrotorUKF::kOneG;
  const double accY = msg->linear_acceleration.y * QuadrotorUKF::kOneG;
  const double accZ = msg->linear_acceleration.z * QuadrotorUKF::kOneG;
    
  // Assemble control input
  QuadrotorUKF::InputVec u;
  u(0) = accX;
  u(1) = accY;
  u(2) = accZ;
  u(3) = msg->angular_velocity.x;
  u(4) = msg->angular_velocity.y;
  u(5) = msg->angular_velocity.z;

  /*if (calCnt < calLimit)  // Calibration
  {
    calCnt++;
    ag += u.segment<3>(0);
  } else if (calCnt == calLimit)  // Save gravity vector
  {
    calCnt++;
    ag /= calLimit;
    double g = ag.norm();
    quadrotor_ukf.SetGravity(g);
  } else */
  
  //static ros::Publisher pubVelOdom = ros::NodeHandle("~").advertise<geometry_msgs::Vector3>("velvec",1);
  
  if (quadrotor_ukf.ProcessUpdate(u, msg->header.stamp))  // Process Update
  {
    nav_msgs::Odometry odom_ukf;
    odom_ukf.header.stamp = quadrotor_ukf.GetStateTime(); //  copies from header
    odom_ukf.header.frame_id = frame_id;
    odom_ukf.child_frame_id = child_frame_id;
    QuadrotorUKF::StateVec x = quadrotor_ukf.GetState();
    odom_ukf.pose.pose.position.x = x(0);
    odom_ukf.pose.pose.position.y = x(1);
    odom_ukf.pose.pose.position.z = x(2);
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(x(6), Eigen::Vector3d::UnitZ()) * //  roll
        Eigen::AngleAxisd(x(7), Eigen::Vector3d::UnitY()) * //  pitch
        Eigen::AngleAxisd(x(8), Eigen::Vector3d::UnitX());  //  yaw
    odom_ukf.pose.pose.orientation.x = q.x();
    odom_ukf.pose.pose.orientation.y = q.y();
    odom_ukf.pose.pose.orientation.z = q.z();
    odom_ukf.pose.pose.orientation.w = q.w();
    odom_ukf.twist.twist.linear.x = x(3);
    odom_ukf.twist.twist.linear.y = x(4);
    odom_ukf.twist.twist.linear.z = x(5);
    odom_ukf.twist.twist.angular.x = u(3);
    odom_ukf.twist.twist.angular.y = u(4);
    odom_ukf.twist.twist.angular.z = u(5);
    QuadrotorUKF::StateCov P = quadrotor_ukf.GetStateCovariance();
    for (int j = 0; j < 6; j++)
      for (int i = 0; i < 6; i++)
        odom_ukf.pose.covariance[i + j * 6] =
            P((i < 3) ? i : i + 3, (j < 3) ? j : j + 3);
    // Publish Msg
    pub_ukf_odom.publish(odom_ukf);
  }
}

void gps_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  // Get yaw 
  kr::quatd wQb = kr::quatd(msg->pose.pose.orientation.w,
                            msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z);
  
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
  z(7) = rpy[1];
  z(8) = rpy[0];
  
  QuadrotorUKF::StateVec x = quadrotor_ukf.GetState();
  //ROS_INFO("X: %f, %f, %f", x(3),x(4),x(5));
  //ROS_INFO("Z: %f, %f, %f", z(3),z(4),z(5));
  auto seg = x.segment<3>(6);
  ROS_INFO("%f, %f, %f, %f, %f, %f", seg(0), seg(1), seg(2), rpy[2], rpy[1], rpy[0]);
  
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
  RnGPS(7,7) = msg->pose.covariance[4 + 4 * 6];   //  (4,4)
  RnGPS(8,8) = msg->pose.covariance[3 + 3 * 6];   //  (3,3)
    
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

  n.param("world_frame_id", frame_id, std::string("/world"));
  n.param("body_frame_id", child_frame_id, std::string("/body"));

  n.param("alpha", alpha, 0.1);
  n.param("beta", beta, 2.0);
  n.param("kappa", kappa, 0.0);

  n.param("noise_std/process/acc/x", stdAcc[0], 0.1);
  n.param("noise_std/process/acc/y", stdAcc[1], 0.1);
  n.param("noise_std/process/acc/z", stdAcc[2], 0.1);
  n.param("noise_std/process/w/x", stdW[0], 0.1);
  n.param("noise_std/process/w/y", stdW[1], 0.1);
  n.param("noise_std/process/w/z", stdW[2], 0.1);
  n.param("noise_std/process/acc_bias/x", stdAccBias[0], 0.0005);
  n.param("noise_std/process/acc_bias/y", stdAccBias[1], 0.0005);
  n.param("noise_std/process/acc_bias/z", stdAccBias[2], 0.0005);

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
