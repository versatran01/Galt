#include <ros/ros.h>

#include <imu_3dm_gx4/imu_3dm_gx4.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_3dm_gx4");
  imu_3dm_gx4::Imu imu(ros::NodeHandle(), ros::Nodehandle("~"));
  
  
  return 0;
}

