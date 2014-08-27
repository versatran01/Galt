#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <signal.h>

FILE * sam_file=0, * gps_file=0;
int sam_count=0, gps_count=0;

void samCallback(const nav_msgs::OdometryConstPtr& odoMsg) {
  fprintf(sam_file, "%u, %u, %f, %f, %f, %f, %f, %f, %f\n",
          odoMsg->header.stamp.sec,
          odoMsg->header.stamp.nsec,
          odoMsg->pose.pose.orientation.w,
          odoMsg->pose.pose.orientation.x,
          odoMsg->pose.pose.orientation.y,
          odoMsg->pose.pose.orientation.z,
          odoMsg->pose.pose.position.x,
          odoMsg->pose.pose.position.y,
          odoMsg->pose.pose.position.z);
  sam_count++;
}

void gpsOdomCallback(const nav_msgs::OdometryConstPtr& odoMsg) {
  fprintf(gps_file, "%u, %u, %f, %f, %f, %f, %f, %f, %f\n",
          odoMsg->header.stamp.sec,
          odoMsg->header.stamp.nsec,
          odoMsg->pose.pose.orientation.w,
          odoMsg->pose.pose.orientation.x,
          odoMsg->pose.pose.orientation.y,
          odoMsg->pose.pose.orientation.z,
          odoMsg->pose.pose.position.x,
          odoMsg->pose.pose.position.y,
          odoMsg->pose.pose.position.z);
  gps_count++;
}

void sigint_handler(int) {
  fflush(gps_file);
  fflush(sam_file);
  fclose(gps_file);
  fclose(sam_file);
  ROS_INFO("Wrote %i SAM messages to disk", sam_count);
  ROS_INFO("Wrote %i GPS messages to disk", gps_count);
  ros::shutdown();
}

int main(int argc, char ** argv) {
  ros::init(argc,argv,"path_capture",
            ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");
  
  //  insall custom sigint handler
  signal(SIGINT, &sigint_handler);

  std::string output_path;
  nh.param("output_path", output_path, std::string("output"));
  
  const std::string sam_path = output_path + "_sam.csv";
  const std::string gps_path = output_path + "_gps.csv";
  
  sam_file = fopen(sam_path.c_str(), "w");
  if (!sam_file) {
    ROS_ERROR("Failed to open %s for writing", sam_path.c_str());
    ROS_ERROR("Reason: %s", strerror(errno));
    return -1;
  }
  gps_file = fopen(gps_path.c_str(), "w");
  if (!gps_file) {
    ROS_ERROR("Failed to open %s for writing", gps_path.c_str());
    ROS_ERROR("Reason: %s", strerror(errno));
    return -1;
  }
  printf("Writing data to %s and %s", sam_path.c_str(), gps_path.c_str());
  
  ros::Subscriber subSam = nh.subscribe("sam_odom", 10, &samCallback);
  ros::Subscriber subGps = nh.subscribe("gps_odom", 10, &gpsOdomCallback);
    
  ros::spin();
  return 0;
}
