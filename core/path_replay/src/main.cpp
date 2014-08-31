#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

/**
 * This node reads a bunch of 'fixed' odometry messages (minus twist) from a CSV
 * file, and publishes them when the original message [read: same timestamp] 
 * arrives and triggers the callback. Just a quick hack to publish the corrected
 * data from the CSV file at the appropriate time in the bagfile, without having
 * to re-write the bagfile.
 */

ros::Publisher pubOdom;
int messagesMissed=0;
std::map<uint64_t, nav_msgs::Odometry> odoMap;

bool loadOdometry(const std::string& filePath) {
  
  FILE * stream = fopen(filePath.c_str(), "r");
  if (!stream) {
    ROS_ERROR("Failed to load %s", filePath.c_str());
    ROS_ERROR("Reason: %s", strerror(errno));
    return false;
  }
  
  uint32_t ts_sec, ts_nsec;
  float qw,qx,qy,qz,px,py,pz;
  
  char buffer[3000];
  int count=0;
  while (fgets(&buffer[0], sizeof(buffer), stream) != NULL) {
    //  read until error or we finish, very simple...
    const int sz = sscanf(buffer, "%u, %u, %f, %f, %f, %f, %f, %f, %f",
                          &ts_sec, &ts_nsec, &qw, &qx, &qy, &qz, &px, &py, &pz);
    if (sz != 9) {
      ROS_ERROR("Expected 9 CSV values, received %i", sz);
      return false;
    }
    
    //  create message
    nav_msgs::Odometry odo;
    odo.header.stamp = ros::Time(ts_sec,ts_nsec);
    odo.pose.pose.orientation.w = qw;
    odo.pose.pose.orientation.x = qx;
    odo.pose.pose.orientation.y = qy;
    odo.pose.pose.orientation.z = qz;
    odo.pose.pose.position.x = px;
    odo.pose.pose.position.y = py;
    odo.pose.pose.position.z = pz;
    
    //  key by nanoseconds, assuming the original timestamps have been
    //  exactly preserved through all processing...
    odoMap[ts_sec*1e9 + ts_nsec] = odo;
    count++;
  }
  ROS_INFO("Read %i messages from %s\n", count, filePath.c_str());
  
  return true;
}

void odomCallback(const nav_msgs::OdometryConstPtr& odo_msg) {
  
  const uint64_t key = odo_msg->header.stamp.sec*1e9 + 
      odo_msg->header.stamp.nsec;
  
  auto ite = odoMap.find(key);
  if (ite == odoMap.end()) {
    messagesMissed++;
  } else {
    ite->second.header.frame_id = odo_msg->header.frame_id;
    pubOdom.publish(ite->second);
  }
  
  if (messagesMissed % 50 == 0) {
    //  not synchronized properly...
    ROS_WARN("Warning, %i messages missed. Is this the right bag file?", 
             messagesMissed);
  }
}

int main(int argc, char ** argv) {
  ros::init(argc,argv,"path_replay");
  ros::NodeHandle nh("~");

  std::string csv_path;
  if (!nh.hasParam("csv_path")) {
    ROS_ERROR("The csv_path parameter must be specified!");
    return -1;
  }
  nh.getParam("csv_path", csv_path);
  
  //  load odometry from CSV file
  if (!loadOdometry(csv_path)) {
    return -1;
  }
  
  ros::Subscriber subOdom = nh.subscribe("odometry_in", 1, &odomCallback);
  pubOdom = nh.advertise<nav_msgs::Odometry>("odometry", 1);
  
  ros::spin();
  return 0;
}
