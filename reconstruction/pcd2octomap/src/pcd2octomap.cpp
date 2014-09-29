/*
 * pcd2octomap.cpp
 *  _   _             _           _____         _
 * | \ | | ___  _   _| | ____ _  |_   _|__  ___| |__
 * |  \| |/ _ \| | | | |/ / _` |   | |/ _ \/ __| '_ \
 * | |\  | (_) | |_| |   < (_| |   | |  __/ (__| | | |
 * |_| \_|\___/ \__,_|_|\_\__,_|   |_|\___|\___|_| |_|
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of pcd2octomap.
 *
 *	Created on: 29/09/2014
 */

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeIterator.hxx>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

int main(int argc, char ** argv) {
  ros::init(argc,argv,"pcd2octomap");
  ros::NodeHandle nh("~");
  
  std::string inputPath,outputPathOT,outputPathCSV,worldFrameId;
  double resolution, maxRange;
  double probHit, probMiss;
  
  nh.param("input_path", inputPath, std::string("input.pcd"));
  nh.param("output_path_ot", outputPathOT, std::string("output.ot"));
  nh.param("output_path_csv", outputPathCSV, std::string("output.csv"));
  nh.param("resolution", resolution, 0.05);
  nh.param("max_laser_range", maxRange, 20.0);
  nh.param("prob_hit", probHit, 0.7);
  nh.param("prob_miss", probMiss, 0.3);
  nh.param("world_frame_id", worldFrameId, std::string("world"));
  
  ROS_INFO("Will process %s into %s with resolution %f", inputPath.c_str(),
           outputPathOT.c_str(), resolution);
  ROS_INFO("Prob hit/miss: %f/%f", probHit, probMiss);
  
  pcl::PointCloud<pcl::PointWithViewpoint>::Ptr cloud;
  cloud.reset(new pcl::PointCloud<pcl::PointWithViewpoint>());
  
  //  load input file as points w/ viewpoints attached
  if (pcl::io::loadPCDFile<pcl::PointWithViewpoint>(inputPath, *cloud) == -1) {
    ROS_INFO("Failed to load file: %s\n", inputPath.c_str());
    return -1;
  }
  ROS_INFO_STREAM("Loaded " << cloud->width*cloud->height 
            << " points.");
  
  //  insert all points into an ocotomap
  octomap::OcTree tree(resolution);
  tree.setProbHit(probHit);
  tree.setProbMiss(probMiss);
  tree.setOccupancyThres(0.5);  //  some arbitrary parameters
  tree.setClampingThresMin(0.01);
  tree.setClampingThresMax(0.99);
  for (size_t i=0; i < cloud->points.size(); i++) {
    octomap::point3d end(cloud->points[i].x, cloud->points[i].y,
                        cloud->points[i].z);
    octomap::point3d origin(cloud->points[i].vp_x,cloud->points[i].vp_y,
                            cloud->points[i].vp_z);
    tree.insertRay(origin,end,maxRange);
  }
  ROS_INFO("Assembled octomap.");
 
  if (!tree.write(outputPathOT)) {
    ROS_INFO("Failed to write to %s.", outputPathOT.c_str());
    return -1;
  }
  ROS_INFO("Wrote to %s.", outputPathOT.c_str());
  
  //  build CSV version of the octomap
  std::stringstream ss;
  
  //  iterate through the tree
  for (octomap::OcTree::leaf_iterator it = tree.begin_leafs();
       it != tree.end_leafs(); it++) {
    
    const double depth = it.getDepth();
    const double log_odds = it->getLogOdds();
    const double size = it.getSize();
    const octomap::point3d position = it.getCoordinate();
    
    ss << depth << "," << log_odds << "," << size << ","
       << position.x() << "," << position.y() << "," << position.z() << "\n";
  }
  const std::string stringRep = ss.str();
  ROS_INFO("Assembled CSV representation. Size: %lu", stringRep.size());
  
  std::ofstream outputStream;
  outputStream.open(outputPathCSV.c_str());
  if (!outputStream.is_open()) {
    ROS_INFO("Failed to write to %s.", outputPathCSV.c_str());
    return -1;
  }
  outputStream << stringRep;
  outputStream.close();
  ROS_INFO("Wrote to %s.", outputPathCSV.c_str());
  
  //  publish the octomap for visualization
  ROS_INFO("Publishing octomap on latched topic: %s", 
           (ros::this_node::getName() + "/map").c_str());
  ros::Publisher pubOctomap = nh.advertise<octomap_msgs::Octomap>("map",1,true);
  octomap_msgs::Octomap msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = worldFrameId;
  octomap_msgs::fullMapToMsg(tree,msg);
  pubOctomap.publish(msg);
  
  ros::spin();
  return 0;
}
