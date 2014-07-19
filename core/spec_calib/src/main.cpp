/*
 * main.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 11/7/2014
 *      Author: gareth
 */

#include <QApplication>
#include <QThread>
#include <ros/ros.h>

#include "mainwindow.h"

ros::NodeHandlePtr nh;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "spec_calib");
  nh = ros::NodeHandlePtr(new ros::NodeHandle("~"));
    
  //  check for camera serial parameter
  if (!nh->hasParam("camera_serial")) {
    ROS_ERROR("Error: You must supply the camera_serial paremeter");
    return -1;
  }
  
  QApplication a(argc, argv);
  MainWindow w;
  w.show();

  while ( ros::ok() ) {
    a.processEvents();
    ros::spinOnce();
    usleep(500);
  }

  return 0;
}
