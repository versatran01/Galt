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
#include <QDir>
#include <ros/ros.h>
#include <ros/package.h>

#include "mainwindow.h"

ros::NodeHandlePtr nh;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "spec_calib");
  nh = ros::NodeHandlePtr(new ros::NodeHandle("~"));
    
  //  check for camera serial parameter
  if (!nh->hasParam("camera_serial")) {
    ROS_ERROR("You must supply the camera_serial parameter");
    return -1;
  }
  
  if (!nh->hasParam("session_name")) {
    ROS_ERROR("You must supply a session_name parameter");
    return -1;
  }
  std::string sessionName;
  nh->getParam("session_name", sessionName);
  
  //  generate the session path
  std::string galtSetup = ros::package::getPath("galt_setup");
  if (galtSetup.empty()) {
    ROS_ERROR("Could not find path to galt_setup. Is your path correct?");
    return -1;
  }
  
  const std::string sessionPath = galtSetup + "/spectral/" + sessionName;
  nh->setParam("session_path", sessionPath);
  
  if (QDir(sessionPath.c_str()).exists()) {
    // warn because we might erase past calibrations...
    ROS_WARN("Working in session folder: %s", sessionPath.c_str());
  } else {
    ROS_INFO("Creating session folder: %s", sessionPath.c_str());
    //  mkpath will make all intermediate folders also, if necessary
    if (!QDir().mkpath(sessionPath.c_str())) {
      ROS_ERROR("Failed to create directory - check your permissions");
      return -1;
    }
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
