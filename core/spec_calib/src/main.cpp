/*
 * main.cpp
 *
 *  Copyright (c) 2014 Kumar Robotics. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 11/7/2014
 *      Author: gareth
 */

#include <QApplication>
#include <ros/ros.h>

#include "mainwindow.hpp"

ros::NodeHandlePtr nh;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "spec_calib");
  nh = ros::NodeHandlePtr(new ros::NodeHandle("~"));
  
  QApplication a(argc, argv);
  MainWindow w;
  w.show();

  while ( ros::ok() ) {
    a.processEvents();
    ros::spinOnce();
  }

  return 0;
}
