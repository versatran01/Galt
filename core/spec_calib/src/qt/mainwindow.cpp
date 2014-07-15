/*
 * mainwindow.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 11/7/2014
 *      Author: gareth
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "posecalibrationview.h"
#include "cvimagewidget.h"

MainWindow::MainWindow(QWidget *parent, const ros::NodeHandlePtr& nhp) : QMainWindow(parent),
  ui(new Ui::MainWindow), mode_(None), nodeHandle_(nhp)
{
  ui->setupUi(this);
  
  setMode(CalibratePose);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::setMode(Mode mode) {
  if (mode != mode_) {
    
    if (mode_ == CalibratePose) {
      delete poseView_;
      poseView_=0;
    }
    
    mode_ = mode;
    if (mode_ == CalibratePose) {
      poseView_ = new PoseCalibrationView(this, nodeHandle_);
      
      ui->horizontalLayout->addWidget(poseView_);
      this->statusBar()->showMessage("Pose calibration mode");
    }
    else if (mode_ == CalibrateSpectrum) {
      //  add other view here
    }
  }
}
