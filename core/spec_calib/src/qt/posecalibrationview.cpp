/*
 * posecalibrationview.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 11/7/2014
 *      Author: gareth
 */

#include "posecalibrationview.h"
#include "ui_posecalibrationview.h"
#include <QException>
#include <iostream>

PoseCalibrationView::PoseCalibrationView(QWidget *parent, const ros::NodeHandlePtr &nhp) :
  QWidget(parent),
  ui(new Ui::PoseCalibrationView), nhp_(nhp)
{
  ui->setupUi(this);
  
  poseCalib_ = new PoseCalibrator(this, nhp);
  
  ui->calibrateButton->setEnabled(false);
  
  QObject::connect(ui->calibrateButton, SIGNAL(clicked(bool)),
                   this, SLOT(calibrateButtonPressed(bool)));
  
  QObject::connect(poseCalib_, SIGNAL(receivedMessage()),
                   this, SLOT(calibratorUpdatedState()));
  
  //  trigger an initial update
  calibratorUpdatedState();
}

PoseCalibrationView::~PoseCalibrationView()
{
  delete ui;
}

void PoseCalibrationView::calibratorUpdatedState(void) {
  const cv::Mat& image = poseCalib_->lastImage();
  if (!image.empty()) {
    ui->imageWidget->setImage( image );
  }
  
  ui->calibrateButton->setEnabled(poseCalib_->canCalibrate());
  
  auto pose = poseCalib_->getSpectrometerPose();
  
  auto linePoint = pose.getPosition();
  auto lineNormal = pose.getDirection();
  
  char buf[1000];
  sprintf(buf, "%lu", poseCalib_->observationCount());
  ui->numSamples->setText(QString(buf));
  
  sprintf(buf, "%.3e, %.3e, %.3e", 
      linePoint[0], 
      linePoint[1], 
      linePoint[2]);
  ui->lineOrigin->setText(QString(buf));

  sprintf(buf, "%.3e, %.3e, %.3e",
      lineNormal[0],
      lineNormal[1],
      lineNormal[2]);
  ui->lineNormal->setText(QString(buf));
  
  sprintf(buf, "%.5e",
      std::sqrt(0.0));//pose.getSquaredError()));
  ui->rSquared->setText(QString(buf));
}

void PoseCalibrationView::calibrateButtonPressed(bool checked) {
  if (poseCalib_->canCalibrate()) {
    poseCalib_->calibrate();
  }
}

void PoseCalibrationView::resetButtonPressed(bool checked) {
 
  if (poseCalib_) {
    delete poseCalib_;
  }
  poseCalib_ = new PoseCalibrator(this, nhp_);
  
  QObject::connect(poseCalib_, SIGNAL(receivedMessage()),
                   this, SLOT(calibratorUpdatedState()));
  
  calibratorUpdatedState();
}

void PoseCalibrationView::saveButtonPressed(bool checked) {
  
}
