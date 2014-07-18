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

#include <QException>
#include <iostream>

#include "posecalibrationview.h"
#include "ui_posecalibrationview.h"

#include <yaml-cpp/yaml.h>

PoseCalibrationView::PoseCalibrationView(QWidget *parent, const ros::NodeHandlePtr &nhp) :
  QWidget(parent),
  ui(new Ui::PoseCalibrationView), nhp_(nhp), poseCalib_(0)
{
  ui->setupUi(this);
   
  //  initialize a new pose estimator
  reset();
  
  //  configure UI objects
  QObject::connect(ui->calibrateButton, SIGNAL(clicked(bool)),
                   this, SLOT(calibrateButtonPressed(bool)));
  
  QObject::connect(ui->resetButton, SIGNAL(clicked(bool)), 
                   this, SLOT(resetButtonPressed(bool)));
  
  QObject::connect(ui->saveButton, SIGNAL(clicked(bool)),
                   this, SLOT(saveButtonPressed(bool)));
}

PoseCalibrationView::~PoseCalibrationView()
{
  delete ui;
}

void PoseCalibrationView::reset() {
  if (poseCalib_) {
    delete poseCalib_;
  }
  poseCalib_ = new PoseCalibrator(this, nhp_);
  
  QObject::connect(poseCalib_, SIGNAL(receivedMessage()),
                   this, SLOT(calibratorUpdatedState()));
  
  calibratorUpdatedState();
}

void PoseCalibrationView::calibratorUpdatedState(void) {
  const cv::Mat& image = poseCalib_->lastImage();
  if (!image.empty()) {
    ui->imageWidget->setImage( image );
  }
  
  ui->calibrateButton->setEnabled(poseCalib_->canCalibrate());
  ui->saveButton->setEnabled(poseCalib_->hasCalibration());
  
  auto pose = poseCalib_->getSpectrometerPose();
  
  auto linePoint = pose.getPosition();
  auto lineNormal = pose.getDirection();
  
  char buf[1000];
  sprintf(buf, "%lu", poseCalib_->observationCount());
  ui->numSamples->setText(QString(buf));
  
  sprintf(buf, "%.4f, %.4f, %.4f", 
      linePoint[0], 
      linePoint[1], 
      linePoint[2]);
  ui->lineOrigin->setText(QString(buf));

  sprintf(buf, "%.4f, %.4f, %.4f",
      lineNormal[0],
      lineNormal[1],
      lineNormal[2]);
  ui->lineNormal->setText(QString(buf));
  
  sprintf(buf, "%.4f", pose.getFov() * 180 / M_PI);
  ui->fieldOfView->setText(QString(buf));
  
  sprintf(buf, "%.5f",
      std::sqrt(pose.getSquaredError()));
  ui->error->setText(QString(buf));
}

void PoseCalibrationView::calibrateButtonPressed(bool) {
  if (poseCalib_->canCalibrate()) {
    poseCalib_->calibrate();
  }
}

void PoseCalibrationView::resetButtonPressed(bool) {
  reset();
}

void PoseCalibrationView::saveButtonPressed(bool) {
  if (poseCalib_ && poseCalib_->hasCalibration()) {
  
    YAML::Node node(poseCalib_->getSpectrometerPose());
    
    if (node.IsSequence()) {
      ROS_INFO("Sequence!\n");
    }
    
    YAML::Emitter emitter;
    emitter.SetSeqFormat(YAML::Flow);
    emitter << node;
    
    FILE* test = fopen("/home/gareth/test.file", "w");
    fprintf(test,"%s", emitter.c_str());
    fclose(test);
  }
}
