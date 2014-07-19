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

#include <QDateTime>
#include <QTime>

#include <iostream>
#include <ros/package.h>

#include "posecalibrationview.h"
#include "ui_posecalibrationview.h"

#include <yaml-cpp/yaml.h>

PoseCalibrationView::PoseCalibrationView(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::PoseCalibrationView), poseCalib_(0)
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
  poseCalib_ = new PoseCalibrator(this);
  
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
  calibratorUpdatedState();
}

void PoseCalibrationView::resetButtonPressed(bool) {
  reset();
}

void PoseCalibrationView::saveButtonPressed(bool) {
  if (poseCalib_ && poseCalib_->hasCalibration()) {
  
    //  serial for the camera we are calibrating
    ros::NodeHandle nh("~");
    std::string camSerial;
    nh.param("camera_serial", camSerial, std::string("this_is_an_error"));
    
    //  path to the package
    const std::string path = ros::package::getPath("spec_calib");
    
    //  pretty-print the date        
    QDateTime dt = QDateTime::currentDateTime();
    QString str = dt.toString(Qt::ISODate);
    const std::string formattedDate(str.toAscii());
    
    //  nice formatted YAML output
    YAML::Emitter emitter;
    emitter.SetSeqFormat(YAML::Flow);
    emitter << YAML::Comment("Spectrometer pose configuration") << YAML::Newline;
    emitter << YAML::Comment("Camera: " + camSerial) << YAML::Newline;
    emitter << YAML::Comment("Generated on: " + formattedDate) << YAML::Newline;
    emitter << poseCalib_->getSpectrometerPose();
    
    const std::string filePath = path + "/config/pose_" + camSerial + ".yaml";
    
    FILE* output = fopen(filePath.c_str(), "w");
    if (!output) {
      //  TODO: add an error dialog here...
      ROS_ERROR("Failed to open %s. Reason: %s", filePath.c_str(), strerror(errno));
      return;
    }
    
    //  write to file
    fprintf(output,"%s", emitter.c_str());
    fclose(output);
  }
}
