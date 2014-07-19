/*
 * SpectrumCalibrationView.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 17/7/2014
 *      Author: gareth
 */

#include "spectrumcalibrationview.h"
#include "ui_spectrumcalibrationview.h"

#include <ros/package.h>
#include <qwt/qwt.h>
#include <qwt/qwt_plot.h>

SpectrumCalibrationView::SpectrumCalibrationView(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::SpectrumCalibrationView), specCalib_(0)
{
  ui->setupUi(this);
  reset();
  
  QwtPlot * plot = ui->plot;
  
  plot->setTitle("Spectrum");
  plot->setCanvasBackground(Qt::white);
  plot->setAxisScale(QwtPlot::yLeft, 0.0, 10.0);
  plot->resize(320,240);
  plot->repaint();
}

SpectrumCalibrationView::~SpectrumCalibrationView()
{
  delete ui;
}

void SpectrumCalibrationView::reset() {
  if (specCalib_) {
    delete specCalib_;
    specCalib_=0;
  }
  
  //  load the configuration file for the pose
  //  TODO: refactor this kind of functionality into a class...
  const std::string path = ros::package::getPath("spec_calib");
  
  ros::NodeHandle nh("~");
  std::string camSerial;
  //nh.param("camera_serial",camSerial,"this_is_an_error");
  
  const std::string filePath = path + "/config/pose_" + camSerial + ".yaml";
  
  //  load the pose information
  ///FILE * input = fopen(filePath.c_str(), "r");
  //fscanf(input,)
  
  galt::SpectrometerPose pose;
  specCalib_ = new SpectrumCalibrator(this,pose);
  QObject::connect(specCalib_,SIGNAL(receivedMessage()),this,SLOT(calibratorUpdateState()));
}


void SpectrumCalibrationView::calibratorUpdateState(void) {
  const cv::Mat& image = specCalib_->lastImage();
  if (!image.empty()) {
    ui->imageWidget->setImage(image);
  }
}
