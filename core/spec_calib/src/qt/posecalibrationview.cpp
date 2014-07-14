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

PoseCalibrationView::PoseCalibrationView(QWidget *parent, PoseCalibrator *poseCalib) :
  QWidget(parent),
  ui(new Ui::PoseCalibrationView), poseCalib_(poseCalib)
{
  ui->setupUi(this);
  
  if (!poseCalib) {
    throw std::invalid_argument("poseCalib cannot be null");
  }
  
  ui->calibrateButton->setEnabled(false);
  
  QObject::connect(ui->calibrateButton, SIGNAL(clicked(bool)),
                   this, SLOT(calibrateButtonPressed(bool)));
  
  QObject::connect(poseCalib, SIGNAL(receivedMessage()),
                   this, SLOT(calibratorUpdatedState()));
}

PoseCalibrationView::~PoseCalibrationView()
{
  delete ui;
}

void PoseCalibrationView::calibratorUpdatedState(void) {
  ui->imageWidget->setImage( poseCalib_->lastImage() );
  
  if (poseCalib_->canCalibrate()) {
    ui->calibrateButton->setEnabled(true);
  }
  
  auto linePoint = poseCalib_->getLineOrigin();
  auto lineNormal = poseCalib_->getLineNormal();
  
  char buf[1000];
  sprintf(buf, "%lu", poseCalib_->observationCount());
  ui->numSamples->setText(QString(buf));
  
  sprintf(buf, "%.3f, %.3f, %.3f", 
      linePoint[0], 
      linePoint[1], 
      linePoint[2]);
  ui->lineOrigin->setText(QString(buf));

  sprintf(buf, "%.3f, %.3f, %.3f",
      lineNormal[0],
      lineNormal[1],
      lineNormal[2]);
  ui->lineNormal->setText(QString(buf));
  
  sprintf(buf, "%.5f",
      poseCalib_->getError());
  ui->rSquared->setText(QString(buf));
}

void PoseCalibrationView::calibrateButtonPressed(bool checked) {
  if (poseCalib_->canCalibrate()) {
    poseCalib_->calibrate();
  }
}
