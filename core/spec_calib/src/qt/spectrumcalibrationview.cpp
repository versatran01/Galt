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
  }
  specCalib_ = new SpectrumCalibrator(this);
  
  QObject::connect(specCalib_,SIGNAL(receivedMessage()),this,SLOT(calibratorUpdateState()));
}


void SpectrumCalibrationView::calibratorUpdateState(void) {
  const cv::Mat& image = specCalib_->lastImage();
  if (!image.empty()) {
    ui->imageWidget->setImage(image);
  }
}
