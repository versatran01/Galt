/*
 * SpectrumCalibrationView.h
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 17/7/2014
 *      Author: gareth
 */

#ifndef SPECTRUMCALIBRATIONVIEW_H
#define SPECTRUMCALIBRATIONVIEW_H

#include <QWidget>
#include <qwt/qwt.h>
#include <qwt/qwt_plot.h> 
#include <qwt/qwt_plot_curve.h>

#include "spectrumcalibrator.h"

namespace Ui {
class SpectrumCalibrationView;
}

class SpectrumCalibrationView : public QWidget
{
  Q_OBJECT
  
public:
  explicit SpectrumCalibrationView(QWidget *parent = 0);
  ~SpectrumCalibrationView();
  
  void reset();
  
signals:
  
public slots:
  void calibratorUpdateState(void);
  
private:
  Ui::SpectrumCalibrationView *ui;
  SpectrumCalibrator * specCalib_;
  
  QwtPlotCurve * curve_;
};

#endif // SPECTRUMCALIBRATIONVIEW_H
