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
  
private slots:
  void calibratorUpdateState(void);
  
  void calibratorReceivedSpectrum(void);
  
  void spinBoxValueChanged(double);
  
  void sampleSourceButtonPressed(bool);
  
  void addObservationButtonPressed(bool);
  
  void calibrateButtonPressed(bool);
  
  void resetButtonPressed(bool);
  
  void saveButtonPressed(bool);
  
private:
  Ui::SpectrumCalibrationView *ui;
  SpectrumCalibrator * specCalib_;
  double currentMax_;
  
  QwtPlotCurve * camCurve_;
  QwtPlotCurve * camCalibCurve_;
};

#endif // SPECTRUMCALIBRATIONVIEW_H
