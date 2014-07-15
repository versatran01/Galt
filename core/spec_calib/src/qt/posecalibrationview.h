/*
 * posecalibrationview.h
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 11/7/2014
 *      Author: gareth
 */

#ifndef POSECALIBRATIONVIEW_H
#define POSECALIBRATIONVIEW_H

#include <QWidget>
#include "posecalibrator.h"

namespace Ui {
class PoseCalibrationView;
}

class PoseCalibrationView : public QWidget
{
  Q_OBJECT
  
public:
  explicit PoseCalibrationView(QWidget *parent, const ros::NodeHandlePtr& nhp);
  ~PoseCalibrationView();
  
signals:
  
public slots:
  void calibratorUpdatedState(void);
  
  void calibrateButtonPressed(bool);
  
  void resetButtonPressed(bool);
  
  void saveButtonPressed(bool);
  
private:
  Ui::PoseCalibrationView *ui;
  ros::NodeHandlePtr nhp_;
  PoseCalibrator * poseCalib_;
};

#endif // POSECALIBRATIONVIEW_H
