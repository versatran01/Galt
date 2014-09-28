/*
 * mainwindow.h
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 11/7/2014
 *      Author: gareth
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "posecalibrator.h"
#include "posecalibrationview.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent=0);
  ~MainWindow();
   
protected:
  void closeEvent(QCloseEvent *event);
  
  void quit();
  
signals:
  
public slots:
  void calibratePoseAction(bool);
  void calibrateSpectrumAction(bool);
  void quitAction(bool);
  
private:
  Ui::MainWindow *ui;
  
  //  active widget
  QWidget * mainWidget_;
  
  enum Mode {
    CalibratePose=0,
    CalibrateSpectrum
  } mode_;  
  
  //  select calibration mode
  void setMode(Mode mode);
};

#endif // MAINWINDOW_H
