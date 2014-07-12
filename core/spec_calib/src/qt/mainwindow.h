#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <ros/ros.h>

#include "posecalibrationview.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent, const ros::NodeHandlePtr& nhp);
  ~MainWindow();
    
private:
  Ui::MainWindow *ui;
  
  enum Mode {
    None=0,
    CalibratePose,
    CalibrateSpectrum,
  } mode_;
  PoseCalibrationView * poseView_;
  
  ros::NodeHandlePtr nodeHandle_;
  
  //  set the current application mode
  void setMode(Mode mode);
};

#endif // MAINWINDOW_H
