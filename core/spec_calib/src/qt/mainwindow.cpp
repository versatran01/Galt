#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "posecalibrationview.h"
#include "cvimagewidget.h"

MainWindow::MainWindow(QWidget *parent, const ros::NodeHandlePtr& nhp) : QMainWindow(parent),
  ui(new Ui::MainWindow), mode_(None), nodeHandle_(nhp)
{
  ui->setupUi(this);
  
  setMode(CalibratePose);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::setMode(Mode mode) {
  if (mode != mode_) {
    
    if (mode_ == CalibratePose) {
      delete poseView_;
      poseView_=0;
    }
    
    mode_ = mode;
    if (mode_ == CalibratePose) {
      poseCalib_ = new PoseCalibrator(0, nodeHandle_);
      poseView_ = new PoseCalibrationView(this, poseCalib_);
      
      ui->horizontalLayout->addWidget(poseView_);
      this->statusBar()->showMessage("Calibrating pose");
    }
    else if (mode_ == CalibrateSpectrum) {
      //  add other view here
    }
  }
}
