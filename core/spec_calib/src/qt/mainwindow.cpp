#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "posecalibrationview.h"
#include "cvimagewidget.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  
  QStatusBar * bar = this->statusBar();
  bar->showMessage(tr("Hey there!"));
  
  CVImageWidget * view = new CVImageWidget(this);
  //PoseCalibrationView * view = new PoseCalibrationView(this);
  ui->horizontalLayout->addWidget(view);  
}

MainWindow::~MainWindow()
{
  delete ui;
}
