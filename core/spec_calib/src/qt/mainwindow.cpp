#include "mainwindow.hpp"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  
  QStatusBar * bar = this->statusBar();
  bar->showMessage(tr("Hey there!"));
}

MainWindow::~MainWindow()
{
  delete ui;
}
