#include "posecalibrationview.h"
#include "ui_posecalibrationview.h"

PoseCalibrationView::PoseCalibrationView(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::PoseCalibrationView)
{
  ui->setupUi(this);
}

PoseCalibrationView::~PoseCalibrationView()
{
  delete ui;
}
