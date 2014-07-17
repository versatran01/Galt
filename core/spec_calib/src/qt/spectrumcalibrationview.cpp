#include "spectrumcalibrationview.h"
#include "ui_spectrumcalibrationview.h"

SpectrumCalibrationView::SpectrumCalibrationView(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::SpectrumCalibrationView)
{
  ui->setupUi(this);
}

SpectrumCalibrationView::~SpectrumCalibrationView()
{
  delete ui;
}
