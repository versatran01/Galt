#include "spectrumcalibrationview.h"
#include "ui_spectrumcalibrationview.h"

#include <qwt/qwt.h>
#include <qwt/qwt_plot.h>

SpectrumCalibrationView::SpectrumCalibrationView(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::SpectrumCalibrationView)
{
  ui->setupUi(this);
  
  QwtPlot * plot = new QwtPlot(0);
}

SpectrumCalibrationView::~SpectrumCalibrationView()
{
  delete ui;
}
