#ifndef SPECTRUMCALIBRATIONVIEW_H
#define SPECTRUMCALIBRATIONVIEW_H

#include <QWidget>

namespace Ui {
class SpectrumCalibrationView;
}

class SpectrumCalibrationView : public QWidget
{
  Q_OBJECT
  
public:
  explicit SpectrumCalibrationView(QWidget *parent = 0);
  ~SpectrumCalibrationView();
  
private:
  Ui::SpectrumCalibrationView *ui;
};

#endif // SPECTRUMCALIBRATIONVIEW_H
