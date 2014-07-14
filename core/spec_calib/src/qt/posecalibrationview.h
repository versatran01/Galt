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
  explicit PoseCalibrationView(QWidget *parent, PoseCalibrator * poseCalib);
  ~PoseCalibrationView();
  
signals:
  
public slots:
  void calibratorUpdatedState(void);
  
  void calibrateButtonPressed(bool);
  
private:
  Ui::PoseCalibrationView *ui;
  PoseCalibrator * poseCalib_;
};

#endif // POSECALIBRATIONVIEW_H
