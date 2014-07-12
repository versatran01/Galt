#ifndef POSECALIBRATIONVIEW_H
#define POSECALIBRATIONVIEW_H

#include <QWidget>

namespace Ui {
class PoseCalibrationView;
}

class PoseCalibrationView : public QWidget
{
  Q_OBJECT
  
public:
  explicit PoseCalibrationView(QWidget *parent = 0);
  ~PoseCalibrationView();
  
private:
  Ui::PoseCalibrationView *ui;
};

#endif // POSECALIBRATIONVIEW_H
