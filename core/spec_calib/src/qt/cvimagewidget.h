#ifndef CVIMAGEWIDGET_H
#define CVIMAGEWIDGET_H

#include <QWidget>
#include <opencv2/opencv.hpp>

class CVImageWidget : public QWidget
{
  Q_OBJECT
public:
  explicit CVImageWidget(QWidget *parent = 0);
  
  void setImage(const cv::Mat& image);
  
  const cv::Mat& getImage() const;
  
protected:
  void paintEvent(QPaintEvent *);
  void resizeEvent(QResizeEvent *);
  
private:
  cv::Mat image_;
  
signals:
  
public slots:
  
};

#endif // CVIMAGEWIDGET_H
