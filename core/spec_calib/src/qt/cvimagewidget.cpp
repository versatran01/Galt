#include "cvimagewidget.h"

#include <QPainter>
#include <QPainterPath>

CVImageWidget::CVImageWidget(QWidget *parent) :
  QWidget(parent)
{
  image_ = cv::Mat(240,320,CV_8UC3,cv::Scalar(255,0,0));
}

void CVImageWidget::setImage(const cv::Mat& image) {
  image_ = image;
}

const cv::Mat& CVImageWidget::getImage() const {
  return image_;
}

void CVImageWidget::paintEvent(QPaintEvent *)
{   
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  
  //  fill black background
  painter.fillRect(QRect(0,0,width(),height()), Qt::SolidPattern);
  
  if (!image_.empty()) {    
    int draw_width = image_.cols;
    int draw_height = image_.rows;
    
    //  scale to aspect fit
    if (draw_width > width()) {
      float ratio = width() / static_cast<float>(draw_width);
      draw_width = width();
      draw_height = std::floor(ratio * draw_height);
    }
    
    if (draw_height > height()) {
      float ratio = height() / static_cast<float>(draw_height);
      draw_height = height();
      draw_width = std::floor(ratio * draw_width);
    }
    
    //  centre
    QRect targetRect;
    targetRect.setX((width() - draw_width) / 2);
    targetRect.setY((height() - draw_height) / 2);
    targetRect.setWidth(draw_width);
    targetRect.setHeight(draw_height);
        
    const size_t step = image_.ptr(1) - image_.ptr(0);
    QImage::Format format;
    
    //  support greyscale and colour, but nothing else for now
    switch(image_.type()) {
      case CV_8UC1:
        format = QImage::Format_Mono;
        break;
      case CV_8UC3:
        format = QImage::Format_RGB888;
        break;
      default:
        //  fill with green and exit
        painter.setBrush(QColor(0,255,0));
        painter.fillRect(targetRect,Qt::SolidPattern);
        return;
    }

    QImage img(image_.ptr(),image_.cols,image_.rows,step,format);
    painter.drawImage(targetRect,img);
  }
}

void CVImageWidget::resizeEvent(QResizeEvent *)
{
}
