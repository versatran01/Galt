/*
 * cvimagewidget.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 11/7/2014
 *      Author: gareth
 */

#include "cvimagewidget.h"

#include <QPainter>
#include <QPainterPath>
#include <QThread>
#include <stdexcept>

CVImageWidget::CVImageWidget(QWidget *parent) :
  QWidget(parent), image_() {
}

CVImageWidget::~CVImageWidget() {
}

void CVImageWidget::setImage(const cv::Mat& image) {
  if (image.type() != CV_8UC3) {
    throw std::invalid_argument("Image format must be RBG8");
  }
  temp_ = image; 
  
  QImage::Format format = QImage::Format_RGB888;
  const size_t step = temp_.ptr(1) - temp_.ptr(0);
  image_ = QImage(temp_.ptr(),temp_.cols,temp_.rows,step,format);
  //image_ = image_.rgbSwapped();
  
  repaint();
}

void CVImageWidget::paintEvent(QPaintEvent *)
{   
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  
  //  fill blank background
  painter.fillRect(QRect(0,0,width(),height()),QColor(229,233,238));
  
  //  draw border
  QPainterPath path;
  path.addRect(QRect(0,0,width(),height()));
  QPen pen;
  pen.setWidth(1);
  pen.setColor(QColor(0,0,0));
  pen.setStyle(Qt::SolidLine);
  
  painter.setPen(pen);
  painter.drawPath(path);
  
  if (!image_.isNull()) {    
    int draw_width = image_.width();
    int draw_height = image_.height();
    
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
    painter.drawImage(targetRect,image_);
  }
}

void CVImageWidget::resizeEvent(QResizeEvent *)
{
}
