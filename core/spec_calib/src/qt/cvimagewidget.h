/*
 * cvimagewidget.h
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 11/7/2014
 *      Author: gareth
 */

#ifndef CVIMAGEWIDGET_H
#define CVIMAGEWIDGET_H

#include <QWidget>
#include <opencv2/opencv.hpp>

//  TODO: refactor into galt_widgets

class CVImageWidget : public QWidget
{
  Q_OBJECT
public:
  explicit CVImageWidget(QWidget *parent = 0);
  virtual ~CVImageWidget();
  
  void setImage(const cv::Mat& image);
  
protected:
  void paintEvent(QPaintEvent *);
  
private:
  cv::Mat temp_;
  QImage image_;
  
signals:
  
public slots:
  
};

#endif // CVIMAGEWIDGET_H
