/*
 * SpectrumCalibrationView.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 17/7/2014
 *      Author: gareth
 */

#include <QFile>

#include "spectrumcalibrationview.h"
#include "ui_spectrumcalibrationview.h"

#include <ros/package.h>
#include <qwt/qwt.h>
#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_grid.h>
#include <qwt/qwt_legend.h>
#include <qwt/qwt_symbol.h>

#include <yaml-cpp/yaml.h>

SpectrumCalibrationView::SpectrumCalibrationView(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::SpectrumCalibrationView), specCalib_(0), currentMax_(0.0)
{
  ui->setupUi(this);
  reset();
  
  //  configure the plot
  QwtPlot * plot = ui->plot;
  plot->setTitle("Spectrum");
  plot->setCanvasBackground(Qt::white);
  plot->setAxisScale(QwtPlot::yLeft, 0.0, 1.0);
  plot->setAxisScale(QwtPlot::xBottom, 0.0, 1.0);
  plot->resize(320,240);
  plot->repaint();
    
  QwtPlotGrid *grid = new QwtPlotGrid();
  grid->attach( plot );
  
  curve_ = new QwtPlotCurve();
  curve_->setTitle("ocean_optics");
  curve_->setPen( QPen(Qt::blue, 2) );
  curve_->setRenderHint( QwtPlotItem::RenderAntialiased, true );
  curve_->setSymbol( new QwtSymbol() );
      
  curve_->attach( plot );      
}

SpectrumCalibrationView::~SpectrumCalibrationView()
{
  delete ui;
}

void SpectrumCalibrationView::reset() {
  if (specCalib_) {
    delete specCalib_;
    specCalib_=0;
  }
  
  //  load the configuration file for the pose
  //  TODO: refactor this kind of functionality into a class...
  const std::string path = ros::package::getPath("spec_calib");
  
  //  load the camera pose info
  ros::NodeHandle nh("~");
  std::string camSerial;
  int camSerialI;
  if (nh.getParam("camera_serial", camSerialI)) {
    //  may be an integer
    camSerial = std::to_string(camSerialI);
  } else if (!nh.getParam("camera_serial", camSerial)) {
    camSerial = "this_is_an_error";
  }
  
  const std::string filePath = path + "/config/pose_" + camSerial + ".yaml";
  
  //  TODO: refactor this into a utility function...
  //  load the pose information
  QFile f(tr(filePath.c_str()));
  if (!f.open(QFile::ReadOnly | QFile::Text)) {
    ROS_ERROR("Unable to load camera pose file: %s", filePath.c_str());
    return;
  }
  else if (f.size() > 2*1024*1024) {
    ROS_ERROR("File %s exceeds 2MB limit", filePath.c_str());
    return;
  }
  
  QTextStream in(&f);
  QString yamlContent = in.readAll();
  
  std::string yaml = yamlContent.toUtf8().constData();
  YAML::Node node = YAML::Load(yaml);
  galt::SpectrometerPose pose;
  try {
    pose = node.as<galt::SpectrometerPose>();
    auto dir = pose.getDirection();
    ROS_INFO("Loaded pose: %f, %f, %f", dir[0], dir[1], dir[2]);
  } catch(YAML::Exception& e) {
    ROS_ERROR("Failed to decode yaml: %s", e.what());
    return;
  }
  
  specCalib_ = new SpectrumCalibrator(this,pose);
  QObject::connect(specCalib_,SIGNAL(receivedMessage()),this,SLOT(calibratorUpdateState()));
}

void SpectrumCalibrationView::calibratorUpdateState(void) {
  const cv::Mat& image = specCalib_->lastImage(); //  RGB image
  
  //  get range of spectrum
  galt::Spectrum spec = specCalib_->lastSpectrum();
  
  //  figure out size of plot axes
  double maxIntensity=0.0;
  for (double I : spec.getIntensities()) {
    maxIntensity = std::max(I, maxIntensity);
  }
  
  const double minWavelength = spec.getWavelengths().front();
  const double maxWavelength = spec.getWavelengths().back();
   
  //  update spectrum plot, just replot every time for now
  QwtPlot * plot = ui->plot;
  
  if (maxIntensity > currentMax_) {
    plot->setAxisScale(QwtPlot::yLeft, 0.0, maxIntensity);
    currentMax_ = maxIntensity;
  }
  plot->setAxisScale(QwtPlot::xBottom, minWavelength, maxWavelength);
  
  QPolygonF points;
  for (size_t i=0; i < spec.getWavelengths().size(); i++) {
    double wavelen = spec.getWavelengths()[i];
    double intensity = spec.getIntensities()[i];
    points.push_back(QPointF(wavelen,intensity));    
  }
  
  curve_->setSamples( points );
  plot->replot();
  
  ui->imageWidget->setImage(image);
}
