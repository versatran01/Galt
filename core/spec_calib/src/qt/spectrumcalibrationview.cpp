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

template <typename T>
T loadYAMLNode(const std::string& filePath) {
  
  QFile f(filePath.c_str());
  if (!f.open(QFile::ReadOnly | QFile::Text)) {
    throw std::runtime_error("Unable to load file: " + filePath);
  }
  else if (f.size() > 2*1024*1024) {
    throw std::runtime_error("File exceeds 2MB limit");
  }
  
  QTextStream in(&f);
  QString yamlContent = in.readAll();
  
  std::string yaml = yamlContent.toUtf8().constData();
  YAML::Node node = YAML::Load(yaml);

  return node.as<T>();
}

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
  
  filterCurve_ = new QwtPlotCurve();
  filterCurve_->setTitle("filter_profile");
  filterCurve_->setPen( QPen(Qt::red, 2) );
  filterCurve_->setRenderHint( QwtPlotItem::RenderAntialiased, true);
  filterCurve_->setSymbol( new QwtSymbol() );
  
  filterCurve_->attach( plot );
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
  
  //  load the pose information
  //  TODO: clean this up and improve error handling
  galt::SpectrometerPose pose;
  try {
    pose = loadYAMLNode<galt::SpectrometerPose>(filePath);
  } catch(std::exception& e) {
    ROS_ERROR("Failed to load pose: %s", e.what());
    return;
  }
  
  //  load the filter profile  
  std::string filterPath;
  if (!nh.getParam("filter_path", filterPath)) {
    ROS_ERROR("Filter path not specified");
    return;
  }
  
  galt::FilterProfile filterProfile;
  try {
    filterProfile = loadYAMLNode<galt::FilterProfile>(filterPath);
  } catch (std::exception& e) {
    ROS_ERROR("Failed to load filter: %s", e.what());
    return;
  }
  
  ROS_INFO("Loaded filter: %s", filterProfile.getName().c_str());
  
  specCalib_ = new SpectrumCalibrator(this,pose,filterProfile);
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
  for (size_t i=0; i < spec.size(); i++) {
    double wavelen = spec.getWavelengths()[i];
    double intensity = spec.getIntensities()[i];
    points.push_back(QPointF(wavelen,intensity));    
  }
  
  curve_->setSamples( points );
  
  //  update filter plot
  points.clear();
  const galt::Spectrum& profile = specCalib_->getFilterProfile().getSpectrum();
  for (size_t i=0; i < profile.size(); i++) {
    double wavelen = profile.getWavelengths()[i];
    double intensity = profile.getIntensities()[i] * maxIntensity;
    points.push_back(QPointF(wavelen,intensity));
  }
  filterCurve_->setSamples(points);
  
  plot->replot();
  
  ui->imageWidget->setImage(image);
}
