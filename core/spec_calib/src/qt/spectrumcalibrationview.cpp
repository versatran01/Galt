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
#include <QPushButton>
#include <QDoubleSpinBox>

#include "spectrumcalibrationview.h"
#include "ui_spectrumcalibrationview.h"
#include "spectralplot.h"

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
  SpectralPlot * plot = ui->spectrumPlot;
  plot->setTitle("Spectrum");
  plot->setCanvasBackground(Qt::white);
  plot->setAxisScale(QwtPlot::yLeft, 0.0, 1.0);
  plot->setAxisScale(QwtPlot::xBottom, 0.0, 1.0);
  plot->resize(320,240);
  plot->repaint();
    
  QwtPlotGrid *grid = new QwtPlotGrid();
  grid->attach( plot );
  
  /*curve_ = new QwtPlotCurve();
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
  
  predictedCurve_ = new QwtPlotCurve();
  predictedCurve_->setTitle("predicted_spectrum");
  predictedCurve_->setPen( QPen(Qt::green, 2) );
  predictedCurve_->setRenderHint( QwtPlotItem::RenderAntialiased, true);
  predictedCurve_->setSymbol( new QwtSymbol() );
  
  predictedCurve_->attach( plot );*/
  
  //  connect UI items
  QObject::connect(ui->addButton, SIGNAL(clicked(bool)), 
                   this, SLOT(addObservationButtonPressed(bool)));
  
  QObject::connect(ui->reflectSpinBox, SIGNAL(valueChanged(double)),
                   this, SLOT(spinBoxValueChanged(double)));
  
  QObject::connect(ui->sourceButton, SIGNAL(clicked(bool)), this,
                   SLOT(sampleSourceButtonPressed(bool)));
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
  
  calibratorUpdateState();  //  trigger UI update
}

void SpectrumCalibrationView::calibratorUpdateState(void) {
  const cv::Mat& image = specCalib_->getUserImage();
  if (!image.empty()) {
    ui->imageWidget->setImage(image);
  }
  
  ui->addButton->setEnabled(specCalib_->hasSourceSpectrum() && specCalib_->hasMeasurement());
  ui->sourceButton->setEnabled(specCalib_->hasMeasurement());
  
  //  get range of spectrum
  galt::Spectrum spec = specCalib_->getSpectrum();
  if (!spec.size()) {
    return;
  }
  galt::Spectrum profile = specCalib_->getFilterProfile().getSpectrum();
  galt::Spectrum source = specCalib_->getSourceSpectrum();
  
  //  figure out size of plot axes
  double maxIntensity=0.0;
  for (double I : spec.getIntensities()) {
    maxIntensity = std::max(I, maxIntensity);
  }
  profile.scale(maxIntensity);  //  for visual effect only
  
  const double minWavelength = spec.getWavelengths().front();
  const double maxWavelength = spec.getWavelengths().back();
   
  //  update spectrum plot, just replot every time for now
  SpectralPlot * plot = ui->spectrumPlot;
  
  if (maxIntensity > currentMax_) {
    plot->setAxisScale(QwtPlot::yLeft, 0.0, maxIntensity);
    currentMax_ = maxIntensity;
  }
  plot->setAxisScale(QwtPlot::xBottom, minWavelength, maxWavelength);
  
  plot->updatePlot("ocean_optics", QPen(Qt::blue, 2), spec);
  plot->updatePlot("source", QPen(Qt::green, 2), source);
  plot->updatePlot("filter_profile", QPen(Qt::red, 2), profile);
 
  plot->replot();
  
  //  update plot of camera + spectrometer samples  
}

void SpectrumCalibrationView::spinBoxValueChanged(double value) {
  if (specCalib_) {
    value = std::max(std::min(value, 1.0), 0.0);
    specCalib_->setCurrentReflectance(value);
    ROS_INFO("Updated current reflectance: %f", value);
  }
}

void SpectrumCalibrationView::sampleSourceButtonPressed(bool) {
  if (specCalib_ && specCalib_->hasMeasurement()) {
    specCalib_->setSource();
    ROS_INFO("Setting the source profile");
  }
}

void SpectrumCalibrationView::addObservationButtonPressed(bool) {
  if (specCalib_ && specCalib_->hasMeasurement()) {
    //  capture...
  }
}

void SpectrumCalibrationView::calibrateButtonPressed(bool) {
  
  
  
}

void SpectrumCalibrationView::resetButtonPressed(bool) {
  
}

void SpectrumCalibrationView::saveButtonPressed(bool) {
  
}
