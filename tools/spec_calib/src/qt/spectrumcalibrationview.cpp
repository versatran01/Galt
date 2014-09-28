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
#include <QLabel>

#include "spectrumcalibrationview.h"
#include "ui_spectrumcalibrationview.h"
#include "spectralplot.h"

#include <ros/package.h>
#include <qwt/qwt.h>
#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_grid.h>
#include <qwt/qwt_legend.h>
#include <qwt/qwt_symbol.h>
#include <qwt/qwt_series_data.h>

#include <yaml-cpp/yaml.h>

class CalibrationFunction : public QwtSyntheticPointData {
public:
  
  CalibrationFunction(double m, double b) : QwtSyntheticPointData(100,QwtInterval(0,1)) {
    m_ = m;
    b_ = b;
  }
  
  virtual double y(double x) const {
    return m_*x + b_;
  }
  
private:
  double m_,b_;
};

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
  
  //  configure spectral plot
  SpectralPlot * plot = ui->spectrumPlot;
  plot->setTitle("Spectrum");
  plot->setCanvasBackground(Qt::white);
  plot->setAxisScale(QwtPlot::yLeft, 0.0, 1.0);
  plot->setAxisScale(QwtPlot::xBottom, 350.0, 1050.0);
  plot->resize(320,240);
  plot->repaint();
  (new QwtPlotGrid())->attach(plot);
  
  //  configure calibration plot for camera
  QwtPlot * camPlot = ui->camCalibPlot;
  camPlot->setTitle("Camera");
  camPlot->setCanvasBackground(Qt::white);
  camPlot->setAxisScale(QwtPlot::yLeft, 0.0, 1.0);
  camPlot->setAxisScale(QwtPlot::xBottom, 0.0, 1.0);
  camPlot->setAxisTitle(QwtPlot::xBottom, "Intensity");
  camPlot->setAxisTitle(QwtPlot::yLeft, "Reflectance");
  camPlot->repaint();
  (new QwtPlotGrid())->attach(camPlot);
  
  camCurve_ = new QwtPlotCurve("camera_measurements");
  camCurve_->setRenderHint(QwtPlotItem::RenderAntialiased, true);
  
  QwtSymbol * sym = new QwtSymbol(QwtSymbol::Ellipse);
  sym->setSize(6);
  camCurve_->setSymbol(sym);
  camCurve_->setStyle(QwtPlotCurve::Dots);
  camCurve_->setPen( QPen(Qt::red, 2) );
  camCurve_->attach(camPlot);
  
  camCalibCurve_ = new QwtPlotCurve("camera_calibration");
  camCalibCurve_->setRenderHint(QwtPlotItem::RenderAntialiased, true);
  camCalibCurve_->setSymbol( new QwtSymbol() );
  camCalibCurve_->setPen( QPen(Qt::green, 2) );
  camCalibCurve_->attach(camPlot);
  
  //  connect UI items
  QObject::connect(ui->addButton, SIGNAL(clicked(bool)), 
                   this, SLOT(addObservationButtonPressed(bool)));
  
  QObject::connect(ui->reflectSpinBox, SIGNAL(valueChanged(double)),
                   this, SLOT(spinBoxValueChanged(double)));
  
  QObject::connect(ui->sourceButton, SIGNAL(clicked(bool)), this,
                   SLOT(sampleSourceButtonPressed(bool)));
  
  QObject::connect(ui->calibrateButton, SIGNAL(clicked(bool)),
                   this, SLOT(calibrateButtonPressed(bool)));
  
  QObject::connect(ui->resetButton, SIGNAL(clicked(bool)),
                   this, SLOT(resetButtonPressed(bool)));
  
  QObject::connect(ui->saveButton, SIGNAL(clicked(bool)),
                   this, SLOT(saveButtonPressed(bool)));
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
  
  specCalib_ = new SpectrumCalibrator(this,camSerial,pose,filterProfile);
  QObject::connect(specCalib_,SIGNAL(receivedMessage()),this,SLOT(calibratorUpdateState()));
  QObject::connect(specCalib_,SIGNAL(receivedSpectrum()),this,SLOT(calibratorReceivedSpectrum()));
  
  calibratorUpdateState();  //  trigger UI update
  calibratorReceivedSpectrum();
}

void SpectrumCalibrationView::calibratorUpdateState(void) {
  const cv::Mat& image = specCalib_->getUserImage();
  if (!image.empty()) {
    ui->imageWidget->setImage(image);
  }
  
  ui->addButton->setEnabled(specCalib_->hasSourceSpectrum() && specCalib_->hasMeasurement());
  ui->sourceButton->setEnabled(specCalib_->hasSpectrum());
  ui->calibrateButton->setEnabled(specCalib_->canCalibrate());
  ui->saveButton->setEnabled(specCalib_->hasCalibration());
  
  if (!specCalib_->hasMeasurement()) {
    return;
  }
  
  //  update camera plot
  const auto& observations = specCalib_->getObservations();
  QPolygonF points;
  if (!observations.empty()) {
    
    for (size_t i=0; i < observations.size(); i++) 
    {
      const SpectrumCalibrator::Observation& O = observations[i];
      points.push_back(QPointF(O.intensityCam,O.sample.reflectance));    
    }
  }
  camCurve_->setSamples(points);  
  ui->obvsCount->setText(std::to_string(observations.size()).c_str());
  
  if (specCalib_->hasCalibration()) {
    
    const galt::CameraCalibration& calib = specCalib_->getCameraCalibration();
    
    //  place new plot on the camera chart
    camCalibCurve_->setData( new CalibrationFunction(calib.slope,calib.intercept) );
    camCalibCurve_->setVisible(true);
    ui->error->setText(std::to_string(std::sqrt(calib.squaredError)).c_str());
  } else {
    camCalibCurve_->setVisible(false);
    ui->error->setText("0");
  }
  ui->camCalibPlot->replot();
  
  //  update incident angle reading
  kr::Pose<double> pose = specCalib_->getPose();  //  tags in camera frame
  
  kr::vec3d tagZ = pose.bRw() * kr::vec3d(0,0,1);
  
  const double dot = -tagZ[2];
  const double ang = std::acos(dot);
  ui->angle->setText(std::to_string(ang * 180 / M_PI).c_str());
}

void SpectrumCalibrationView::calibratorReceivedSpectrum(void) {
  //  get range of spectrum
  galt::Spectrum spec = specCalib_->getSpectrum();
  if (!spec.size()) {
    return;
  }
  ui->sourceButton->setEnabled(specCalib_->hasSpectrum());  
  
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
  if (source.size()) {
    plot->updatePlot("source", QPen(Qt::green, 2), source);
  }
  if (profile.size()) {
    plot->updatePlot("filter_profile", QPen(Qt::red, 2), profile);
  }
  plot->replot();
}

void SpectrumCalibrationView::spinBoxValueChanged(double value) {
  if (specCalib_) {
    value = std::max(std::min(value, 1.0), 0.0);
    specCalib_->setCurrentReflectance(value);
    ROS_INFO("Updated current reflectance: %f", value);
  }
}

void SpectrumCalibrationView::sampleSourceButtonPressed(bool) {
  if (specCalib_ && specCalib_->hasSpectrum()) {
    specCalib_->setSource();
    ROS_INFO("Setting the source profile");
  }
}

void SpectrumCalibrationView::addObservationButtonPressed(bool) {
  if (specCalib_ && specCalib_->hasMeasurement()) {
    specCalib_->collectSample();
    ROS_INFO("Collecting observation");
  }
}

void SpectrumCalibrationView::calibrateButtonPressed(bool) {
  if (specCalib_ && specCalib_->canCalibrate()) {
    specCalib_->calibrate();
  }
}

void SpectrumCalibrationView::resetButtonPressed(bool) {
  reset();
  ROS_INFO("Resetting");
}

//  TODO: this is ugly crap, fix it...
void SpectrumCalibrationView::saveButtonPressed(bool) {
  if (specCalib_ && specCalib_->hasCalibration()) {
    const galt::CameraCalibration& calib = specCalib_->getCameraCalibration();
    const std::string& camSerial = calib.cameraSerial;
    
    ros::NodeHandle nh("~");
    std::string sessionPath;
    nh.getParam("session_path", sessionPath); //  path to save all output
    
    const std::string& date = calib.calibrationDate;
  
    //  nice formatted YAML output
    YAML::Emitter emitter;
    emitter.SetSeqFormat(YAML::Flow);
    emitter << YAML::Comment("Camera spectral calibration") << YAML::Newline;
    emitter << YAML::Comment("Camera: " + camSerial) << YAML::Newline;
    emitter << YAML::Comment("Generated on: " + date) << YAML::Newline;
    emitter << calib;
    
    std::string filePath = sessionPath + "/camera_" + camSerial + ".yaml";
    
    QFile file(filePath.c_str());
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
      ROS_ERROR("Failed to open file for writing: %s", filePath.c_str());
      return;
    }
    
    QTextStream out(&file);
    out << emitter.c_str();
    
    file.close(); 
    ROS_INFO("Wrote camera calibration to %s", filePath.c_str());
    
    //  now write spectrometer data
    YAML::Emitter specEmitter;
    specEmitter.SetSeqFormat(YAML::Flow);
    specEmitter << YAML::Comment("Spectrometer calibration data") << YAML::Newline;
    specEmitter << YAML::Comment("Generated on: " + date) << YAML::Newline;
    
    galt::SpectrometerCalibration sc;
    sc.calibrationDate = date;
    sc.sourceSpectrum = specCalib_->getSourceSpectrum();
    
    for (const SpectrumCalibrator::Observation& o : specCalib_->getObservations()) {
      sc.sampleSpectra.push_back(o.sample);
    }
    
    specEmitter << sc;
    
    filePath = sessionPath + "/spectrometer.yaml";
    QFile specFile(filePath.c_str());
    if (!specFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
      ROS_ERROR("Failed to open file for writing: %s", filePath.c_str());
      return;
    }
    
    QTextStream specOut(&specFile);
    specOut << specEmitter.c_str();
    specFile.close();
    
    ROS_INFO("Wrote spectrometer calibration data to %s", filePath.c_str());
  }
}
