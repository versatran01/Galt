/*
 * spectralplot.h
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 21/7/2014
 *      Author: gareth
 */

#ifndef SPECTRALPLOT_H
#define SPECTRALPLOT_H

#include <QWidget>
#include <QPen>

#include <qwt/qwt.h>
#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_curve.h>
#include <map>

#include <spectral/Spectrum.hpp>

class SpectralPlot : public QwtPlot
{
  Q_OBJECT
public:
  explicit SpectralPlot(QWidget *parent = 0);
  
  void updatePlot(const std::string& title, const QPen &pen, const galt::Spectrum& spectrum);
  
signals:
  
public slots:
  
private:
  
  std::map<std::string,QwtPlotCurve*> curves_;  
};

#endif // SPECTRALPLOT_H
