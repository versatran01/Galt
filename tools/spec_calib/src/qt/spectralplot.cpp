/*
 * spectralplot.cpp
 *
 *  Copyright (c) 2014 Nouka Technologies. All rights reserved.
 *
 *  This file is part of galt.
 *
 *  Created on: 21/7/2014
 *      Author: gareth
 */

#include "spectralplot.h"
#include <qwt/qwt_symbol.h>

SpectralPlot::SpectralPlot(QWidget *parent) :
  QwtPlot(parent)
{
}

static QPolygonF spectrumToPoly(const galt::Spectrum& spec) {
  QPolygonF points;
  for (size_t i=0; i < spec.size(); i++) {
    double wavelen = spec.getWavelengths()[i];
    double intensity = spec.getIntensities()[i];
    points.push_back(QPointF(wavelen,intensity));    
  }
  return points;
}

void SpectralPlot::updatePlot(const std::string& title, const QPen &brush, 
                              const galt::Spectrum& spectrum) {
  
  auto I = curves_.find(title);
  QwtPlotCurve * qpc;
  if (I == curves_.end())
  {
    //  insert new plot
    qpc = new QwtPlotCurve();
    
    qpc->setTitle(title.c_str());
    qpc->setPen(brush);
    qpc->setRenderHint(QwtPlotItem::RenderAntialiased, true);
    qpc->setSymbol( new QwtSymbol() );
    
    qpc->attach(this);
    curves_[title] = qpc;
  } else {
    //  update old plot
    qpc = I->second;
  }
  qpc->setSamples(spectrumToPoly(spectrum));
}
