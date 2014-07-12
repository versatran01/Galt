#include "customwidget.hpp"

#include <QTime>
#include <QPainter>
#include <QPainterPath>

CustomWidget::CustomWidget(QWidget *parent) :
  QWidget(parent)
{
}

void CustomWidget::paintEvent(QPaintEvent *)
{ 
  QColor emptyColor(127, 127, 127);
  QColor filledColor(0, 127, 0);
  
  int side = qMin(width(), height());
  
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.translate(width() / 2, height() / 2);
  painter.scale(side / 200.0, side / 200.0);
  
  painter.setBrush(emptyColor);  
  painter.setPen(emptyColor);
  
  for (int i = 0; i < 120; ++i) {
    painter.drawLine(88, 0, 96, 0);
    painter.drawLine(76, 0, 84, 0);
    painter.rotate(360 / 120);
  }
  
  painter.setBrush(filledColor);  
  painter.setPen(filledColor);
  
  for (int j = 0; j < 10; ++j) {
    painter.drawLine(88, 0, 96, 0);
    painter.rotate(360 / 120);
  }
}

void CustomWidget::resizeEvent(QResizeEvent *)
{
}
