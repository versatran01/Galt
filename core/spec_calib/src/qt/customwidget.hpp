#ifndef CUSTOMWIDGET_H
#define CUSTOMWIDGET_H

#include <QWidget>

class CustomWidget : public QWidget
{
  Q_OBJECT
public:
  explicit CustomWidget(QWidget *parent = 0);
  
protected:
    void paintEvent(QPaintEvent *);
    void resizeEvent(QResizeEvent *);
    
signals:
  
public slots:

};

#endif // CUSTOMWIDGET_H
