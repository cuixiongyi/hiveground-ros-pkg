#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <boost/smart_ptr.hpp>

class QPaintEvent;
class QWidget;

class GLWidget : public QGLWidget
{
  Q_OBJECT

public:
  GLWidget(QWidget *parent);

public slots:
  //void updateImage(boost::shared_ptr<QImage> image_ptr); 
  void updateImage(const QImage& image);

protected:
  void paintEvent(QPaintEvent *event);

private:
  //boost::shared_ptr<QImage> image_ptr_;
  QImage image_;
};

#endif