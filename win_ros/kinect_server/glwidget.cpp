#include <QtGui>
#include "glwidget.h"


GLWidget::GLWidget(QWidget *parent)
  : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
  setFixedSize(640, 480);
  setAutoFillBackground(false);
}

//void GLWidget::updateImage(boost::shared_ptr<QImage> image_ptr)
void GLWidget::updateImage(const QImage& image)
{
  image_ = image;
  repaint();
}

void GLWidget::paintEvent(QPaintEvent *event)
{
  QPainter painter;
  painter.begin(this);
  painter.setRenderHint(QPainter::Antialiasing);

  if(!image_.isNull())
  {
    painter.drawImage(0, 0, image_);
  }
  //painter.fillRect(event->rect(), Qt::white);
  //painter.setPen(Qt::black);
  //painter.drawEllipse(QPoint(100, 100), 10, 10);


  painter.end();
}