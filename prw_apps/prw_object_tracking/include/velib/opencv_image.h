#ifndef VELIB_OPENCV_IMAGE_H
#define VELIB_OPENCV_IMAGE_H

#include <velib/velib_global.h>
#include <opencv2/opencv.hpp>

namespace ve
{

class VELIB_EXPORT OpenCVImage : public QObject, public QGraphicsItem
{
  Q_OBJECT
public:
  OpenCVImage();
  OpenCVImage(const cv::Mat& image);
  void updateImage(const cv::Mat& image);
  QRectF boundingRect() const;
  QPainterPath shape() const;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *item, QWidget *widget);

signals:
  void mouseClicked(QPointF point, QRgb color);

protected:
  void mousePressEvent(QGraphicsSceneMouseEvent *event);

protected:
  cv::Mat image_;
  QImage q_image_;
  bool has_image_;
};

}

#endif