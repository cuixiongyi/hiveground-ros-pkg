#include <velib/opencv_image.h>
#include <QDebug>
using namespace ve;

OpenCVImage::OpenCVImage()
  : has_image_(false)
{

}

OpenCVImage::OpenCVImage(const cv::Mat& image)
  : image_(image)
{
  q_image_ = QImage(image_.data,
                    image_.cols,
                    image_.rows,
                    image_.step, QImage::Format_RGB888);
  has_image_ = true;
}

void OpenCVImage::updateImage(const cv::Mat& image)
{
  image_= image;
  q_image_ = QImage(image_.data,
                    image_.cols,
                    image_.rows,
                    image_.step, QImage::Format_RGB888);
  has_image_ = true;
  update();
}

QRectF OpenCVImage::boundingRect() const
{
  return QRect(0, 0, image_.cols, image_.rows);
}

QPainterPath OpenCVImage::shape() const
{
  QPainterPath path;
  path.addRect(boundingRect());
  return path;
}

void OpenCVImage::paint(QPainter *painter, const QStyleOptionGraphicsItem *item, QWidget *widget)
{
  if(has_image_)
    painter->drawImage(QPointF(0, 0), q_image_);
}

void OpenCVImage::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
  qDebug() << __FUNCTION__;
}
