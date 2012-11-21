#include <velib/opencv_image.h>
#include <QDebug>
using namespace ve;

OpenCVImage::OpenCVImage()
  : has_image_(false)
{
  setFlags(ItemIsMovable|ItemIsSelectable|ItemIsFocusable|ItemSendsGeometryChanges);
  prepareGeometryChange();
}

OpenCVImage::OpenCVImage(const cv::Mat& image)
  : image_(image)
{
  q_image_ = QImage(image_.data,
                    image_.cols,
                    image_.rows,
                    image_.step, QImage::Format_RGB888);
  has_image_ = true;
  prepareGeometryChange();
}

void OpenCVImage::updateImage(const cv::Mat& image)
{
  image_= image;
  prepareGeometryChange();
  q_image_ = QImage(image_.data,
                    image_.cols,
                    image_.rows,
                    image_.step, QImage::Format_RGB888);

  has_image_ = true;
  update();
}

QRectF OpenCVImage::boundingRect() const
{
  return q_image_.rect();
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
  //qDebug() << event->pos();
  //QGraphicsItem::mousePressEvent(event);
  QColor pixel(q_image_.pixel(event->pos().toPoint()));
  //qDebug() << pixel;
  Q_EMIT mouseClicked(event->pos(), q_image_.pixel(event->pos().toPoint()));

}
