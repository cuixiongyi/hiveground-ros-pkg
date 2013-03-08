#include <ve_link.h>
#include <ve_node.h>
#include <ve_scene.h>
#include <QDebug>
#include <cmath>

using namespace ve;

const qreal Pi = 3.14;

Link::Link()
{
  //qDebug() << __FUNCTION__;
  initialize();
}

Link::Link(const QPointF& point_out, const QPointF& point_in) :
    point_out_(point_out), point_in_(point_in)
{
  initialize();
}

Link::Link(Node* node_out, Node* node_in) :
    node_out_(node_out), node_in_(node_in)
{
  initialize();
  adjust();
}

Link::~Link()
{
  //qDebug() << __FUNCTION__;
}

void Link::adjust()
{
  if ((node_out_ != 0) && (node_out_ != 0))
  {
    point_out_ = node_out_->mapToScene(node_out_->get_node_rect().center());
    point_in_ = node_in_->mapToScene(node_in_->get_node_rect().center());
    //prepareGeometryChange();
    update();
  }
}

void Link::prepare_for_delete()
{
  qDebug() << __FUNCTION__;
  node_out_->remove_output_link();
  node_in_->remove_input_link();
  node_out_ = 0;
  node_in_ = 0;
  ve::Scene* scene = qobject_cast<ve::Scene*>(this->scene());
  Q_EMIT scene->signal_link_deleted(this);
}

void Link::initialize()
{
  setFlags(ItemIsSelectable | ItemIsFocusable | ItemSendsGeometryChanges);
  setCacheMode(DeviceCoordinateCache);
  setBoundingRegionGranularity(0.25);
  setAcceptHoverEvents(true);
  setZValue(1.0f);
}

QRectF Link::boundingRect() const
{
  qreal extra = 10;

  return QRectF(point_out_, QSizeF(point_in_.x() - point_out_.x(), point_in_.y() - point_out_.y())).normalized().adjusted(
      -extra, -extra, extra, extra);
}

QPainterPath Link::shape() const
{

  QPainterPath path;
  path.addRegion(boundingRegion(this->transform()));
  path.addPolygon(arrow_head_);
  return path;
}

void Link::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
  QLineF line(point_out_, point_in_);
  QColor color;
  if (qFuzzyCompare(line.length(), qreal(0.)))
    return;

  if (option->state & QStyle::State_MouseOver)
  {
    color = Qt::red;
  }
  else
  {
    color = Qt::cyan;
  }

  painter->setPen(QPen(color, 2));
  painter->drawLine(line);

  qreal arrowSize = 8;
  double angle = ::acos(line.dx() / line.length());
  if (line.dy() >= 0)
    angle = (Pi * 2) - angle;

  QPointF arrowP1 = line.p2() + QPointF(sin(angle - Pi / 3) * arrowSize, cos(angle - Pi / 3) * arrowSize);
  QPointF arrowP2 = line.p2() + QPointF(sin(angle + Pi + Pi / 3) * arrowSize, cos(angle + Pi + Pi / 3) * arrowSize);

  arrow_head_.clear();
  arrow_head_ << line.p2() << arrowP1 << arrowP2;
  painter->drawLine(line);
  painter->drawPolygon(arrow_head_);
}

void Link::hoverEnterEvent(QGraphicsSceneHoverEvent * event)
{
  //qDebug() << __FUNCTION__;
  update();
}

void Link::hoverLeaveEvent(QGraphicsSceneHoverEvent * event)
{
  //qDebug() << __FUNCTION__;
  update();
}

void Link::contextMenuEvent(QGraphicsSceneContextMenuEvent *event)
{
  //if(!isSelected()) return;

  QMenu menu;
  QAction *remove_action = menu.addAction(tr("Remove"));
  //QAction *markAction = menu.addAction("Mark");
  QAction *selected_action = menu.exec(event->screenPos());

  if (selected_action == remove_action)
  {
    qDebug() << "remove:" << objectName();
    prepare_for_delete();
  }
}

QVariant Link::itemChange(GraphicsItemChange change, const QVariant &value)
{
  ve::Scene* scene = qobject_cast<ve::Scene*>(this->scene());
  switch (change)
  {
    case ItemSelectedChange:
    {

      if (value.toBool())
      {
        Q_EMIT scene->signal_object_selected(this);
      }
    }
      break;
    default:
      break;
  };

  return QGraphicsItem::itemChange(change, value);
}
