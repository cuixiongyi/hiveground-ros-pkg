#ifndef VELIB_LINK_H
#define VELIB_LINK_H

#include <ve_global.h>
#include <QtGui>

namespace ve
{
class Node;

class Link : public QObject, public QGraphicsItem
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)

public:
  Link();
  Link(const QPointF& point_out, const QPointF& point_in);
  Link(Node* node_out, Node* node_in);
  virtual ~Link();

  enum
  {
    TYPE = UserType + VE_TYPE + 1
  };
  int type() const
  {
    return TYPE;
  }

  void adjust();
  void initialize();
  void prepare_for_delete();

protected:
  QRectF boundingRect() const;
  QPainterPath shape() const;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

  void hoverEnterEvent(QGraphicsSceneHoverEvent * event);
  void hoverLeaveEvent(QGraphicsSceneHoverEvent * event);

  void contextMenuEvent(QGraphicsSceneContextMenuEvent *event);

  QVariant itemChange(GraphicsItemChange change, const QVariant &value);

protected:
  friend class Node;
  friend class Scene;

private:
  QPointF point_out_;
  QPointF point_in_;

  Node* node_out_;
  Node* node_in_;
  QPolygonF arrow_head_;

};

}
#endif
