/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Imai Laboratory, Keio University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Imai Laboratory, nor the name of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mahisorn Wongphati
 */

#include <ve_link.h>
#include <ve_node.h>
#include <ve_scene.h>


using namespace ve;

Node::Node()
{
  //qDebug() << __FUNCTION__;
  initialize();
}

Node::Node(const QRectF& rect) :
    node_rect_(rect)
{
  initialize();
}

Node::~Node()
{

}

void Node::set_edit_mode(bool edit_mode)
{
  edit_mode_ = edit_mode;
}

/*
 QRectF Node::input_link_rect(int line) const
 {
 }
 

 QRectF Node::output_link_rect(int line) const
 {
 }
 */

void Node::initialize()
{
  edit_mode_ = false;
  corner_resize_ = false;

  link_in_ = 0;
  link_out_ = 0;

  setFlags(ItemIsMovable | ItemIsSelectable | ItemIsFocusable | ItemSendsGeometryChanges);
  setCacheMode(DeviceCoordinateCache);
  setAcceptHoverEvents(true);
  setZValue(2.0f);
}

void Node::draw_editor(QPainter *painter)
{
  painter->setPen(QPen(Qt::yellow, 1, Qt::DashDotLine));
  painter->drawRect(node_rect_);

  //draw corner
  painter->setPen(QPen(Qt::black, 1));
  painter->drawRect(node_rect_.topLeft().x() - RESIZE_CORNER_MARK_SIZE,
                    node_rect_.topLeft().y() - RESIZE_CORNER_MARK_SIZE, RESIZE_CORNER_MARK_SIZE * 2,
                    RESIZE_CORNER_MARK_SIZE * 2);
  painter->drawRect(node_rect_.bottomLeft().x() - RESIZE_CORNER_MARK_SIZE,
                    node_rect_.bottomLeft().y() - RESIZE_CORNER_MARK_SIZE, RESIZE_CORNER_MARK_SIZE * 2,
                    RESIZE_CORNER_MARK_SIZE * 2);
  painter->drawRect(node_rect_.bottomRight().x() - RESIZE_CORNER_MARK_SIZE,
                    node_rect_.bottomRight().y() - RESIZE_CORNER_MARK_SIZE, RESIZE_CORNER_MARK_SIZE * 2,
                    RESIZE_CORNER_MARK_SIZE * 2);
  painter->drawRect(node_rect_.topRight().x() - RESIZE_CORNER_MARK_SIZE,
                    node_rect_.topRight().y() - RESIZE_CORNER_MARK_SIZE, RESIZE_CORNER_MARK_SIZE * 2,
                    RESIZE_CORNER_MARK_SIZE * 2);

  //draw center
  painter->drawRect(node_rect_.topLeft().x() - RESIZE_CENTER_MARK_SIZE,
                    node_rect_.center().y() - RESIZE_CENTER_MARK_SIZE, RESIZE_CENTER_MARK_SIZE * 2,
                    RESIZE_CENTER_MARK_SIZE * 2);
  painter->drawRect(node_rect_.center().x() - RESIZE_CENTER_MARK_SIZE,
                    node_rect_.bottomLeft().y() - RESIZE_CENTER_MARK_SIZE, RESIZE_CENTER_MARK_SIZE * 2,
                    RESIZE_CENTER_MARK_SIZE * 2);
  painter->drawRect(node_rect_.bottomRight().x() - RESIZE_CENTER_MARK_SIZE,
                    node_rect_.center().y() - RESIZE_CENTER_MARK_SIZE, RESIZE_CENTER_MARK_SIZE * 2,
                    RESIZE_CENTER_MARK_SIZE * 2);
  painter->drawRect(node_rect_.center().x() - RESIZE_CENTER_MARK_SIZE,
                    node_rect_.topRight().y() - RESIZE_CENTER_MARK_SIZE, RESIZE_CENTER_MARK_SIZE * 2,
                    RESIZE_CENTER_MARK_SIZE * 2);
}

int Node::get_editor_corner(const QPointF& position, QPointF& corner, QPointF& pivot_corner)
{
  QRectF top_left(node_rect_.topLeft().x() - RESIZE_CORNER_MARK_SIZE,
                  node_rect_.topLeft().y() - RESIZE_CORNER_MARK_SIZE, RESIZE_CORNER_MARK_SIZE * 2,
                  RESIZE_CORNER_MARK_SIZE * 2);
  QRectF bottom_left(node_rect_.bottomLeft().x() - RESIZE_CORNER_MARK_SIZE,
                     node_rect_.bottomLeft().y() - RESIZE_CORNER_MARK_SIZE, RESIZE_CORNER_MARK_SIZE * 2,
                     RESIZE_CORNER_MARK_SIZE * 2);
  QRectF bottom_right(node_rect_.bottomRight().x() - RESIZE_CORNER_MARK_SIZE,
                      node_rect_.bottomRight().y() - RESIZE_CORNER_MARK_SIZE, RESIZE_CORNER_MARK_SIZE * 2,
                      RESIZE_CORNER_MARK_SIZE * 2);
  QRectF top_right(node_rect_.topRight().x() - RESIZE_CORNER_MARK_SIZE,
                   node_rect_.topRight().y() - RESIZE_CORNER_MARK_SIZE, RESIZE_CORNER_MARK_SIZE * 2,
                   RESIZE_CORNER_MARK_SIZE * 2);

  top_left.adjust(0, 0, 2, 2);
  bottom_left.adjust(0, 0, 2, 2);
  bottom_right.adjust(0, 0, 2, 2);
  top_right.adjust(0, 0, 2, 2);

  if (top_left.contains(position))
  {
    corner = top_left.center();
    pivot_corner = bottom_right.center();
    return 0;
  }
  if (bottom_left.contains(position))
  {
    corner = bottom_left.center();
    pivot_corner = top_right.center();
    return 1;
  }
  if (bottom_right.contains(position))
  {
    corner = bottom_right.center();
    pivot_corner = top_left.center();
    return 2;
  }
  if (top_right.contains(position))
  {
    corner = top_right.center();
    pivot_corner = bottom_left.center();
    return 3;
  }
  return -1;
}

int Node::get_editor_center(const QPointF& position, QPointF& center)
{
  QRectF left(node_rect_.topLeft().x() - RESIZE_CENTER_MARK_SIZE, node_rect_.center().y() - RESIZE_CENTER_MARK_SIZE,
              RESIZE_CENTER_MARK_SIZE * 2, RESIZE_CENTER_MARK_SIZE * 2);
  QRectF bottom(node_rect_.center().x() - RESIZE_CENTER_MARK_SIZE,
                node_rect_.bottomLeft().y() - RESIZE_CENTER_MARK_SIZE, RESIZE_CENTER_MARK_SIZE * 2,
                RESIZE_CENTER_MARK_SIZE * 2);
  QRectF right(node_rect_.bottomRight().x() - RESIZE_CENTER_MARK_SIZE,
               node_rect_.center().y() - RESIZE_CENTER_MARK_SIZE, RESIZE_CENTER_MARK_SIZE * 2,
               RESIZE_CENTER_MARK_SIZE * 2);
  QRectF top(node_rect_.center().x() - RESIZE_CENTER_MARK_SIZE, node_rect_.topRight().y() - RESIZE_CENTER_MARK_SIZE,
             RESIZE_CENTER_MARK_SIZE * 2, RESIZE_CENTER_MARK_SIZE * 2);

  left.adjust(0, 0, 2, 2);
  bottom.adjust(0, 0, 2, 2);
  right.adjust(0, 0, 2, 2);
  top.adjust(0, 0, 2, 2);

  if (left.contains(position))
  {
    center = left.center();
    return 0;
  }
  if (bottom.contains(position))
  {
    center = bottom.center();
    return 1;
  }
  if (right.contains(position))
  {
    center = right.center();
    return 2;
  }
  if (top.contains(position))
  {
    center = top.center();
    return 3;
  }
  return -1;
}

QRectF Node::get_rect_from_two_points(QPointF p1, QPointF p2)
{
  if (p1.x() < p2.x())
  {
    if (p1.y() < p2.y())
    {
      //p1 top left
      return QRectF(p1, p2);
    }
    else
    {
      //p1 bottom left
      qreal temp_y = p1.y();
      p1.setY(p2.y());
      p2.setY(temp_y);
      return QRectF(p1, p2);
    }
  }
  else
  {
    if (p1.y() < p2.y())
    {
      //p1 top right
      qreal temp_x = p1.x();
      p1.setX(p2.x());
      p2.setX(temp_x);
      return QRectF(p1, p2);
    }
    else
    {
      //p1 bottom right
      return QRectF(p2, p1);
    }
  }
}

void Node::adjust_links()
{
  if (link_in_)
    link_in_->adjust();
  if (link_out_)
    link_out_->adjust();
}

void Node::remove_input_link()
{
  link_in_ = 0;
}

void Node::remove_output_link()
{
  link_out_ = 0;
}

QRectF Node::boundingRect() const
{
  return node_rect_.adjusted(-(RESIZE_CORNER_MARK_SIZE + 2), -(RESIZE_CORNER_MARK_SIZE + 2),
                             (RESIZE_CORNER_MARK_SIZE + 2), (RESIZE_CORNER_MARK_SIZE + 2));
}

QPainterPath Node::shape() const
{
  QPainterPath path;
  path.addRect(node_rect_);
  return path;
}

void Node::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
  if (option->state & QStyle::State_MouseOver)
  {
    painter->setPen(QPen(Qt::red, 1));
  }
  else
  {
    painter->setPen(QPen(Qt::green, 1));
  }
  painter->drawRect(node_rect_);
  painter->drawEllipse(node_rect_.center(), CENTER_RADIUS, CENTER_RADIUS);

  if (edit_mode_)
  {
    draw_editor(painter);
  }

}

void Node::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
  //qDebug() << __FUNCTION__;
  bool handle = false;
  if (edit_mode_)
  {
    QPointF corner, pivot_corner;
    int corner_index = get_editor_corner(event->pos(), corner, pivot_corner);
    QPointF center;
    int center_index = get_editor_center(event->pos(), center);
    if (corner_index != -1)
    {
      //resize corner
      corner_resize_ = true;
      corner_index_ = corner_index;
      corner_ = corner;
      pivot_corner_ = pivot_corner;
      handle = true;
    }
    else if (center_index != -1)
    {
      //resize edge
      center_resize_ = true;
      center_index_ = center_index;
      center_ = center;
      handle = true;
    }
  }

  if (!handle)
    QGraphicsItem::mousePressEvent(event);

}

void Node::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
  //qDebug() << __FUNCTION__;
  corner_resize_ = false;
  center_resize_ = false;

  QGraphicsItem::mouseReleaseEvent(event);
}

void Node::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
  //qDebug() << __FUNCTION__;
  ve::Scene* scene = qobject_cast<ve::Scene*>(this->scene());
  bool handle = false;
  if (edit_mode_)
  {
    if (corner_resize_)
    {
      //qDebug() << "Corner resize:" << corner_index_ << corner_ << event->pos();
      corner_temp_ = event->pos();
      QRectF rect_temp = get_rect_from_two_points(pivot_corner_, corner_temp_);
      //qDebug() << pivot_corner_ << corner_temp_;
      //qDebug() << node_rect_ << rect_temp;
      prepareGeometryChange();
      node_rect_ = rect_temp;
      adjust_links();
      Q_EMIT scene->signal_node_size_changed(this);
      handle = true;
    }
    else if (center_resize_)
    {
      float dx = event->pos().x() - center_.x();
      float dy = event->pos().y() - center_.y();
      center_ = event->pos();
      prepareGeometryChange();
      switch (center_index_)
      {
        case 0:
          node_rect_.adjust(dx, 0, 0, 0);
          break;
        case 1:
          node_rect_.adjust(0, 0, 0, dy);
          break;
        case 2:
          node_rect_.adjust(0, 0, dx, 0);
          break;
        case 3:
          node_rect_.adjust(0, dy, 0, 0);
          break;
      }
      Q_EMIT scene->signal_node_size_changed(this);
      handle = true;
    }
  }

  //qDebug() << mapToScene(node_rect_.center());

  if (!handle)
    QGraphicsItem::mouseMoveEvent(event);
}

void Node::hoverEnterEvent(QGraphicsSceneHoverEvent * event)
{
  //qDebug() << __FUNCTION__;

  //if(!edit_mode_)
  //{
  ve::Scene* scene = qobject_cast<ve::Scene*>(this->scene());
  if (scene->mode_ == ve::Scene::MODE_CURSOR)
    Q_EMIT scene->signal_node_mouse_hover_enter(this);
  //}
  //else
  //{
  update();
  //}

}

void Node::hoverLeaveEvent(QGraphicsSceneHoverEvent * event)
{
  //qDebug() << __FUNCTION__;
  //if(!edit_mode_)
  //{
  ve::Scene* scene = qobject_cast<ve::Scene*>(this->scene());
  if (scene->mode_ == ve::Scene::MODE_CURSOR)
    Q_EMIT scene->signal_node_mouse_hover_leave(this);
  //}
  //else
  //{
  update();
  //}
}

void Node::hoverMoveEvent(QGraphicsSceneHoverEvent * event)
{
  if (edit_mode_)
  {
    QPointF corner, pivot_corner;
    int corner_index = get_editor_corner(event->pos(), corner, pivot_corner);
    QPointF center;
    int center_index = get_editor_center(event->pos(), center);
    if (corner_index != -1)
    {
      if (corner_index == 0 || corner_index == 2)
      {
        setCursor(Qt::SizeFDiagCursor);
      }
      else
      {
        setCursor(Qt::SizeBDiagCursor);
      }
    }
    else if (center_index != -1)
    {
      if (center_index == 0 || center_index == 2)
      {
        setCursor(Qt::SizeHorCursor);
      }
      else
      {
        setCursor(Qt::SizeVerCursor);
      }
    }
    else
    {
      setCursor(Qt::SizeAllCursor);
    }
  }
}

void Node::contextMenuEvent(QGraphicsSceneContextMenuEvent *event)
{
  //if(!isSelected()) return;

  QMenu menu;
  QAction *remove_action = menu.addAction(tr("Remove"));
  //QAction *markAction = menu.addAction("Mark");
  QAction *selected_action = menu.exec(event->screenPos());

  ve::Scene* scene = qobject_cast<ve::Scene*>(this->scene());
  if (selected_action == remove_action)
  {
    qDebug() << "remove:" << objectName();
    //remove input link
    if (link_in_)
      link_in_->prepare_for_delete();

    //remove output link
    if (link_out_)
      link_out_->prepare_for_delete();

    Q_EMIT scene->signal_node_deleted(this);
  }
}

QVariant Node::itemChange(GraphicsItemChange change, const QVariant &value)
{
  ve::Scene* scene = qobject_cast<ve::Scene*>(this->scene());
  switch (change)
  {
    //case Item
    case ItemPositionHasChanged:
    {
      //Q_EMIT scene->slot_object_selected(this);
      adjust_links();
      Q_EMIT scene->signal_node_position_changed(this);
      break;
    }

    case ItemSelectedChange:
    {
      if (value.toBool())
      {
        set_edit_mode(true);
        Q_EMIT scene->signal_object_selected(this);
      }
      else
      {
        set_edit_mode(false);
        setCursor(Qt::ArrowCursor);
        Q_EMIT scene->signal_object_deselected();
      }
      break;
    }
    default:
      break;
  };

  return QGraphicsItem::itemChange(change, value);
}
