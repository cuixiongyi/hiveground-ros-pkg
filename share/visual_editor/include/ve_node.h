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

#ifndef VELIB_NODE_H
#define VELIB_NODE_H

#include <ve_global.h>
#include <QtGui>

namespace ve
{

class Link;

class Node : public QObject, public QGraphicsItem
{
  Q_OBJECT
  Q_PROPERTY(QRectF SceneROI READ scene_roi WRITE set_scene_roi)
  Q_INTERFACES(QGraphicsItem)

public:
  static const int RESIZE_CORNER_MARK_SIZE = 3;
  static const int RESIZE_CENTER_MARK_SIZE = 2;
  static const int CENTER_RADIUS = 2;

  Node();
  Node(const QRectF& rect);
  virtual ~Node();

  enum
  {
    TYPE = (UserType + VE_TYPE + 4096)
  };
  int type() const
  {
    return TYPE;
  }

  QRectF scene_roi() const
  {
    return this->mapRectToScene(node_rect_);
  }
  QRectF get_node_rect() const
  {
    return node_rect_;
  }
  void set_edit_mode(bool edit_mode);
  void set_link_in(Link* link_in)
  {
    link_in_ = link_in;
  }
  void set_link_out(Link* link_out)
  {
    link_out_ = link_out;
  }

  //virtual int input_link_count()  const { return 1; }
  //virtual int output_link_count()  const { return 1; }
  //virtual QRectF input_link_rect(int line) const;
  //virtual QRectF output_link_rect(int line) const;

public Q_SLOTS:
  //void set_name(const QString& name) { setObjectName(name); update(); }
  void set_scene_roi(const QRectF& scene_roi)
  {}

protected:
  void initialize();
  void draw_editor(QPainter *painter);
  int get_editor_corner(const QPointF& position, QPointF& corner, QPointF& pivot_corner);
  int get_editor_center(const QPointF& position, QPointF& center);
  QRectF get_rect_from_two_points(QPointF p1, QPointF p2);
  void adjust_links();

  void remove_input_link();
  void remove_output_link();

protected:
  QRectF boundingRect() const;
  QPainterPath shape() const;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

  void mousePressEvent(QGraphicsSceneMouseEvent *event);
  void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
  void mouseMoveEvent(QGraphicsSceneMouseEvent *event);

  void hoverEnterEvent(QGraphicsSceneHoverEvent *event);
  void hoverLeaveEvent(QGraphicsSceneHoverEvent *event);
  void hoverMoveEvent(QGraphicsSceneHoverEvent *event);

  void contextMenuEvent(QGraphicsSceneContextMenuEvent *event);

  QVariant itemChange(GraphicsItemChange change, const QVariant &value);

protected:
  friend class Link;
  friend class Scene;

  QPointF new_point_;
  QPointF mouse_point_;
  QRectF node_rect_;

  bool edit_mode_;
  bool corner_resize_;
  int corner_index_;
  QPointF pivot_corner_;
  QPointF corner_;
  QPointF corner_temp_;

  bool center_resize_;
  int center_index_;
  QPointF center_;

  Link* link_in_;
  Link* link_out_;

  QMutex mutex_;
};

}

#endif