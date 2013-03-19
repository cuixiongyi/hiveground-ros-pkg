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

#ifndef VELIB_SCENE_H
#define VELIB_SCENE_H

#include <QtGui>

namespace ve
{

class Node;
class Link;

class Scene : public QGraphicsScene
{
  Q_OBJECT
public:
  enum
  {
    MODE_CURSOR,
    MODE_ADD_NODE,
    MODE_ADD_LINK,
    MODE_MEASURE_LENGTH
  };

  Scene(QWidget *parent = 0);
  virtual ~Scene();

  void set_mode(int mode);

  //public signal interface
  void emit_signal_node_deleted(QObject* deleted_node)
  {
    Q_EMIT signal_node_deleted(deleted_node);
  }

  void clear_added_mesurement_items();

Q_SIGNALS:
  void signal_object_selected(QObject* object);
  void signal_object_deselected();
  void signal_node_added(QPointF scene_position);
  void signal_nodes_linked(QObject* node_out, QObject* node_in);
  void signal_node_deleted(QObject* deleted_node);
  void signal_link_deleted(QObject* deleted_link);

  void signal_node_position_changed(QObject* node);
  void signal_node_size_changed(QObject* node);

  void signal_node_mouse_hover_enter(QObject* node);
  void signal_node_mouse_hover_leave(QObject* node);

protected:
  Node* node_at(const QPointF& position);
  Link* link_at(const QPointF& position);
  //bool handle_node_select(QGraphicsSceneMouseEvent* event, bool press);
  //bool handleLinkSelect(QGraphicsSceneMouseEvent* event);

protected:
  void mousePressEvent(QGraphicsSceneMouseEvent* event);
  void mouseMoveEvent(QGraphicsSceneMouseEvent* event);
  void mouseReleaseEvent(QGraphicsSceneMouseEvent* event);

public:
  friend class Node;
  friend class Link;

private:
  int mode_;
  Link* link_temp_;
  QGraphicsLineItem *line_temp_;
  Node* node_out_temp_;
  QVector<QPointF> measurement_points_;
  QVector<QGraphicsItem*> added_mesurement_items_;
};

}

#endif
