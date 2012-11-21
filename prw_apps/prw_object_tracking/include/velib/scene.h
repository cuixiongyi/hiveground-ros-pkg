#ifndef VELIB_SCENE_H
#define VELIB_SCENE_H

#include <velib/velib_global.h>

namespace ve
{

class Node;
class Link;

class VELIB_EXPORT Scene : public QGraphicsScene
{
Q_OBJECT
public:
  enum
  {
    MODE_CURSOR, MODE_ADD_NODE, MODE_ADD_LINK
  };

  Scene(QWidget *parent = 0);
  virtual ~Scene();

  void set_mode(int mode)
  {
    mode_ = mode;
  }

  //public signal interface
  void emit_signal_node_deleted(QObject* deleted_node)
  {
    Q_EMIT signal_node_deleted(deleted_node);
  }

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
};

}

#endif
