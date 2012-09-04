#include <velib/scene.h>
#include <velib/node.h>
#include <velib/link.h>

#include <QDebug>

using namespace ve;

Scene::Scene(QWidget *parent) :
    QGraphicsScene(parent), mode_(MODE_CURSOR), link_temp_(0), line_temp_(0)
{
  //qDebug() << __FUNCTION__;
}

Scene::~Scene()
{
  //qDebug() << __FUNCTION__;
}

Node* Scene::node_at(const QPointF& position)
{
  //qDebug() << __FUNCTION__;
  QGraphicsItem* item = itemAt(position);
  //qDebug() << item;
  if (!item)
    return 0;

  Node* node = qgraphicsitem_cast<Node*>(item);
  if (!node)
    return 0;

  //QPointF position_on_node = item->mapFromScene(position);
  return node;
}

Link* Scene::link_at(const QPointF& position)
{
  //qDebug() << __FUNCTION__;
  QGraphicsItem* item = itemAt(position);
  //qDebug() << item;
  if (!item)
    return 0;

  Link* link = qgraphicsitem_cast<Link*>(item);
  if (!link)
    return 0;

  //QPointF position_on_link = item->mapFromScene(position);
  return link;
}

void Scene::mousePressEvent(QGraphicsSceneMouseEvent* event)
{
  bool handle = false;
  Node* node = node_at(event->scenePos());
  Link* link = link_at(event->scenePos());

  if (node)
  {
    if (mode_ == MODE_ADD_LINK)
    {
      //handle = true;
      if (event->buttons() == Qt::LeftButton)
      {
        handle = true;
        if (node->link_out_ == 0)
        {
          if (link_temp_)
          {
            qDebug() << "OLD link exist !!!!";
            delete link_temp_;
            link_temp_ = 0;
          }

          if (line_temp_)
          {
            qDebug() << "OLD line exist !!!!";
            delete line_temp_;
            link_temp_ = 0;
          }

          line_temp_ = new QGraphicsLineItem(0, this);
          line_temp_->setZValue(3.0f);
          line_temp_->setPen(QPen(Qt::blue, 2.0f));

          QLineF line(node->mapToScene(node->get_node_rect().center()), event->scenePos());

          line_temp_->setLine(line);
          line_temp_->show();

          node_out_temp_ = node;
        }
      } //if event
    } //if mode
    else if (mode_ == MODE_ADD_NODE)
    {
      //add only
      handle = true;
    }
  }
  else if (link)
  {

  }
  else
  {
    //open area
    if (mode_ == MODE_ADD_NODE)
    {
      handle = true;
      if (event->buttons() == Qt::LeftButton)
      {
        qDebug() << "add new node";
        emit signal_node_added(event->scenePos());
      }
    }

  }

  if (!handle)
    QGraphicsScene::mousePressEvent(event);
}

void Scene::mouseMoveEvent(QGraphicsSceneMouseEvent* event)
{
  //qDebug() << __FUNCTION__;
  bool handle = false;
  if (line_temp_)
  {
    if (event->buttons() == Qt::LeftButton)
    {
      QLineF line = line_temp_->line();
      line_temp_->setLine(QLineF(line.p1(), event->scenePos()));
    }
    else
    {
      //qDebug() << "delete line temp";
      delete line_temp_;
      line_temp_ = 0;
    }
    handle = true;
  }

  if (!handle)
    QGraphicsScene::mouseMoveEvent(event);
}

void Scene::mouseReleaseEvent(QGraphicsSceneMouseEvent* event)
{
  //qDebug() << __FUNCTION__ << event->scenePos();
  bool handle = false;

  if (line_temp_)
  {
    delete line_temp_;
    line_temp_ = 0;

    Node* node = node_at(event->scenePos());
    if (node)
    {
      //qDebug() << "release on node:" << (int)node;
      if (event->buttons() == !Qt::LeftButton)
      {
        if (node != node_out_temp_)
          emit signal_nodes_linked(node_out_temp_, node);
      }
    }

  }
  if (!handle)
    QGraphicsScene::mouseReleaseEvent(event);
}
