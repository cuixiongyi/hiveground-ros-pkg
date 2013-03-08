#include <ve_link.h>
#include <ve_node.h>
#include <ve_scene.h>

#include <QtGui>

using namespace ve;

Scene::Scene(QWidget *parent) :
    QGraphicsScene(parent), mode_(MODE_CURSOR), link_temp_(0), line_temp_(0)
{

}

Scene::~Scene()
{

}

void Scene::set_mode(int mode)
{
  clear_added_mesurement_items();
  mode_ = mode;
}

void Scene::clear_added_mesurement_items()
{
  Q_FOREACH(QGraphicsItem* item, added_mesurement_items_)
  {
    this->removeItem(item);
  }
  added_mesurement_items_.clear();
}

Node* Scene::node_at(const QPointF& position)
{
  QGraphicsItem* item = itemAt(position);
  if (!item)
    return 0;

  Node* node = qgraphicsitem_cast<Node*>(item);
  if (!node)
    return 0;

  return node;
}

Link* Scene::link_at(const QPointF& position)
{
  QGraphicsItem* item = itemAt(position);
   if (!item)
    return 0;

  Link* link = qgraphicsitem_cast<Link*>(item);
  if (!link)
    return 0;

  return link;
}

void Scene::mousePressEvent(QGraphicsSceneMouseEvent* event)
{
  bool handle = false;
  Node* node = node_at(event->scenePos());
  Link* link = link_at(event->scenePos());

  if (mode_ == MODE_MEASURE_LENGTH)
  {
    handle = true;

    if (measurement_points_.empty() || measurement_points_.size() >= 2)
    {
      clear_added_mesurement_items();

      measurement_points_.clear();
      measurement_points_.push_back(event->scenePos());
      QGraphicsEllipseItem* p = this->addEllipse(event->scenePos().x() - 1.5, event->scenePos().y() - 1.5, 3.0, 3.0,
                                                 QPen(Qt::red, 1));
      added_mesurement_items_.push_back(p);
    }
    else
    {
      measurement_points_.push_back(event->scenePos());
      QGraphicsEllipseItem* p = this->addEllipse(event->scenePos().x() - 1.5, event->scenePos().y() - 1.5, 3.0, 3.0,
                                                 QPen(Qt::red, 1));
      added_mesurement_items_.push_back(p);
      if (measurement_points_.size() == 2)
      {
        qDebug() << measurement_points_[0] << measurement_points_[1];
        QGraphicsLineItem* l1 = this->addLine(measurement_points_[0].x(), measurement_points_[0].y(),
                                              measurement_points_[1].x(), measurement_points_[1].y(),
                                              QPen(Qt::cyan, 1));
        added_mesurement_items_.push_back(l1);

        float dx = measurement_points_[0].x() - measurement_points_[1].x();
        float dy = measurement_points_[0].y() - measurement_points_[1].y();
        float dist = sqrt(dx * dx + dy * dy);

        QFont sansFont("Helvetica [Cronyx]", 8, QFont::Black);
        QGraphicsSimpleTextItem* text1 = this->addSimpleText(QString("dist: %1 px").arg(dist, 0, 'f', 2), sansFont);
        text1->setBrush(QBrush(Qt::black));
        text1->setPen(QPen(Qt::white, 0.3));
        text1->setPos(measurement_points_[1].x(), measurement_points_[1].y());
        added_mesurement_items_.push_back(text1);

      }
    }
  }
  else if (node)
  {
    if (mode_ == MODE_ADD_LINK)
    {
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
    //qDebug() << "link selected:" << (int)link;
  }
  else
  {
    //open area
    if (mode_ == MODE_ADD_NODE)
    {
      handle = true;
      if (event->buttons() == Qt::LeftButton)
      {
        //qDebug() << "add new node";
        Q_EMIT signal_node_added(event->scenePos());
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
          Q_EMIT signal_nodes_linked(node_out_temp_, node);
      }
    }

  }
  if (!handle)
    QGraphicsScene::mouseReleaseEvent(event);
}
