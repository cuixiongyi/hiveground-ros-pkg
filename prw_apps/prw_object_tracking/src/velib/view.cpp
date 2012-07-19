#include <velib/view.h>
#include <QDebug>
#include <cmath>
#include <QScrollBar>

using namespace ve;

View::View(QWidget *parent) :
    QGraphicsView(parent)
{
  setCacheMode(CacheBackground);
  setViewportUpdateMode(BoundingRectViewportUpdate);
  setRenderHint(QPainter::Antialiasing);
  setTransformationAnchor(AnchorUnderMouse);

}

View::~View()
{
  //qDebug() << __FUNCTION__;
}

void View::keyPressEvent(QKeyEvent *event)
{
  //qDebug() << __FUNCTION__;
  //qDebug() << event->key();
  switch (event->key())
  {
    case Qt::Key_Up:
    {
      verticalScrollBar()->setValue(verticalScrollBar()->value() - 5);
      break;
    }
    case Qt::Key_Down:
    {
      verticalScrollBar()->setValue(verticalScrollBar()->value() + 5);
      break;
    }
    case Qt::Key_Left:
    {
      horizontalScrollBar()->setValue(horizontalScrollBar()->value() - 5);
      break;
    }
    case Qt::Key_Right:
    {
      horizontalScrollBar()->setValue(horizontalScrollBar()->value() + 5);
      break;
    }
  }

}

void View::wheelEvent(QWheelEvent *event)
{
  if (event->modifiers() == Qt::CTRL)
  {
    scaleView(pow((double)2, -event->delta() / 240.0));
  }
}

void View::drawBackground(QPainter *painter, const QRectF &rect)
{
  if (!isEnabled())
    return;

  QMatrix matrix = painter->matrix();
  double distance = 10.0f; // 10 pixels distance between grid points.

  int gridXCount = int(sceneRect().width() / distance) + 1;
  int gridYCount = int(sceneRect().height() / distance) + 1;

  // Figure out the part of the grid that actually needs to
  // be redrawn.
  int gridXStart = int((rect.left() - sceneRect().left()) / distance);
  int gridYStart = int((rect.top() - sceneRect().top()) / distance);

  if (gridXStart < 0)
    gridXStart = 0;
  if (gridYStart < 0)
    gridYStart = 0;

  painter->fillRect(sceneRect(), Qt::white);

  painter->save();
  painter->resetMatrix();

  double x = sceneRect().left();
  double y = sceneRect().top();

  for (int i = gridXStart; i < gridXCount; i++)
  {
    for (int j = gridYStart; j < gridYCount; j++)
    {
      x = sceneRect().left() + double(i) * distance;
      y = sceneRect().top() + double(j) * distance;
      painter->drawPoint(matrix.map(QPointF(x, y)));
    }
  }

  painter->restore();

  painter->drawRect(sceneRect());
}

void View::scaleView(qreal scale_factor)
{
  qreal factor = transform().scale(scale_factor, scale_factor).mapRect(QRectF(0, 0, 1, 1)).width();
  if (factor < 0.07 || factor > 100)
    return;
  scale(scale_factor, scale_factor);
}
