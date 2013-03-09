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

#include <ve_view.h>

using namespace ve;

View::View(QWidget *parent) :
    QGraphicsView(parent)
{
  //qDebug() << __FUNCTION__;
  setCacheMode(CacheBackground);
  setViewportUpdateMode(BoundingRectViewportUpdate);
  setRenderHint(QPainter::Antialiasing);
  setTransformationAnchor(AnchorUnderMouse);
  //scale(qreal(0.5), qreal(0.5));
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
