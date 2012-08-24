#ifndef VELIB_VIEW_H
#define VELIB_VIEW_H

#include <velib/velib_global.h>

namespace ve
{

class VELIB_EXPORT View : public QGraphicsView
{
    Q_OBJECT
public:
    View(QWidget *parent = 0);
    virtual ~View();     

    void itemMoved();

protected:
  void keyPressEvent(QKeyEvent *event);
  void wheelEvent(QWheelEvent *event);
  void drawBackground(QPainter *painter, const QRectF &rect);

  void scaleView(qreal scale_factor);

private:

};

}

#endif
