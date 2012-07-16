#ifndef RPW_OBJECT_TRACKING_H
#define RPW_OBJECT_TRACKING_H

#include "ui_object_tracking.h"

namespace prw
{


class ObjectTracking : public QMainWindow
{
  Q_OBJECT
public:

  ObjectTracking(QWidget *parent = 0, Qt::WFlags flags = 0);
  ~ObjectTracking();


public slots:
protected:

public:
  Ui::ObjectTracking ui;
  bool quit_threads_;
};

}


#endif
