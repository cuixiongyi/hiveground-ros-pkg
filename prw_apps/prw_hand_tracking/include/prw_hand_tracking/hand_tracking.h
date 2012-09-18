#ifndef RPW_HAND_TRACKING_H
#define RPW_HAND_TRACKING_H

#include "ui_hand_tracking.h"

namespace prw
{


class HandTracking : public QMainWindow
{
  Q_OBJECT
public:

  HandTracking(QWidget *parent = 0, Qt::WFlags flags = 0);
  ~HandTracking();


public slots:
protected:

public:
  Ui::HandTracking ui;
  bool quit_threads_;
};

}


#endif
