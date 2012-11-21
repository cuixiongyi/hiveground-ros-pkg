#ifndef RPW_USER_TRACKING_H
#define RPW_USER_TRACKING_H

#include "ui_user_tracking.h"

namespace prw
{


class UserTracking : public QMainWindow
{
  Q_OBJECT
public:

  UserTracking(QWidget *parent = 0, Qt::WFlags flags = 0);
  ~UserTracking();


public slots:
protected:

public:
  Ui::UserTracking ui;
  bool quit_threads_;
};

}


#endif
