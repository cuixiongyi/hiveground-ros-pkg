#include <ros/ros.h>

#include <qmainwindow.h>
#include "ui_les.h"

class LES : public QMainWindow
{
  Q_OBJECT
public:
  LES(ros::NodeHandle& nh, QWidget *parent = 0, Qt::WFlags flags = 0);
  ~LES();
public:
  ros::NodeHandle& nh_;
  bool quit_threads_;

private:
  Ui::LesUi ui;

};
