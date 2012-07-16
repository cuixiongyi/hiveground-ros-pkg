#include <ros/ros.h>
#include <prw_object_tracking/object_tracking.h>
#include <QtGui/QApplication>
#include <QDebug>

#include <boost/thread.hpp>

using namespace prw;

ObjectTracking::ObjectTracking(QWidget *parent, Qt::WFlags flags) :
    QMainWindow(parent, flags),
    quit_threads_(false)
{
  ui.setupUi(this);
}

ObjectTracking::~ObjectTracking()
{
}




ObjectTracking* object_tracking_ = NULL;
bool initialized_ = false;

void spin_function()
{
  ros::WallRate r(100.0);
  while (ros::ok() && !initialized_)
  {
    r.sleep();
    ros::spinOnce();
  }
  while (ros::ok() && !object_tracking_->quit_threads_)
  {
    r.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "prw_object_tracking", ros::init_options::NoSigintHandler);

  boost::thread spin_thread(boost::bind(&spin_function));

  QApplication a(argc, argv);



  ObjectTracking w;
  object_tracking_ = &w;
  w.show();

  initialized_ = true;

  int ret = a.exec();

  spin_thread.join();

  return ret;
}
