#include <ros/ros.h>

#include <boost/thread.hpp>

#include <QtGui/QApplication>

#include <les/les.h>


LES* g_les = NULL;
bool g_initialized = false;

void spin_function()
{
  ros::WallRate r(100.0);
  while (ros::ok() && !g_initialized)
  {
    r.sleep();
    ros::spinOnce();
  }
  while (ros::ok() && !g_les->quit_threads_)
  {
    r.sleep();
    ros::spinOnce();
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "prw_object_tracking", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");

  boost::thread spin_thread(boost::bind(&spin_function));

  QApplication a(argc, argv);

  LES w(nh);
  g_les = &w;
  w.show();

  g_initialized = true;

  int ret = a.exec();

  spin_thread.join();

  return ret;
}
