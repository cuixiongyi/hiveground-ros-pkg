#include "kinect_server.h"
#include <QtGui/QApplication>
#include <qdebug.h>

#include <ros/ros.h>
#include <boost/thread.hpp>

kinect_server* g_kinect_server = NULL;
bool g_initialized = false;

void spin_function()
{
  ros::WallRate r(100.0);
  while (ros::ok() && !g_initialized)
  {
    r.sleep();
    ros::spinOnce();
  }
  while (ros::ok() && !g_kinect_server->quit_threads_)
  {
    r.sleep();
    ros::spinOnce(); 
  }
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "kinect_server", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");
  boost::thread spin_thread(boost::bind(&spin_function));

	QApplication a(argc, argv);
	kinect_server w(nh);
	w.show();

  g_kinect_server = &w;

  g_initialized = true;

  int ret = a.exec();

  spin_thread.join();

	return ret;
}
