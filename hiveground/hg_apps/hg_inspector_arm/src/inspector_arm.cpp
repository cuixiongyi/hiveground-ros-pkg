/*
 * inspector_arm.cpp
 *
 *  Created on: Jan 7, 2013
 *      Author: mahisorn
 */

#include <QtGui/QApplication>
#include <qevent.h>


#include <ros/ros.h>
#include <hg_inspector_arm/inspector_arm.h>
#include <boost/thread.hpp>

InspectorArm::InspectorArm(QWidget *parent, Qt::WFlags flags)
  : QMainWindow(parent, flags),
    quit_thread_(false),
    nh_(), nh_private_("~")
{
  ui.setupUi(this);
}

InspectorArm::~InspectorArm()
{

}

bool InspectorArm::initialize()
{
  return true;
}

void InspectorArm::closeEvent(QCloseEvent *event)
{
  quit_thread_ = true;
  event->accept();
}

InspectorArm* g_inspector_arm = NULL;
bool g_initialized = false;

void spin_function()
{
  ros::WallRate r(100.0);
  while (ros::ok() && !g_initialized)
  {
    r.sleep();
    ros::spinOnce();
  }
  while (ros::ok() && !g_inspector_arm->quit_thread_)
  {
    r.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hg_inspection_arm", ros::init_options::NoSigintHandler);
  boost::thread spin_thread(boost::bind(&spin_function));

  QApplication a(argc, argv);

  InspectorArm arm;
  g_inspector_arm = &arm;
  arm.show();

  g_initialized = arm.initialize();

  if(!g_initialized)
    exit(-1);

  int ret = a.exec();

  spin_thread.join();

  return ret;
}

