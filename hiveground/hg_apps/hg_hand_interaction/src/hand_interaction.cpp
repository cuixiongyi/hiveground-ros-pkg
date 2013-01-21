/*
 * hand_interaction.cpp
 *
 *  Created on: Jan 21, 2013
 *      Author: mahisorn
 */

#include <ros/ros.h>
#include <QtGui/QApplication>
#include <QDebug>
#include <boost/thread.hpp>
#include <hg_hand_interaction/hand_interaction.h>


using namespace hg_hand_interaction;

HandInteraction::HandInteraction(QWidget *parent, Qt::WFlags flags) :
  QMainWindow(parent, flags),
  nh_(), nh_private_("~"),
  quit_threads_(false)
{

}

HandInteraction::~HandInteraction()
{

}

bool HandInteraction::initialize()
{
  hands_subscriber_ = nh_private_.subscribe("hands_in", 1, &HandInteraction::handsCallBack, this);
  one_hand_gesture_publisher_ = nh_private_.advertise<OneHandGesture>("one_hand_gesture", 128);
  two_hand_gesture_publisher_ = nh_private_.advertise<TwoHandGesture>("two_hand_gesture", 128);
  return true;
}


void HandInteraction::handsCallBack(const hg_object_tracking::HandsConstPtr message)
{
  ROS_INFO("%s: %lu", __FUNCTION__, message->hands.size());
}

HandInteraction* g_hand_interaction = NULL;
bool g_initialized = false;

void spin_function()
{
  ros::WallRate r(100.0);
  while (ros::ok() && !g_initialized)
  {
    r.sleep();
    ros::spinOnce();
  }
  while (ros::ok() && !g_hand_interaction->quit_threads_)
  {
    r.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "prw_hand_tracking", ros::init_options::NoSigintHandler);

  boost::thread spin_thread(boost::bind(&spin_function));

  QApplication a(argc, argv);



  HandInteraction w;
  g_hand_interaction = &w;
  w.show();

  g_initialized = w.initialize();
  if(!g_initialized)
    exit(-1);

  int ret = a.exec();

  spin_thread.join();

  return ret;
}
