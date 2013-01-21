/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Imai Laboratory, Keio University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Imai Laboratory, nor the name of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mahisorn Wongphati
 */

#include <ros/ros.h>
#include <QtGui/QApplication>
#include <QDebug>
#include <boost/thread.hpp>
#include <hg_hand_interaction/hand_interaction.h>


using namespace hg_hand_interaction;
using namespace visualization_msgs;

HandInteraction::HandInteraction(QWidget *parent, Qt::WFlags flags) :
  QMainWindow(parent, flags),
  nh_(), nh_private_("~"),
  quit_threads_(false)
{
  ui.setupUi(this);

  ui.toolBox->removeItem(ui.toolBox->currentIndex());
}

HandInteraction::~HandInteraction()
{
  if(gesture_detectors_.size() != 0)
  {
    for(size_t i = 0; i < gesture_detectors_.size(); i++)
    {
      if(gesture_detectors_[i])
        delete gesture_detectors_[i];
    }
  }
}

bool HandInteraction::initialize()
{
  gesture_detectors_.push_back(new SweepHandGestureDetector(nh_private_));




  //UI for each detector if any
  for(size_t i = 0; i < gesture_detectors_.size(); i++)
  {
    gesture_detectors_[i]->addUI(ui.toolBox);
    gesture_detectors_[i]->initialize();

  }



  hands_subscriber_ = nh_private_.subscribe("hands_in", 1, &HandInteraction::handsCallBack, this);
  hand_gesture_publisher_ = nh_private_.advertise<HandGesture>("hand_gesture", 128);
  marker_array_publisher_ = nh_private_.advertise<MarkerArray>("gesture_history", 128);
  return true;
}


void HandInteraction::handsCallBack(const hg_object_tracking::HandsConstPtr message)
{
  HandGestureDetector* hand_gesture_detector;
  MarkerArray marker_array;
  for (size_t i = 0; i < gesture_detectors_.size(); i++)
  {
    hand_gesture_detector = dynamic_cast<HandGestureDetector*>(gesture_detectors_[i]);
    if(hand_gesture_detector)
    {
      hand_gesture_detector->addHandMessage(message);
      if(hand_gesture_detector->lookForGesture())
      {

      }
    }

    if(ui.checkBoxShowHistory->isChecked())
    {
      gesture_detectors_[i]->drawHistory(marker_array, message->header.frame_id);
    }
  }

  if(ui.checkBoxShowHistory->isChecked())
  {
    if(marker_array_publisher_.getNumSubscribers() != 0)
    {
      marker_array_publisher_.publish(marker_array);
    }
  }

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
