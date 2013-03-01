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


#include <hg_user_interaction/main.h>
#include <boost/thread.hpp>

using namespace hg_user_interaction;
using namespace message_filters;

UserInteraction::UserInteraction(QWidget *parent, Qt::WFlags flags)
  : QMainWindow(parent, flags),
    nh_(), nh_private_("~"),
    quit_threads_(false)
{

}

UserInteraction::~UserInteraction()
{

}

bool UserInteraction::initialize()
{
  ROS_INFO(__FUNCTION__);

  nh_private_.getParam("skeletons_topic", skeletons_topic_);
  nh_private_.getParam("hands_topic", hands_topic_);
  ROS_INFO_STREAM(skeletons_topic_ << ":" << hands_topic_);

  skeletons_sub_ = boost::shared_ptr<message_filters::Subscriber<kinect_msgs::Skeletons> >(new message_filters::Subscriber<kinect_msgs::Skeletons>(nh_, skeletons_topic_, 1));
  hands_sub_ = boost::shared_ptr<message_filters::Subscriber<hg_object_tracking::Hands> >(new message_filters::Subscriber<hg_object_tracking::Hands>(nh_, hands_topic_, 1));

  skeltons_hands_sync_ = boost::shared_ptr<message_filters::Synchronizer<SkeltonsHandsSyncPolicy> >(new message_filters::Synchronizer<SkeltonsHandsSyncPolicy>(SkeltonsHandsSyncPolicy(100), *skeletons_sub_, *hands_sub_));
  skeltons_hands_sync_->registerCallback(boost::bind(&UserInteraction::handSkeletonCallback, this, _1, _2));

  return true;
}


void UserInteraction::handSkeletonCallback(const kinect_msgs::SkeletonsConstPtr& skeletons, const hg_object_tracking::HandsConstPtr& hands)
{
  ROS_INFO_THROTTLE(1.0, __FUNCTION__);
  //ROS_INFO_STREAM(skeletons->header.stamp);
  //ROS_INFO_STREAM(hands->header.stamp);






}



UserInteraction* g_handle = NULL;
bool g_initialized = false;

void spin_function()
{
  ros::WallRate r(100.0);
  while (ros::ok() && !g_initialized)
  {
    r.sleep();
    ros::spinOnce();
  }
  while (ros::ok() && !g_handle->quit_threads_)
  {
    r.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hg_kinect_interaction", ros::init_options::NoSigintHandler);

  boost::thread spin_thread(boost::bind(&spin_function));

  QApplication a(argc, argv);



  UserInteraction w;
  g_handle = &w;
  w.show();

  g_initialized = w.initialize();
  if(!g_initialized)
    exit(-1);

  int ret = a.exec();

  spin_thread.join();

  return ret;
}
