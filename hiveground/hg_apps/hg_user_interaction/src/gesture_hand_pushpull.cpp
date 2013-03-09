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
 *       File: gesture_hand_pushpull.cpp
 * Created on: Mar 9, 2013
 *     Author: Mahisorn Wongphati
 */

#include <hg_user_interaction/gesture_hand_pushpull.h>

using namespace hg_user_interaction;

int GestureDetectorHandPushPull::RTTI = Rtti_HandPushPull;

GestureDetectorHandPushPull::GestureDetectorHandPushPull(ros::NodeHandle& nh_private)
 : GestureDetectorItem(nh_private)
{

}

GestureDetectorHandPushPull::GestureDetectorHandPushPull(ros::NodeHandle& nh_private, const QRectF& rect)
 : GestureDetectorItem(nh_private, rect)
{

}

GestureDetectorHandPushPull::~GestureDetectorHandPushPull()
{
  //ROS_INFO("haha");
}

bool GestureDetectorHandPushPull::initialize()
{
  current_state_[0] = current_state_[1] = IDEL;
  three_axes_[0] = tf::Vector3(1, 0, 0); //X
  three_axes_[1] = tf::Vector3(0, 1, 0); //Y
  three_axes_[2] = tf::Vector3(0, 0, 1); //Z

  nh_private_.getParam("push_pull_gesture_draw_history", draw_history_);
  nh_private_.getParam("push_pull_gesture_draw_result", draw_result_);

  nh_private_.getParam("push_pull_gesture_r1", r1_);
  nh_private_.getParam("push_pull_gesture_r2", r2_);
  nh_private_.getParam("push_pull_gesture_r3", r3_);

  nh_private_.getParam("push_pull_gesture_timeout", time_out_);
  nh_private_.getParam("push_pull_gesture_activating_time", activating_time_);


  return true;
}

void GestureDetectorHandPushPull::addHandsMessage(const hg_object_tracking::HandsConstPtr& hands)
{

}

void GestureDetectorHandPushPull::drawHistory(visualization_msgs::MarkerArray& marker_array)
{

}

void GestureDetectorHandPushPull::drawResult(visualization_msgs::MarkerArray& marker_array)
{

}

int GestureDetectorHandPushPull::lookForGesture()
{
  return 0;
}

