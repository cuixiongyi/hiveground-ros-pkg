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
 *       File: gesture_body_movement.cpp
 * Created on: Mar 11, 2013
 *     Author: Mahisorn Wongphati
 */

#include <hg_user_interaction/gesture_body_movement.h>
#include <kinect_msgs/Skeletons.h>

using namespace hg_user_interaction;
using namespace visualization_msgs;
using namespace kinect_msgs;

int GestureBodyMovement::RTTI = Rtti_BodyMovement;


GestureBodyMovement::GestureBodyMovement(ros::NodeHandle& nh_private)
 : GestureDetectorItem(nh_private)
{

}

GestureBodyMovement::GestureBodyMovement(ros::NodeHandle& nh_private, const QRectF& rect)
 : GestureDetectorItem(nh_private, rect)
{

}

GestureBodyMovement::~GestureBodyMovement()
{

}

bool GestureBodyMovement::initialize()
{
  return true;
}

void GestureBodyMovement::addSkeletonsMessage(const kinect_msgs::SkeletonsConstPtr& skeletons)
{
  ROS_INFO_THROTTLE(1.0, __FUNCTION__);

  for (int i = 0; i < Skeletons::SKELETON_COUNT; i++)
  {
    if (skeletons->skeletons[i].skeleton_tracking_state == Skeleton::SKELETON_TRACKED)
    {
      ROS_INFO_STREAM("[" << i << "] " << skeletons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_SHOULDER_CENTER]);
    }
  }



}

void GestureBodyMovement::addHandsMessage(const hg_object_tracking::HandsConstPtr& hands)
{

}

void GestureBodyMovement::drawHistory(visualization_msgs::MarkerArray& marker_array, const std::string& frame_id)
{
  if(!getDrawHistory()) return;
}

void GestureBodyMovement::drawResult(visualization_msgs::MarkerArray& marker_array, const std::string& frame_id)
{
  if(!getDrawResult()) return;
}

int GestureBodyMovement::lookForGesture(hg_user_interaction::Gesture& gesture)
{
  return Gesture::GESTURE_NOT_DETECTED;
}
