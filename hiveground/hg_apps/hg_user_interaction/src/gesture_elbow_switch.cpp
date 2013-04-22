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
 * gesture_elbow_switch.cpp
 *
 *  Created on: Apr 12, 2013
 *      Author: chang
 */

#include <hg_user_interaction/gesture_elbow_switch.h>
#include <kinect_msgs/Skeletons.h>

using namespace hg_user_interaction;
using namespace visualization_msgs;
using namespace kinect_msgs;

int GestureDetectorElbowSwitch::RTTI = Rtti_ElbowSwitch;


GestureDetectorElbowSwitch::GestureDetectorElbowSwitch(ros::NodeHandle& nh_private)
 : GestureDetectorItem(nh_private)
{

}

GestureDetectorElbowSwitch::GestureDetectorElbowSwitch(ros::NodeHandle& nh_private, const QRectF& rect)
 : GestureDetectorItem(nh_private, rect)
{

}

GestureDetectorElbowSwitch::~GestureDetectorElbowSwitch()
{

}

bool GestureDetectorElbowSwitch::initialize()
{
  skeleton_updated_ = false;

  nh_private_.getParam("elbow_switch_gesture_max_frame", max_frame_);
  nh_private_.getParam("elbow_switch_gesture_min_length", min_length_);
  nh_private_.getParam("elbow_switch_gesture_max_detecting_rate", max_detecting_rate_);


  return true;
}

void GestureDetectorElbowSwitch::addSkeletonsMessage(const kinect_msgs::SkeletonsConstPtr& skeletons)
{
  //ROS_INFO_THROTTLE(1.0, __FUNCTION__);
  tf::Transform tf0, tf1, tf2;
  for (int i = 0; i < Skeletons::SKELETON_COUNT; i++)
  {
    if (skeletons->skeletons[i].skeleton_tracking_state == Skeleton::SKELETON_TRACKED)
    {
      tf::transformMsgToTF(skeletons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_ELBOW_LEFT], tf1);
      l_elbow_path_.push_back(tf1.getOrigin());

      tf::transformMsgToTF(skeletons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_ELBOW_RIGHT], tf1);
      r_elbow_path_.push_back(tf1.getOrigin());

      if(l_elbow_path_.size() > max_frame_) l_elbow_path_.pop_front();
      if(r_elbow_path_.size() > max_frame_) r_elbow_path_.pop_front();
    }
  }
}

void GestureDetectorElbowSwitch::drawHistory(visualization_msgs::MarkerArray& marker_array, const std::string& frame_id)
{
  if(!getDrawHistory()) return;

  Marker marker;
  marker.lifetime = ros::Duration(0.1);
  marker.header.frame_id = frame_id;
  marker.ns = "GestureDetectorElbowSwitch_history";
  marker.type = Marker::LINE_STRIP;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.01;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.id = 0;

  if(l_elbow_path_.size() > 1)
  {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.points.resize(l_elbow_path_.size());
    int idx = 0;
    for(std::list<tf::Vector3>::iterator i = l_elbow_path_.begin(); i != l_elbow_path_.end(); i++, idx++)
    {
      marker.points[idx].x = i->x();
      marker.points[idx].y = i->y();
      marker.points[idx].z = i->z();
    }
    marker_array.markers.push_back(marker);
  }


  marker.id++;
  if(r_elbow_path_.size() > 1)
  {
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.points.resize(r_elbow_path_.size());
    int idx = 0;
    for (std::list<tf::Vector3>::iterator i = r_elbow_path_.begin(); i != r_elbow_path_.end(); i++, idx++)
    {
      marker.points[idx].x = i->x();
      marker.points[idx].y = i->y();
      marker.points[idx].z = i->z();
    }
    marker_array.markers.push_back(marker);
  }

}

void GestureDetectorElbowSwitch::drawResult(visualization_msgs::MarkerArray& marker_array, const std::string& frame_id)
{

}

int GestureDetectorElbowSwitch::lookForGesture(hg_user_interaction::Gesture& gesture)
{
  //check length
  double l_length, r_length;
  l_length = r_length = 0;
  tf::Vector3 l_dir, r_dir;
  if(l_elbow_path_.size() > 2)
  {
    l_dir = l_elbow_path_.back() - l_elbow_path_.front();
    l_length = l_dir.length();
  }
  else
  {
    return Gesture::GESTURE_NOT_DETECTED;
  }

  if(r_elbow_path_.size() > 2)
  {
    r_dir = r_elbow_path_.back() - r_elbow_path_.front();
    r_length = r_dir.length();

  }
  else
  {
    return Gesture::GESTURE_NOT_DETECTED;
  }



  if((l_length > min_length_) && (r_length > min_length_))
  {
    double dt = (ros::Time::now() - last_detected_time_).toSec();
    if(dt >= 1/max_detecting_rate_)
    {
      //check direction
      if((l_dir.z() > 0) && (r_dir.z() > 0))
      {
        l_elbow_path_.clear();
        r_elbow_path_.clear();
        last_detected_time_ = ros::Time::now();

        //ROS_INFO("l_length: %6.3f r_length: %6.3f", l_length, r_length);
        gesture.type = Gesture::GESTURE_ELBOW_TOGGLE;
        return Gesture::GESTURE_ELBOW_TOGGLE;
      }
    }
  }


  return Gesture::GESTURE_NOT_DETECTED;
}
