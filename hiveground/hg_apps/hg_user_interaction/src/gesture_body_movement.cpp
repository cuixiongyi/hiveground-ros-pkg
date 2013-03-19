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
  current_state_= IDEL;
  three_axes_[0] = tf::Vector3(1, 0, 0); //X
  three_axes_[1] = tf::Vector3(0, 1, 0); //Y
  three_axes_[2] = tf::Vector3(0, 0, 1); //Z
  skeleton_updated_ = false;

  nh_private_.getParam("body_movement_gesture_draw_history", draw_history_);
  nh_private_.getParam("body_movement_gesture_draw_result", draw_result_);

  nh_private_.getParam("body_movement_gesture_r1", r1_);
  nh_private_.getParam("body_movement_gesture_r2", r2_);
  nh_private_.getParam("body_movement_gesture_r3", r3_);

  nh_private_.getParam("body_movement_gesture_timeout", time_out_);
  nh_private_.getParam("body_movement_gesture_activating_time", activating_time_);

  return true;
}

void GestureBodyMovement::addSkeletonsMessage(const kinect_msgs::SkeletonsConstPtr& skeletons)
{
  //ROS_INFO_THROTTLE(1.0, __FUNCTION__);

  tf::Transform tf0, tf1, tf2, tf3;
  for (int i = 0; i < Skeletons::SKELETON_COUNT; i++)
  {
    if (skeletons->skeletons[i].skeleton_tracking_state == Skeleton::SKELETON_TRACKED)
    {
      skeleton_updated_ = true;
      //ROS_INFO_STREAM("[" << i << "] " << skeletons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_SHOULDER_CENTER]);
      tf::transformMsgToTF(skeletons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_SHOULDER_CENTER], tf0);
      tf::transformMsgToTF(skeletons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_SHOULDER_LEFT], tf1);
      tf::transformMsgToTF(skeletons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_SHOULDER_RIGHT], tf2);
      tf::transformMsgToTF(skeletons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_HEAD], tf3);
      last_position_.setOrigin((tf0.getOrigin() + tf1.getOrigin() + tf2.getOrigin() + tf3.getOrigin()) / 4.0);
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
  if(!skeleton_updated_) return;

  Marker marker;
  marker.lifetime = ros::Duration(0.1);
  marker.header.frame_id = frame_id;
  marker.ns = "GestureBodyMovement_result";
  marker.type = Marker::SPHERE;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.id = 0;


  marker.pose.position.x = center_position_.x();
  marker.pose.position.y = center_position_.y();
  marker.pose.position.z = center_position_.z();
  marker.scale.x = marker.scale.y = marker.scale.z = 0.02;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.id++;
  marker_array.markers.push_back(marker);


  marker.pose.position.x = activated_position_.getOrigin().x();
  marker.pose.position.y = activated_position_.getOrigin().y();
  marker.pose.position.z = activated_position_.getOrigin().z();
  marker.scale.x = marker.scale.y = marker.scale.z = getR1() * (current_state_ == ACTIVATED ? 1 : 2);
  marker.color.r = (current_state_ == ACTIVATED ? 0.0 : (current_state_ == ACTIVATING ? 0.5 : 1.0));
  marker.color.g = (current_state_ == ACTIVATED ? (is_moving_) ? 0.5 : 1.0 : 0.0);
  marker.color.b = 0.0;
  marker.color.a = 0.5;
  marker.id++;
  marker_array.markers.push_back(marker);

  //R2
  marker.scale.x = marker.scale.y = marker.scale.z = getR2() * 2;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.2;
  marker.id++;
  marker_array.markers.push_back(marker);

  //R3
  marker.scale.x = marker.scale.y = marker.scale.z = getR3() * 2;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 0.1;
  marker.id++;
  marker_array.markers.push_back(marker);


}

int GestureBodyMovement::lookForGesture(hg_user_interaction::Gesture& gesture)
{
  if(!skeleton_updated_) return Gesture::GESTURE_NOT_DETECTED;

  center_position_ = last_position_.getOrigin();
  int detected_gesture = Gesture::GESTURE_NOT_DETECTED;

  tf::Vector3 vec_to_body;
  if (current_state_ == IDEL)
    vec_to_body = last_position_.getOrigin() - center_position_;
  else
    vec_to_body = last_position_.getOrigin() - activated_position_.getOrigin();
  detected_gesture = getState(vec_to_body);

  if(detected_gesture != Gesture::GESTURE_NOT_DETECTED)
  {
    gesture.direction = detected_gesture;
    gesture.hand_count = 0;
    gesture.type = Gesture::GESTURE_BODY_MOVE;
    gesture.vectors.resize(1);
    gesture.vars.resize(1);
    tf::vector3TFToMsg(vec_to_body, gesture.vectors[0]);
    gesture.vars[0] = vec_to_body.length() / getR2();
  }

  return detected_gesture;
}

int GestureBodyMovement::getState(const tf::Vector3& vec_to_body)
{
  int detected_gesture = Gesture::GESTURE_NOT_DETECTED;
  double ds = vec_to_body.length();
  switch (current_state_)
  {
    case IDEL:
      //ROS_INFO("IDEL %f", ds);
      if (ds < getR2())
      {
        //set current hand position as center
        start_activating_time_ = ros::Time::now();
        activated_position_ = last_position_;
        current_state_ = ENTERING;
      }
      break;
    case ENTERING:
      //ROS_INFO("ENTERING");1
      if (ds < getR1())
      {
        double entering_time = (ros::Time::now() - start_activating_time_).toSec();
        if (entering_time > (getActivatingTime() * 0.5))
        {
          start_activating_time_ = ros::Time::now();
          current_state_ = ACTIVATING;
        }
      }
      else
      {
        current_state_ = IDEL;
      }
      break;
    case ACTIVATING:
      //ROS_INFO("ACTIVATING");
      if (ds < getR1() * 0.5)
      {
        double activating_time = (ros::Time::now() - start_activating_time_).toSec();
        if (activating_time >= getActivatingTime())
        {
          //set current hand position as center
          activated_position_ = last_position_;
          current_state_ = ACTIVATED;
        }
      }
      else
      {
        current_state_ = IDEL;
      }
      break;
      break;
    case ACTIVATED:
      //ROS_INFO("ACTIVATED");
      if (ds < getR2())
      {
        if (ds < (getR1() / 2.0))
        {
          is_moving_ = false;
        }
        else
        {
          is_moving_ = true;
          //check direction
          double dot_products[3];
          double min_error = 1e6;
          int min_error_index = -1;
          for (int i = 0; i < 3; i++)
          {
            dot_products[i] = vec_to_body.dot(three_axes_[i]);

            if ((1 - fabs(dot_products[i])) < min_error)
            {
              min_error = 1 - fabs(dot_products[i]);
              min_error_index = i;
            }
          }

          switch (min_error_index)
          {
            case 0:
              if (dot_products[min_error_index] > 0)
              {
                ROS_INFO_THROTTLE(1.0, "X+");
                detected_gesture = Gesture::DIR_X_POS;
              }
              else
              {
                ROS_INFO_THROTTLE(1.0, "X-");
                detected_gesture = Gesture::DIR_X_NEG;
              }
              break;
            case 1:
              if (dot_products[min_error_index] > 0)
              {
                ROS_INFO_THROTTLE(1.0, "Y+");
                detected_gesture = Gesture::DIR_Y_POS;
              }
              else
              {
                ROS_INFO_THROTTLE(1.0, "Y-");
                detected_gesture = Gesture::DIR_Y_NEG;
              }
              break;
            case 2:
              if (dot_products[min_error_index] > 0)
              {
                ROS_INFO_THROTTLE(1.0, "Z+");
                detected_gesture = Gesture::DIR_Z_POS;
              }
              else
              {
                ROS_INFO_THROTTLE(1.0, "Z-");
                detected_gesture = Gesture::DIR_Z_NEG;
              }
              break;
            default:
              break;
          }
        }
      }
      else
      {
        last_state_ = current_state_;
        current_state_ = LEAVING;
      }
      break;
    case LEAVING:
      //ROS_INFO("LEAVING");
      if (ds > getR3())
      {
        current_state_ = IDEL;
      }
      else if (ds < getR2())
      {
        current_state_ = last_state_;
      }
      break;
  }
  return detected_gesture;
}
