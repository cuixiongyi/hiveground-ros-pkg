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
using namespace visualization_msgs;

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
  if(hands->hands.empty())
  {
    num_hands_ = 0;
    return;
  }

  if(hands->hands.size() > 2)
  {
    num_hands_ = 0;
    return;
  }

  if(num_hands_ != (int)hands->hands.size())
  {
    current_state_[0] = current_state_[1] = IDEL;
  }

  if(hands->hands.size() == 2)
  {
    if(hands->hands[0].hand_centroid.translation.y > hands->hands[1].hand_centroid.translation.y)
    {
      tf::transformMsgToTF(hands->hands[0].hand_centroid, last_hand_positions_[1]);
      tf::transformMsgToTF(hands->hands[1].hand_centroid, last_hand_positions_[0]);
    }
    else
    {
      tf::transformMsgToTF(hands->hands[0].hand_centroid, last_hand_positions_[0]);
      tf::transformMsgToTF(hands->hands[1].hand_centroid, last_hand_positions_[1]);
    }
    num_hands_ = 2;
  }
  else
  {
    if(hands->hands[0].hand_centroid.translation.y > 0)
    {
      left_hand_ = false;
    }
    else
    {
      left_hand_ = true;
    }
    tf::transformMsgToTF(hands->hands[0].hand_centroid, last_hand_positions_[0]);
    num_hands_ = 1;
  }
}

void GestureDetectorHandPushPull::drawHistory(visualization_msgs::MarkerArray& marker_array, const std::string& frame_id)
{
  if(!getDrawHistory()) return;
}

void GestureDetectorHandPushPull::drawResult(visualization_msgs::MarkerArray& marker_array, const std::string& frame_id)
{
  if(!getDrawResult()) return;
  if(num_hands_ == 0) return;

  Marker marker;
  marker.lifetime = ros::Duration(0.1);
  marker.header.frame_id = frame_id;
  marker.ns = "GestureDetectorHandPushPull_result";
  marker.type = Marker::SPHERE;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.id = 0;

  if (current_state_[0] == IDEL)
  {
    marker.pose.position.x = center_positions_[0].x();
    marker.pose.position.y = center_positions_[0].y();
    marker.pose.position.z = center_positions_[0].z();
  }
  else
  {
    marker.pose.position.x = activated_hand_positions_[0].getOrigin().x();
    marker.pose.position.y = activated_hand_positions_[0].getOrigin().y();
    marker.pose.position.z = activated_hand_positions_[0].getOrigin().z();
  }

  //if(current_state_[0] != IDEL)
  {
    marker.scale.x = marker.scale.y = marker.scale.z = getR1() * (current_state_[0] == ACTIVATED ? 1 : 2);
    marker.color.r = (current_state_[0] == ACTIVATED ? 0.0 : (current_state_[0] == ACTIVATING ? 0.5 : 1.0));
    marker.color.g = (current_state_[0] == ACTIVATED ? (is_moving_[0]) ? 0.5 : 1.0 : 0.0);
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    marker.id++;
    marker_array.markers.push_back(marker);
  }

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

  if (num_hands_ == 2)
  {
    if (current_state_[1] == IDEL)
    {
      marker.pose.position.x = center_positions_[1].x();
      marker.pose.position.y = center_positions_[1].y();
      marker.pose.position.z = center_positions_[1].z();
    }
    else
    {
      marker.pose.position.x = activated_hand_positions_[1].getOrigin().x();
      marker.pose.position.y = activated_hand_positions_[1].getOrigin().y();
      marker.pose.position.z = activated_hand_positions_[1].getOrigin().z();
    }

    if (current_state_[1] != IDEL)
    {
      marker.scale.x = marker.scale.y = marker.scale.z = getR1() * (current_state_[1] == ACTIVATED ? 1 : 2);
      marker.color.r = (current_state_[1] == ACTIVATED ? 0.0 : (current_state_[1] == ACTIVATING ? 0.5 : 1.0));
      marker.color.g = (current_state_[1] == ACTIVATED ? (is_moving_[1]) ? 0.5 : 1.0 : 0.0);
      marker.color.b = 0.0;
      marker.color.a = 0.5;
      marker.id++;
      marker_array.markers.push_back(marker);
    }

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

}

int GestureDetectorHandPushPull::lookForGesture(hg_user_interaction::Gesture& gesture)
{
  if (num_hands_ == 0)
  {
    current_state_[0] = current_state_[1] = IDEL;
    return Gesture::GESTURE_NOT_DETECTED;
  }

  center_positions_[0] = last_hand_positions_[0].getOrigin();
  if (num_hands_ > 1)
    center_positions_[1] = last_hand_positions_[1].getOrigin();

  int detected_gesture = Gesture::GESTURE_NOT_DETECTED;
  if (num_hands_ == 1)
  {
    tf::Vector3 vec_to_hand;
    if (current_state_[0] == IDEL)
      vec_to_hand = last_hand_positions_[0].getOrigin() - center_positions_[left_hand_ ? 0 : 1];
    else
      vec_to_hand = last_hand_positions_[0].getOrigin() - activated_hand_positions_[0].getOrigin();
    detected_gesture = getStateAuto(0, vec_to_hand);

    gesture.direction = detected_gesture;
    gesture.hand_count = 1;
    gesture.type = Gesture::GESTURE_HAND_PUSH_PULL;
    gesture.vectors.resize(1);
    gesture.vars.resize(1);
    tf::vector3TFToMsg(vec_to_hand, gesture.vectors[0]);
    gesture.vars[0] = vec_to_hand.length() / getR3();
    //return Gesture::GESTURE_HAND_PUSH_PULL;
  }
  else
  {
    tf::Vector3 vec_to_left;
    if (current_state_[0] == IDEL)
      vec_to_left = last_hand_positions_[0].getOrigin() - center_positions_[0];
    else
      vec_to_left = last_hand_positions_[0].getOrigin() - activated_hand_positions_[0].getOrigin();

    tf::Vector3 vec_to_right;
    if (current_state_[1] == IDEL)
      vec_to_right = last_hand_positions_[1].getOrigin() - center_positions_[1];
    else
      vec_to_right = last_hand_positions_[1].getOrigin() - activated_hand_positions_[1].getOrigin();

    int detected_gesture_l = getStateAuto(0, vec_to_left);
    int detected_gesture_r = getStateAuto(1, vec_to_right);


    switch (detected_gesture_l)
    {
      case Gesture::DIR_Z_POS:
        if (detected_gesture_r == Gesture::DIR_Z_NEG)
        {
          ROS_INFO_THROTTLE(1.0, "Rotate X-");
          detected_gesture = Gesture::ROT_X_NEG;
        }
        break;
      case Gesture::DIR_Z_NEG:
        if (detected_gesture_r == Gesture::DIR_Z_POS)
        {
          ROS_INFO_THROTTLE(1.0, "Rotate X+");
          detected_gesture = Gesture::ROT_X_POS;
        }
        break;
      case Gesture::DIR_X_POS:
        if (detected_gesture_r == Gesture::DIR_X_POS)
        {
          ROS_INFO_THROTTLE(1.0, "Rotate Y+");
          detected_gesture = Gesture::ROT_Y_POS;
        }
        else if (detected_gesture_r == Gesture::DIR_X_NEG)
        {
          ROS_INFO_THROTTLE(1.0, "Rotate Z+");
          detected_gesture = Gesture::ROT_Z_POS;
        }
        break;
      case Gesture::DIR_X_NEG:
        if (detected_gesture_r == Gesture::DIR_X_POS)
        {
          ROS_INFO_THROTTLE(1.0, "Rotate Z-");
          detected_gesture = Gesture::ROT_Z_NEG;
        }
        else if (detected_gesture_r == Gesture::DIR_X_NEG)
        {
          ROS_INFO_THROTTLE(1.0, "Rotate Y-");
          detected_gesture = Gesture::ROT_Y_NEG;
        }
        break;
    }

    if(detected_gesture != Gesture::GESTURE_NOT_DETECTED)
    {
      gesture.direction = detected_gesture;
      gesture.hand_count = 2;
      gesture.type = Gesture::GESTURE_HAND_PUSH_PULL;
      gesture.vectors.resize(2);
      gesture.vars.resize(2);
      tf::vector3TFToMsg(vec_to_left, gesture.vectors[0]);
      tf::vector3TFToMsg(vec_to_right, gesture.vectors[1]);
      gesture.vars[0] = vec_to_left.length() / getR3();
      gesture.vars[1] = vec_to_right.length() / getR3();
      //return Gesture::GESTURE_HAND_PUSH_PULL;
    }
  }


  return detected_gesture;
}

int GestureDetectorHandPushPull::getStateAuto(int hand, const tf::Vector3& vec_to_hand)
{
  int detected_gesture = Gesture::GESTURE_NOT_DETECTED;
  double ds = vec_to_hand.length();
  switch (current_state_[hand])
  {
    case IDEL:
      //ROS_INFO("%d IDEL %f", hand, ds);
      if (ds < getR2())
      {
        //set current hand position as center
        start_activating_time_[hand] = ros::Time::now();
        activated_hand_positions_[hand] = last_hand_positions_[hand];
        current_state_[hand] = ENTERING;
      }
      break;
    case ENTERING:
      //ROS_INFO("%d ENTERING", hand);
      if (ds < getR1())
      {
        double entering_time = (ros::Time::now() - start_activating_time_[hand]).toSec();
        if (entering_time > (getActivatingTime() * 0.5))
        {
          start_activating_time_[hand] = ros::Time::now();
          current_state_[hand] = ACTIVATING;
        }
      }
      else
      {
        current_state_[hand] = IDEL;
      }
      break;
    case ACTIVATING:
      //ROS_INFO("%d ACTIVATING", hand);
      if (ds < getR1() * 0.5)
      {
        double activating_time = (ros::Time::now() - start_activating_time_[hand]).toSec();
        if (activating_time >= getActivatingTime())
        {
          //set current hand position as center
          activated_hand_positions_[hand] = last_hand_positions_[hand];
          current_state_[hand] = ACTIVATED;
        }
      }
      else
      {
        current_state_[hand] = IDEL;
      }
      break;
      break;
    case ACTIVATED:
      //ROS_INFO("%d ACTIVATED", hand);
      if (ds < getR2())
      {
        if (ds < (getR1() / 2.0))
        {
          is_moving_[hand] = false;
        }
        else
        {
          is_moving_[hand] = true;
          //check direction
          double dot_products[3];
          double min_error = 1e6;
          int min_error_index = -1;
          for (int i = 0; i < 3; i++)
          {
            dot_products[i] = vec_to_hand.dot(three_axes_[i]);

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
                ROS_INFO_THROTTLE(1.0, "[%d] X+", hand);
                detected_gesture = Gesture::DIR_X_POS;
              }
              else
              {
                ROS_INFO_THROTTLE(1.0, "[%d] X-", hand);
                detected_gesture = Gesture::DIR_X_NEG;
              }
              break;
            case 1:
              if (dot_products[min_error_index] > 0)
              {
                ROS_INFO_THROTTLE(1.0, "[%d] Y+", hand);
                detected_gesture = Gesture::DIR_Y_POS;
              }
              else
              {
                ROS_INFO_THROTTLE(1.0, "[%d] Y-", hand);
                detected_gesture = Gesture::DIR_Y_NEG;
              }
              break;
            case 2:
              if (dot_products[min_error_index] > 0)
              {
                ROS_INFO_THROTTLE(1.0, "[%d] Z+", hand);
                detected_gesture = Gesture::DIR_Z_POS;
              }
              else
              {
                ROS_INFO_THROTTLE(1.0, "[%d] Z-", hand);
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
        last_state_[hand] = current_state_[hand];
        current_state_[hand] = LEAVING;
      }
      break;
    case LEAVING:
      //ROS_INFO("%d LEAVING", hand);
      if (ds > getR3())
      {
        current_state_[hand] = IDEL;
      }
      else if (ds < getR2())
      {
        current_state_[hand] = last_state_[hand];
      }
      break;
  }
  return detected_gesture;
}

