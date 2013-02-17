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

#include <hg_hand_interaction/gesture_push_pull.h>

using namespace hg_hand_interaction;
using namespace visualization_msgs;


PushPullHandGestureDetector::PushPullHandGestureDetector(ros::NodeHandle& nh_private)
  : HandGestureDetector(nh_private),
    num_hands_(0)
{
  current_state_[0] = current_state_[1] = IDEL;
  three_axes_[0] = tf::Vector3(1, 0, 0); //X
  three_axes_[1] = tf::Vector3(0, 1, 0); //Y
  three_axes_[2] = tf::Vector3(0, 0, 1); //Z
}

PushPullHandGestureDetector::~PushPullHandGestureDetector()
{

}

bool PushPullHandGestureDetector::initialize()
{
  bool b;
  double d;
  int i;

  nh_private_.getParam("push_pull_gesture_draw", b);
  checkbox_draw_marker_->setChecked(b);

  nh_private_.getParam("push_pull_gesture_x", d);
  spinbox_x_->setValue(d);
  nh_private_.getParam("push_pull_gesture_y", d);
  spinbox_y_->setValue(d);
  nh_private_.getParam("push_pull_gesture_z", d);
  spinbox_z_->setValue(d);

  nh_private_.getParam("push_pull_gesture_r1", d);
  spinbox_r1_->setValue(d);
  nh_private_.getParam("push_pull_gesture_r2", d);
  spinbox_r2_->setValue(d);
  nh_private_.getParam("push_pull_gesture_r3", d);
  spinbox_r3_->setValue(d);

  nh_private_.getParam("push_pull_gesture_timeout", d);
  spinbox_time_out_->setValue(d);
  nh_private_.getParam("push_pull_gesture_activating_time", d);
  spinbox_activating_time_->setValue(d);


  return true;
}

void PushPullHandGestureDetector::drawHistory(visualization_msgs::MarkerArray& marker_array, const std::string& frame_id)
{

}

void PushPullHandGestureDetector::drawResult(visualization_msgs::MarkerArray& marker_array, const std::string& frame_id)
{
  if(!checkbox_draw_marker_->isChecked())
    return;
  int marker_id = 0;
  Marker marker;
  marker.lifetime = ros::Duration(0.1);
  marker.header.frame_id = frame_id;
  marker.ns = "PushPullHandGestureDetector_result";
  marker.type = Marker::SPHERE;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;

  //right
  marker.pose.position.x = spinbox_x_->value();
  marker.pose.position.y = spinbox_y_->value();
  marker.pose.position.z = spinbox_z_->value();

  //R1
  if(num_hands_ == 1)
  {
    marker.scale.x = marker.scale.y = marker.scale.z = spinbox_r1_->value() * (current_state_[left_hand_ ? 1 : 0] == ACTIVATED ? 1 : 2);
    marker.color.r = (current_state_[left_hand_ ? 1 : 0] == ACTIVATED ? 0.0 : (current_state_[left_hand_ ? 1 : 0] == ACTIVATING ? 0.5 : 1.0));
    marker.color.g = (current_state_[left_hand_ ? 1 : 0] == ACTIVATED ? (is_moving_[left_hand_ ? 1 : 0]) ? 0.5 : 1.0 : 0.0);
  }
  else
  {
    marker.scale.x = marker.scale.y = marker.scale.z = spinbox_r1_->value() * (current_state_[1] == ACTIVATED ? 1 : 2);
    marker.color.r = (current_state_[1] == ACTIVATED ? 0.0 : (current_state_[1] == ACTIVATING ? 0.5 : 1.0));
    marker.color.g = (current_state_[1] == ACTIVATED ? (is_moving_[1]) ? 0.5 : 1.0 : 0.0);
  }
  marker.color.b = 0.0;
  marker.color.a = 0.5;
  marker.id = marker_id++;
  marker_array.markers.push_back(marker);

  //R2
  marker.scale.x = marker.scale.y = marker.scale.z = spinbox_r2_->value()*2;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.2;
  marker.id = marker_id++;
  marker_array.markers.push_back(marker);

  //R3
  marker.scale.x = marker.scale.y = marker.scale.z = spinbox_r3_->value()*2;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 0.1;
  marker.id = marker_id++;
  marker_array.markers.push_back(marker);


  //Left
  marker.pose.position.y = -spinbox_y_->value();

  //R1
  if(num_hands_ == 1)
  {
    marker.scale.x = marker.scale.y = marker.scale.z = spinbox_r1_->value() * (current_state_[left_hand_ ? 0 : 1] == ACTIVATED ? 1 : 2);
    marker.color.r = (current_state_[left_hand_ ? 0 : 1] == ACTIVATED ? 0.0 : (current_state_[left_hand_ ? 0 : 1] == ACTIVATING ? 0.5 : 1.0));
    marker.color.g = (current_state_[left_hand_ ? 0 : 1] == ACTIVATED ? (is_moving_[left_hand_ ? 0 : 1]) ? 0.5 : 1.0 : 0.0);
  }
  else
  {
    marker.scale.x = marker.scale.y = marker.scale.z = spinbox_r1_->value() * (current_state_[0] == ACTIVATED ? 1 : 2);
    marker.color.r = (current_state_[0] == ACTIVATED ? 0.0 : (current_state_[0] == ACTIVATING ? 0.5 : 1.0));
    marker.color.g = (current_state_[0] == ACTIVATED ? (is_moving_[0]) ? 0.5 : 1.0 : 0.0);
  }
  marker.color.b = 0.0;
  marker.color.a = 0.5;
  marker.id = marker_id++;
  marker_array.markers.push_back(marker);

  //R2
  marker.scale.x = marker.scale.y = marker.scale.z = spinbox_r2_->value() * 2;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.2;
  marker.id = marker_id++;
  marker_array.markers.push_back(marker);

  //R3
  marker.scale.x = marker.scale.y = marker.scale.z = spinbox_r3_->value() * 2;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 0.1;
  marker.id = marker_id++;
  marker_array.markers.push_back(marker);



}

void PushPullHandGestureDetector::addHandMessage(const hg_object_tracking::HandsConstPtr message)
{
  num_hands_ = 0;
  left_hand_ = true;

  if(message->hands.empty())
  {
    num_hands_ = 0;
    return;
  }

  if(message->hands.size() > 2)
  {
    num_hands_ = 0;
    return;
  }


  if(message->hands.size() == 2)
  {
    if(message->hands[0].hand_centroid.translation.y > message->hands[1].hand_centroid.translation.y)
    {
      tf::transformMsgToTF(message->hands[0].hand_centroid, last_hand_positions_[1]);
      tf::transformMsgToTF(message->hands[1].hand_centroid, last_hand_positions_[0]);
    }
    else
    {
      tf::transformMsgToTF(message->hands[0].hand_centroid, last_hand_positions_[0]);
      tf::transformMsgToTF(message->hands[1].hand_centroid, last_hand_positions_[1]);
    }
    num_hands_ = 2;
  }
  else
  {
    if(message->hands[0].hand_centroid.translation.y > 0)
    {
      left_hand_ = false;
    }
    else
    {
      left_hand_ = true;
    }
    tf::transformMsgToTF(message->hands[0].hand_centroid, last_hand_positions_[0]);
    num_hands_ = 1;
  }
}

int PushPullHandGestureDetector::lookForGesture()
{
  QMutexLocker locker(&mutex_);

  if(num_hands_ == 0)
  {
    //ROS_INFO_THROTTLE(1.0, "No hand");
    current_state_[0] = current_state_[1] = IDEL;
    return HandGesture::NOT_DETECTED;
  }
  int detected_gesture = HandGesture::NOT_DETECTED;

  //0 is LEFT side

  center_positions_[0].setValue(spinbox_x_->value(), -spinbox_y_->value(), spinbox_z_->value());
  center_positions_[1].setValue(spinbox_x_->value(), spinbox_y_->value(), spinbox_z_->value());
  if(num_hands_ == 1)
  {
    tf::Vector3 vec_to_hand;
    if(left_hand_)
    {
      vec_to_hand = last_hand_positions_[0].getOrigin() - center_positions_[0];
    }
    else
    {
      vec_to_hand = last_hand_positions_[0].getOrigin() - center_positions_[1];
    }
    detected_gesture = getState(0, vec_to_hand);


  }
  else
  {
    tf::Vector3 vec_to_left = last_hand_positions_[0].getOrigin() - center_positions_[0];
    tf::Vector3 vec_to_right = last_hand_positions_[1].getOrigin() - center_positions_[1];
    int detected_gesture_l = getState(0, vec_to_left);
    int detected_gesture_r = getState(1, vec_to_right);
    switch(detected_gesture_l)
    {
      case HandGesture::PUSH_PULL_ZP:
        if(detected_gesture_r == HandGesture::PUSH_PULL_ZN)
        {
          ROS_INFO("Rotate X-");
          detected_gesture = HandGesture::PUSH_PULL_RXN;
        }
        break;
      case HandGesture::PUSH_PULL_ZN:
        if(detected_gesture_r == HandGesture::PUSH_PULL_ZP)
        {
          ROS_INFO("Rotate X+");
          detected_gesture = HandGesture::PUSH_PULL_RXP;
        }
        break;
      case HandGesture::PUSH_PULL_XP:
        if (detected_gesture_r == HandGesture::PUSH_PULL_XP)
        {
          ROS_INFO("Rotate Y+");
          detected_gesture = HandGesture::PUSH_PULL_RYP;
        }
        else if (detected_gesture_r == HandGesture::PUSH_PULL_XN)
        {
          ROS_INFO("Rotate Z+");
          detected_gesture = HandGesture::PUSH_PULL_RZP;
        }
        break;
      case HandGesture::PUSH_PULL_XN:
        if (detected_gesture_r == HandGesture::PUSH_PULL_XP)
        {
          ROS_INFO("Rotate Z-");
          detected_gesture = HandGesture::PUSH_PULL_RZN;
        }
        else if (detected_gesture_r == HandGesture::PUSH_PULL_XN)
        {
          ROS_INFO("Rotate Y-");
          detected_gesture = HandGesture::PUSH_PULL_RYN;
        }
        break;
    }
  }

  return detected_gesture;
}

int PushPullHandGestureDetector::getState(int hand, const tf::Vector3& vec_to_hand)
{
  int detected_gesture = HandGesture::NOT_DETECTED;
  double ds = vec_to_hand.length();
  switch (current_state_[hand])
  {
    case IDEL:
      //ROS_INFO("IDEL");
      if (ds < spinbox_r3_->value())
        current_state_[hand] = ENTERING;
      break;
    case ENTERING:
      //ROS_INFO("ENTERING");
      if (ds < spinbox_r2_->value())
      {
        start_activating_time_[hand] = ros::Time::now();
        current_state_[hand] = ACTIVATING;
      }
      else if (ds > spinbox_r3_->value())
      {
        current_state_[hand] = IDEL;
      }
      break;
    case ACTIVATING:
      //ROS_INFO("ACTIVATING");
      if (ds < spinbox_r1_->value())
      {
        double activating_time = (ros::Time::now() - start_activating_time_[hand]).toSec();
        if (activating_time >= spinbox_activating_time_->value())
        {
          current_state_[hand] = ACTIVATED;
        }
      }
      else
      {
        current_state_[hand] = IDEL;
      }
      break;
    case ACTIVATED:
      //ROS_INFO("ACTIVATED");
      if (ds < spinbox_r2_->value())
      {
        if (ds < (spinbox_r1_->value() / 2.0))
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
                ROS_INFO("[%d] X+", hand);
                detected_gesture = HandGesture::PUSH_PULL_XP;
              }
              else
              {
                ROS_INFO("[%d] X-", hand);
                detected_gesture = HandGesture::PUSH_PULL_XN;
              }
              break;
            case 1:
              if (dot_products[min_error_index] > 0)
              {
                ROS_INFO("[%d] Y+", hand);
                detected_gesture = HandGesture::PUSH_PULL_YP;
              }
              else
              {
                ROS_INFO("[%d] Y-", hand);
                detected_gesture = HandGesture::PUSH_PULL_YN;
              }
              break;
            case 2:
              if (dot_products[min_error_index] > 0)
              {
                ROS_INFO("[%d] Z+", hand);
                detected_gesture = HandGesture::PUSH_PULL_ZP;
              }
              else
              {
                ROS_INFO("[%d] Z-", hand);
                detected_gesture = HandGesture::PUSH_PULL_ZN;
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
      //ROS_INFO("LEAVING");
      if (ds > spinbox_r3_->value())
      {
        current_state_[hand] = IDEL;
      }
      else if (ds < spinbox_r2_->value())
      {
        current_state_[hand] = last_state_[hand];
      }
      break;
    default:
      break;
  }
  return detected_gesture;
}


void PushPullHandGestureDetector::addUI(QToolBox* tool_box)
{
  spinbox_x_ = new QDoubleSpinBox;
  spinbox_x_->setMinimum(-10.0);
  spinbox_x_->setMaximum(10.0);
  spinbox_x_->setSingleStep(0.01);

  spinbox_y_ = new QDoubleSpinBox;
  spinbox_y_->setMinimum(-10.0);
  spinbox_y_->setMaximum(10.0);
  spinbox_y_->setSingleStep(0.01);

  spinbox_z_ = new QDoubleSpinBox;
  spinbox_z_->setMinimum(-10.0);
  spinbox_z_->setMaximum(10.0);
  spinbox_z_->setSingleStep(0.01);

  QHBoxLayout* h_layout_x = new QHBoxLayout;
  QHBoxLayout* h_layout_y = new QHBoxLayout;
  QHBoxLayout* h_layout_z = new QHBoxLayout;
  h_layout_x->addWidget(new QLabel("x"));
  h_layout_x->addWidget(spinbox_x_);
  h_layout_y->addWidget(new QLabel("y"));
  h_layout_y->addWidget(spinbox_y_);
  h_layout_z->addWidget(new QLabel("z"));
  h_layout_z->addWidget(spinbox_z_);

  spinbox_r1_ = new QDoubleSpinBox;
  spinbox_r1_->setMinimum(0.0);
  spinbox_r1_->setMaximum(1.0);
  spinbox_r1_->setSingleStep(0.01);

  spinbox_r2_ = new QDoubleSpinBox;
  spinbox_r2_->setMinimum(0.0);
  spinbox_r2_->setMaximum(1.0);
  spinbox_r2_->setSingleStep(0.01);

  spinbox_r3_ = new QDoubleSpinBox;
  spinbox_r3_->setMinimum(0.0);
  spinbox_r3_->setMaximum(1.0);
  spinbox_r3_->setSingleStep(0.01);

  QHBoxLayout* h_layout_r1 = new QHBoxLayout;
  QHBoxLayout* h_layout_r2 = new QHBoxLayout;
  QHBoxLayout* h_layout_r3 = new QHBoxLayout;
  h_layout_r1->addWidget(new QLabel("R1"));
  h_layout_r1->addWidget(spinbox_r1_);
  h_layout_r2->addWidget(new QLabel("R2"));
  h_layout_r2->addWidget(spinbox_r2_);
  h_layout_r3->addWidget(new QLabel("R3"));
  h_layout_r3->addWidget(spinbox_r3_);


  spinbox_time_out_ = new QDoubleSpinBox;
  spinbox_time_out_->setMinimum(0.0);
  spinbox_time_out_->setMaximum(10.0);
  spinbox_time_out_->setSingleStep(0.1);

  spinbox_activating_time_ = new QDoubleSpinBox;
  spinbox_activating_time_->setMinimum(0.0);
  spinbox_activating_time_->setMaximum(10.0);
  spinbox_activating_time_->setSingleStep(0.1);

  QHBoxLayout* h_layout_time_out = new QHBoxLayout;
  QHBoxLayout* h_layout_activating_time = new QHBoxLayout;
  h_layout_time_out->addWidget(new QLabel("Time out"));
  h_layout_time_out->addWidget(spinbox_time_out_);
  h_layout_activating_time->addWidget(new QLabel("Active"));
  h_layout_activating_time->addWidget(spinbox_activating_time_);


  QVBoxLayout* v_layout = new QVBoxLayout;
  v_layout->addWidget(checkbox_draw_marker_);
  v_layout->addLayout(h_layout_x);
  v_layout->addLayout(h_layout_y);
  v_layout->addLayout(h_layout_z);
  v_layout->addLayout(h_layout_r1);
  v_layout->addLayout(h_layout_r2);
  v_layout->addLayout(h_layout_r3);
  v_layout->addLayout(h_layout_time_out);
  v_layout->addLayout(h_layout_activating_time);
  v_layout->addStretch();

  QWidget *item_widget = new QWidget;
  item_widget->setLayout(v_layout);
  tool_box->setSizePolicy(QSizePolicy(QSizePolicy::Maximum, QSizePolicy::Ignored));
  tool_box->setMinimumWidth(item_widget->sizeHint().width());
  tool_box->addItem(item_widget, tr("Push Pull Gesture"));


}




