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

#include <hg_hand_interaction/gesture.h>


using namespace visualization_msgs;
using namespace hg_hand_interaction;

GestureDetector::GestureDetector(ros::NodeHandle& nh_private)
    : nh_private_(nh_private)
{

}

SweepHandGestureDetector::SweepHandGestureDetector(ros::NodeHandle& nh_private)
 : HandGestureDetector(nh_private),
   last_hand_count_(0)
{



}

bool SweepHandGestureDetector::initialize()
{
  double d;
  nh_private_.getParam("sweep_hand_gesture_windows", d);
  spinbox_window_->setValue(d);

  nh_private_.getParam("sweep_hand_gesture_gap", d);
  spinbox_gap_->setValue(d);
  return true;
}

void SweepHandGestureDetector::drawHistory(MarkerArray& marker_array, const std::string& frame_id)
{
  Marker markers[2];
  markers[0].type = Marker::LINE_STRIP;
  markers[0].lifetime = ros::Duration(0.1);
  markers[0].header.frame_id = frame_id;
  markers[0].ns = "SweepHandGestureDetector";
  markers[0].id = 0;
  markers[0].scale.x = 0.01;
  markers[1].type = Marker::LINE_STRIP;
  markers[1].lifetime = ros::Duration(0.1);
  markers[1].header.frame_id = frame_id;
  markers[1].ns = "SweepHandGestureDetector";
  markers[1].id = 1;
  markers[1].scale.x = 0.01;
  std::list<PositionsStamped>::iterator it;
  geometry_msgs::Point point;
  for(it = gesture_entries_.begin(); it != gesture_entries_.end(); it++)
  {
    point.x = it->second[0].getOrigin().x();
    point.y = it->second[0].getOrigin().y();
    point.z = it->second[0].getOrigin().z();
    markers[0].points.push_back(point);

    if(last_hand_count_ == 2)
    {
      point.x = it->second[1].getOrigin().x();
      point.y = it->second[1].getOrigin().y();
      point.z = it->second[1].getOrigin().z();
      markers[1].points.push_back(point);
    }
  }



  markers[0].pose.position.x = 0;
  markers[0].pose.position.y = 0;
  markers[0].pose.position.z = 0;
  markers[0].pose.orientation.x = 0;
  markers[0].pose.orientation.y = 0;
  markers[0].pose.orientation.z = 0;
  markers[0].pose.orientation.w = 1;
  markers[0].color.r = 1.0;
  markers[0].color.g = 0.0;
  markers[0].color.b = 1.0;
  markers[0].color.a = 1.0;
  marker_array.markers.push_back(markers[0]);
  if(last_hand_count_ == 2)
  {
    markers[1].pose.position.x = 0;
    markers[1].pose.position.y = 0;
    markers[1].pose.position.z = 0;
    markers[1].pose.orientation.x = 0;
    markers[1].pose.orientation.y = 0;
    markers[1].pose.orientation.z = 0;
    markers[1].pose.orientation.w = 1;
    markers[1].color.r = 0.0;
    markers[1].color.g = 1.0;
    markers[1].color.b = 0.0;
    markers[1].color.a = 1.0;
    marker_array.markers.push_back(markers[1]);
  }
}

void SweepHandGestureDetector::addHandMessage(const hg_object_tracking::HandsConstPtr message)
{
  if(message->hands.empty())
  {
    ROS_WARN_THROTTLE(1.0, "%s got an empty Hands message.", __FUNCTION__);
    return;
  }

  if(message->hands.size() > 2)
  {
    ROS_WARN_THROTTLE(1.0, "%s got more than two hands in Hands message."
                           " The first two hands will be used", __FUNCTION__);
  }

  int num_hand = message->hands.size();

  if(last_hand_count_ != num_hand)
  {
    ROS_DEBUG("Hand count changed from %d to %d", last_hand_count_, num_hand);
    gesture_entries_.clear();
    last_hand_count_ = num_hand;
  }

  PositionsStamped position_stamped;
  position_stamped.first = message->header.stamp;
  tf::Vector3 translation;
  tf::Quaternion rotation;

  //use hand position and arm direction
  if (num_hand > 1)
  {
    if(message->hands[1].hand_centroid.translation.y > message->hands[0].hand_centroid.translation.y)
    {
      tf::vector3MsgToTF(message->hands[0].hand_centroid.translation, translation);
      tf::quaternionMsgToTF(message->hands[0].arm_centroid.rotation, rotation);
      position_stamped.second.push_back(tf::Transform(rotation, translation));

      tf::vector3MsgToTF(message->hands[1].hand_centroid.translation, translation);
      tf::quaternionMsgToTF(message->hands[1].arm_centroid.rotation, rotation);
      position_stamped.second.push_back(tf::Transform(rotation, translation));
    }
    else
    {
      tf::vector3MsgToTF(message->hands[1].hand_centroid.translation, translation);
      tf::quaternionMsgToTF(message->hands[1].arm_centroid.rotation, rotation);
      position_stamped.second.push_back(tf::Transform(rotation, translation));

      tf::vector3MsgToTF(message->hands[0].hand_centroid.translation, translation);
      tf::quaternionMsgToTF(message->hands[0].arm_centroid.rotation, rotation);
      position_stamped.second.push_back(tf::Transform(rotation, translation));
    }
  }
  else
  {
    tf::vector3MsgToTF(message->hands[0].hand_centroid.translation, translation);
    tf::quaternionMsgToTF(message->hands[0].arm_centroid.rotation, rotation);
    position_stamped.second.push_back(tf::Transform(rotation, translation));
  }

  gesture_entries_.push_front(position_stamped);
  if(gesture_entries_.size() > 1)
  {

    ros::Duration duration(gesture_entries_.front().first - gesture_entries_.back().first);
    if(duration > gesture_window_)
    {
      gesture_entries_.pop_back();
    }
  }
  detected_gesture_ = NOT_DETECTED;
}

bool SweepHandGestureDetector::lookForGesture()
{
  return true;
}

void SweepHandGestureDetector::addUI(QToolBox* tool_box)
{
  QLabel* label1 = new QLabel("Window");
  QLabel* label2 = new QLabel("Gap");
  spinbox_window_ = new QDoubleSpinBox;
  spinbox_gap_ = new QDoubleSpinBox;
  spinbox_window_->setMinimum(0.01);
  spinbox_window_->setMaximum(10.0);
  spinbox_window_->setSingleStep(0.01);
  spinbox_gap_->setMinimum(0.01);
  spinbox_gap_->setMaximum(10.0);
  spinbox_gap_->setSingleStep(0.01);


  QGridLayout* grid_layout = new QGridLayout;
  grid_layout->addWidget(label1, 0, 0);
  grid_layout->addWidget(spinbox_window_, 0, 1);
  grid_layout->addWidget(label2, 1, 0);
  grid_layout->addWidget(spinbox_gap_, 1, 1);

  QWidget *item_widget = new QWidget;
  item_widget->setLayout(grid_layout);
  tool_box->setSizePolicy(QSizePolicy(QSizePolicy::Maximum, QSizePolicy::Ignored));
  tool_box->setMinimumWidth(item_widget->sizeHint().width());
  tool_box->addItem(item_widget, tr("Sweep Hand Gesture"));

  connect(spinbox_window_, SIGNAL(valueChanged(double)), this, SLOT(onWindowTimeValueChanged(double)));
  connect(spinbox_gap_, SIGNAL(valueChanged(double)), this, SLOT(onGapTimeValueChanged(double)));
}

void SweepHandGestureDetector::onWindowTimeValueChanged(double d)
{
  QMutexLocker lock(&mutex_);
  gesture_window_.fromSec(d);
  gesture_entries_.clear();
}

void SweepHandGestureDetector::onGapTimeValueChanged(double d)
{
  QMutexLocker lock(&mutex_);
  gesture_gap_.fromSec(d);
  gesture_entries_.clear();
}



