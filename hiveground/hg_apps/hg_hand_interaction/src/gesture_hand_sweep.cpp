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

#include <hg_hand_interaction/gesture_hand_sweep.h>

#include <QLabel>
#include <QGridLayout>

using namespace hg_hand_interaction;
using namespace visualization_msgs;

SweepHandGestureDetector::SweepHandGestureDetector(ros::NodeHandle& nh_private)
 : HandGestureDetector(nh_private),
   last_hand_count_(0)
{
  direction_updated_[0] = false;
  direction_updated_[1] = false;
  three_axes[0] = tf::Vector3(1, 0, 0); //X
  three_axes[1] = tf::Vector3(0, 1, 0); //Y
  three_axes[2] = tf::Vector3(0, 0, 1); //Z
  hand_moving_direction_filters_.resize(2);
}

bool SweepHandGestureDetector::initialize()
{
  double d;
  int i;
  nh_private_.getParam("sweep_hand_gesture_windows", d);
  spinbox_window_->setValue(d);

  nh_private_.getParam("sweep_hand_gesture_gap", d);
  spinbox_gap_->setValue(d);

  nh_private_.getParam("filter_window_size", i);
  spinbox_filter_windows_size_->setValue(i);

  return true;
}

void SweepHandGestureDetector::drawHistory(MarkerArray& marker_array, const std::string& frame_id)
{
  Marker markers[2];

  for(int i = 0; i < last_hand_count_; i++)
  {
    markers[i].type = Marker::LINE_STRIP;
    markers[i].lifetime = ros::Duration(0.1);
    markers[i].header.frame_id = frame_id;
    markers[i].ns = "SweepHandGestureDetector";
    markers[i].id = i;
    markers[i].scale.x = 0.01;
    std::list<PositionsStamped>::iterator it;
    geometry_msgs::Point point;
    for (it = gesture_entries_.begin(); it != gesture_entries_.end(); it++)
    {
      point.x = it->second[i].getOrigin().x();
      point.y = it->second[i].getOrigin().y();
      point.z = it->second[i].getOrigin().z();
      markers[i].points.push_back(point);
    }

    markers[i].pose.position.x = 0;
    markers[i].pose.position.y = 0;
    markers[i].pose.position.z = 0;
    markers[i].pose.orientation.x = 0;
    markers[i].pose.orientation.y = 0;
    markers[i].pose.orientation.z = 0;
    markers[i].pose.orientation.w = 1;
    markers[i].color.r = (i == 0 ? 1.0 : 0.0);
    markers[i].color.g = (i == 0 ? 0.0 : 1.0);
    markers[i].color.b = 0.0;
    markers[i].color.a = 1.0;
    marker_array.markers.push_back(markers[i]);
  }
}

void SweepHandGestureDetector::drawResult(visualization_msgs::MarkerArray& marker_array,
                                                const std::string& frame_id)
{
  int marker_id = 0;
  Marker marker;
  marker.lifetime = ros::Duration(0.1);
  marker.header.frame_id = frame_id;
  marker.ns = "SweepHandGestureDetector_result";
  marker.type = Marker::ARROW;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;

  for (int i = 0; i < last_hand_count_; i++)
  {
    if (direction_updated_[i] && (hand_moving_direction_[i].length2() != 0))
    {
      marker.pose.position.x = gesture_entries_.front().second[i].getOrigin().x();
      marker.pose.position.y = gesture_entries_.front().second[i].getOrigin().y();
      marker.pose.position.z = gesture_entries_.front().second[i].getOrigin().z();

      Eigen::Quaternionf q_main;
      Eigen::Vector3f axis_main(hand_moving_direction_[i].x(),
                                hand_moving_direction_[i].y(),
                                hand_moving_direction_[i].z());
      q_main.setFromTwoVectors(Eigen::Vector3f(1, 0, 0), axis_main.normalized());

      marker.id = marker_id++;
      marker.scale.z = 0.3;
      marker.pose.orientation.x = q_main.x();
      marker.pose.orientation.y = q_main.y();
      marker.pose.orientation.z = q_main.z();
      marker.pose.orientation.w = q_main.w();
      marker.color.r = (i == 0 ? 1.0 : 0.0);
      marker.color.g = (i == 0 ? 0.0 : 1.0);
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker_array.markers.push_back(marker);
      direction_updated_[i] = false;
    }
  }
}

void SweepHandGestureDetector::addHandMessage(const hg_object_tracking::HandsConstPtr message)
{HandGesture gesture;

  if(message->hands.empty())
  {
    ROS_WARN_THROTTLE(1.0, "%s got an empty Hands message.", __FUNCTION__);
    return;
  }

  if(message->hands.size() > 2)
  {
    ROS_WARN_THROTTLE(1.0, "%s got more than two hands in Hands message."
                           " The first two hands will be used", __FUNCTION__);
    return;
  }

  int num_hand = message->hands.size();

  if(last_hand_count_ != num_hand)
  {
    ROS_DEBUG("Hand count changed from %d to %d", last_hand_count_, num_hand);
    onFilterWindowSizeChanged(spinbox_filter_windows_size_->value());
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
}

int SweepHandGestureDetector::lookForGesture()
{
  if (gesture_entries_.front().first - gesture_entries_.back().first < gesture_window_)
  {
    ROS_DEBUG("Not enough gesture entries");
    return 0;
  }

  int detected_gesture[2];
  for(int i = 0; i < last_hand_count_; i++)
  {
    detected_gesture[i] = detectOneHandGesture(i);
  }



  if(last_hand_count_ == 1)
  {
    return detected_gesture[0];
  }
  else
  {
    if(detected_gesture[0] == detected_gesture[1])
    {
      switch(detected_gesture[0])
      {
        case HandGesture::SWEEP_UP_ONE_HAND:
          return HandGesture::SWEEP_UP_TWO_HAND;
        case HandGesture::SWEEP_DOWN_ONE_HAND:
          return HandGesture::SWEEP_DOWN_TWO_HAND;
        case HandGesture::SWEEP_LEFT_ONE_HAND:
          return HandGesture::SWEEP_LEFT_TWO_HAND;
        case HandGesture::SWEEP_RIGHT_ONE_HAND:
          return HandGesture::SWEEP_RIGHT_TWO_HAND;
        case HandGesture::SWEEP_FORWARD_ONE_HAND:
          return HandGesture::SWEEP_FORWARD_TWO_HAND;
        case HandGesture::SWEEP_BACKWARD_ONE_HAND:
          return HandGesture::SWEEP_BACKWARD_TWO_HAND;
        default:
          return HandGesture::NOT_DETECTED;
      }
    }
    else
    {
      if (detected_gesture[0] == HandGesture::SWEEP_LEFT_ONE_HAND
          && detected_gesture[1] == HandGesture::SWEEP_RIGHT_ONE_HAND)
      {
        return HandGesture::SWEEP_OPEN_TWO_HAND;
      }

      if (detected_gesture[0] == HandGesture::SWEEP_RIGHT_ONE_HAND
          && detected_gesture[1] == HandGesture::SWEEP_LEFT_ONE_HAND)
      {
        return HandGesture::SWEEP_OPEN_TWO_HAND;
      }
    }
  }
  return HandGesture::NOT_DETECTED;
}

int SweepHandGestureDetector::detectOneHandGesture(int id)
{
  int detected_gesture = HandGesture::NOT_DETECTED;
  hand_moving_direction_[id] = getFilteredDirection(id, getHandMovingDirection(id));
  if(hand_moving_direction_[id].length2() == 0)
  {
    direction_updated_[id] = false;

  }
  else
  {
    direction_updated_[id] = true;
    double dot_products[3];
    double min_error = 1e6;
    int min_error_index = -1;
    for(int i = 0; i < 3; i++)
    {
      dot_products[i] = hand_moving_direction_[id].dot(three_axes[i]);

      if((1 - fabs(dot_products[i])) < min_error)
      {
        min_error = 1 - fabs(dot_products[i]);
        min_error_index = i;
      }
    }
    //ROS_INFO("hand %d %d %f", id, min_error_index, min_error);
    if(min_error > DIRECTION_EPSILON)
    {
      //ROS_INFO("Too much error");
    }
    else
    {
      switch(min_error_index)
      {
        case 0:
          if(dot_products[min_error_index] > 0)
          {
            ROS_INFO("[%d]: SWEEP_BACKWARD_ONE_HAND", id);
            detected_gesture = HandGesture::SWEEP_BACKWARD_ONE_HAND;
          }
          else
          {
            ROS_INFO("[%d]: SWEEP_FORWARD_ONE_HAND", id);
            detected_gesture = HandGesture::SWEEP_FORWARD_ONE_HAND;
          }
          break;
        case 1:
          if (dot_products[min_error_index] > 0)
          {
            ROS_INFO("[%d]: SWEEP_RIGHT_ONE_HAND", id);
            detected_gesture = HandGesture::SWEEP_RIGHT_ONE_HAND;
          }
          else
          {
            ROS_INFO("[%d]: SWEEP_LEFT_ONE_HAND", id);
            detected_gesture = HandGesture::SWEEP_LEFT_ONE_HAND;
          }
          break;
        case 2:
          if (dot_products[min_error_index] > 0)
          {
            ROS_INFO("[%d]: SWEEP_UP_ONE_HAND", id);
            detected_gesture = HandGesture::SWEEP_UP_ONE_HAND;
          }
          else
          {
            ROS_INFO("[%d]: SWEEP_DOWN_ONE_HAND", id);
            detected_gesture = HandGesture::SWEEP_DOWN_ONE_HAND;
          }
          break;
        default:
          break;
      }
    }
  }
  return detected_gesture;
}


bool SweepHandGestureDetector::detectTwoHandGesture()
{
  hand_moving_direction_[0] = getFilteredDirection(0, getHandMovingDirection(0));
  direction_updated_[0] = true;
  hand_moving_direction_[1] = getFilteredDirection(1, getHandMovingDirection(1));
  direction_updated_[1] = true;
  return true;
}

tf::Vector3 SweepHandGestureDetector::getHandMovingDirection(int id)
{
  if(gesture_entries_.size() < 3)
  {
    ROS_DEBUG_THROTTLE(1.0, "entries count is too short");
    return tf::Vector3(0,0,0);
  }

  tf::Vector3 dir_path = gesture_entries_.front().second[id].getOrigin() - gesture_entries_.back().second[id].getOrigin();
  if(dir_path.length() < 0.02)
  {
    ROS_DEBUG_THROTTLE(1.0, "distance is too short for hand %d", id);
    return tf::Vector3(0,0,0);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr motion_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::list<PositionsStamped>::iterator it;
  pcl::PointXYZ point;
  for (it = gesture_entries_.begin(); it != gesture_entries_.end(); it++)
  {
    motion_cloud->points.push_back(
        pcl::PointXYZ(it->second[id].getOrigin().x(), it->second[id].getOrigin().y(), it->second[id].getOrigin().z()));
  }
  motion_cloud->width = motion_cloud->points.size();
  motion_cloud->height = 1;
  motion_cloud->is_dense = true;

  pca_[id].setInputCloud(motion_cloud);
  ROS_DEBUG_STREAM(pca_[id].getEigenValues());
  ROS_DEBUG_STREAM(pca_[id].getEigenVectors());


  tf::Vector3 dir_eigen(pca_[id].getEigenVectors().coeff(0, 0), pca_[id].getEigenVectors().coeff(1, 0),
                        pca_[id].getEigenVectors().coeff(2, 0));
  dir_path.normalize();
  dir_eigen.normalize();
  double dot_value = dir_path.dot(dir_eigen);
  if (dot_value < 0)
    dir_eigen = -dir_eigen;
  return dir_eigen;
}

tf::Vector3 SweepHandGestureDetector::getFilteredDirection(int id, const tf::Vector3& latest_vector)
{
  if(latest_vector.length2() == 0)
  {
    hand_moving_direction_filters_[id].clear();
    return latest_vector;
  }
  hand_moving_direction_filters_[id].push_back(latest_vector);
  if((int)hand_moving_direction_filters_[id].size() > filter_windows_size_)
    hand_moving_direction_filters_[id].pop_front();
  std::list<tf::Vector3>::iterator it = hand_moving_direction_filters_[id].begin();
  tf::Vector3 tmp(0,0,0);
  while(it != hand_moving_direction_filters_[id].end())
  {
    tmp += *it;
    it++;
  }

  return (tmp * (1.0/hand_moving_direction_filters_[id].size())).normalized();
}

void SweepHandGestureDetector::addUI(QToolBox* tool_box)
{
  QLabel* label1 = new QLabel("Window");
  spinbox_window_ = new QDoubleSpinBox;
  spinbox_window_->setMinimum(0.01);
  spinbox_window_->setMaximum(10.0);
  spinbox_window_->setSingleStep(0.01);

  spinbox_gap_ = new QDoubleSpinBox;
  QLabel* label2 = new QLabel("Gap");
  spinbox_gap_->setMinimum(0.01);
  spinbox_gap_->setMaximum(10.0);
  spinbox_gap_->setSingleStep(0.01);

  QLabel* label3 = new QLabel("Filter");
  spinbox_filter_windows_size_ = new QSpinBox;
  spinbox_filter_windows_size_->setMinimum(1);
  spinbox_filter_windows_size_->setMaximum(50);
  spinbox_filter_windows_size_->setSingleStep(1);

  QGridLayout* grid_layout = new QGridLayout;
  grid_layout->addWidget(label1, 0, 0);
  grid_layout->addWidget(spinbox_window_, 0, 1);
  grid_layout->addWidget(label2, 1, 0);
  grid_layout->addWidget(spinbox_gap_, 1, 1);
  grid_layout->addWidget(label3, 2, 0);
  grid_layout->addWidget(spinbox_filter_windows_size_, 2, 1);

  QWidget *item_widget = new QWidget;
  item_widget->setLayout(grid_layout);
  tool_box->setSizePolicy(QSizePolicy(QSizePolicy::Maximum, QSizePolicy::Ignored));
  tool_box->setMinimumWidth(item_widget->sizeHint().width());
  tool_box->addItem(item_widget, tr("Sweep Hand Gesture"));

  connect(spinbox_window_, SIGNAL(valueChanged(double)), this, SLOT(onWindowTimeValueChanged(double)));
  connect(spinbox_gap_, SIGNAL(valueChanged(double)), this, SLOT(onGapTimeValueChanged(double)));
  connect(spinbox_filter_windows_size_, SIGNAL(valueChanged(int)), this, SLOT(onFilterWindowSizeChanged(int)));
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

void SweepHandGestureDetector::onFilterWindowSizeChanged(int d)
{
  QMutexLocker lock(&mutex_);
  filter_windows_size_  = d;
  gesture_entries_.clear();
  hand_moving_direction_filters_[0].clear();
  hand_moving_direction_filters_[1].clear();
}

