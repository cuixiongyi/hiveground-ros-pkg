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

#ifndef GESTURE_H_
#define GESTURE_H_

#include <ros/ros.h>
#include <qobject.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <hg_object_tracking/Hands.h>
#include <QToolBox>
#include <QMutex>
#include <QDoubleSpinBox>


namespace hg_hand_interaction
{

enum Gesture
{
  NOT_DETECTED = -1,
  SWEEP_LEFT_ONE_HAND,
  SWEEP_LEFT_TWO_HAND,
  SWEEP_RIGHT_ONE_HAND,
  SWEEP_RIGHT_TWO_HAND,
  SWEEP_UP_ONE_HAND,
  SWEEP_UP_TWO_HAND,
  SWEEP_DOWN_ONE_HAND,
  SWEEP_DOWN_TWO_HAND,
  SWEEP_FORWARD_ONE_HAND,
  SWEEP_FORWARD_TWO_HAND,
  SWEEP_BACKWARD_ONE_HAND,
  SWEEP_BACKWARD_TWO_HAND,
  OPEN_TWO_HAND,
  CLOSE_TWO_HAND
};

class GestureDetector : public QObject
{
  Q_OBJECT
public:
  typedef std::vector<tf::Transform> Positions;
  typedef std::pair<ros::Time, Positions> PositionsStamped;


  GestureDetector(ros::NodeHandle& nh_private);
  virtual ~GestureDetector() { }

  virtual bool initialize() = 0;

  virtual void drawHistory(visualization_msgs::MarkerArray& marker_array,
                              const std::string& frame_id = "base_link") = 0;
  virtual void drawResult(visualization_msgs::MarkerArray& marker_array,
                             const std::string& frame_id = "base_link") = 0;
  virtual bool lookForGesture() = 0;

  virtual void addUI(QToolBox* tool_box) = 0;

protected:
  ros::NodeHandle& nh_private_;
  ros::Duration gesture_window_;
  ros::Duration gesture_gap_;
  ros::Time last_detected_time_;
  std::list<PositionsStamped> gesture_entries_;
  Gesture detected_gesture_;
  QMutex mutex_;
  QDoubleSpinBox* spinbox_window_;
  QDoubleSpinBox* spinbox_gap_;
};

class HandGestureDetector : public GestureDetector
{
  Q_OBJECT
public:
  HandGestureDetector(ros::NodeHandle& nh_private)
    : GestureDetector(nh_private)
  { }
  virtual ~HandGestureDetector() { }

  virtual void addHandMessage(const hg_object_tracking::HandsConstPtr message) = 0;

protected:

};

}


#endif /* GESTURE_H_ */
