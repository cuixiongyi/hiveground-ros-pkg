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

#ifndef GESTURE_DETECTOR_H_
#define GESTURE_DETECTOR_H_

#include <ros/ros.h>
#include <qtpropertymanager.h>
#include <qteditorfactory.h>
#include <qttreepropertybrowser.h>
#include <ve_node.h>
#include <kinect_msgs/Skeletons.h>
#include <hg_object_tracking/Hands.h>
#include <visualization_msgs/MarkerArray.h>
#include <hg_user_interaction/Gestures.h>

namespace hg_user_interaction
{

class GestureDetectorItem;

typedef QList<GestureDetectorItem*> GestureDetectorItemList;

class GestureDetectorItem : public ve::Node
{
  Q_OBJECT
public:
  GestureDetectorItem(ros::NodeHandle& nh_private);
  GestureDetectorItem(ros::NodeHandle& nh_private, const QRectF& rect);
  ~GestureDetectorItem();

  virtual bool initialize() = 0;

  virtual void addSkeletonsMessage(const kinect_msgs::SkeletonsConstPtr& skeletons) = 0;
  virtual void addHandsMessage(const hg_object_tracking::HandsConstPtr& hands) = 0;

  virtual void drawHistory(visualization_msgs::MarkerArray& marker_array, const std::string& frame_id) = 0;
  virtual void drawResult(visualization_msgs::MarkerArray& marker_array, const std::string& frame_id) = 0;

  virtual int lookForGesture(hg_user_interaction::Gesture& gesture) = 0;

  enum RttiValue
  {
    Rtti_Item = 0,
    Rtti_HandPushPull,
    Rtti_BodyMovement,
    Rtti_ElbowSwitch
  };

  virtual int rtti() const
  {
    return RTTI;
  }

  VE_GETSETG(bool, draw_history_, DrawHistory);
  VE_GETSETG(bool, draw_result_, DrawResult);

protected:
  ros::NodeHandle& nh_private_;
  bool draw_history_;
  bool draw_result_;

private:
  Q_DISABLE_COPY(GestureDetectorItem);
  static int RTTI;
};

}

#endif /* GESTURE_DETECTOR_H_ */
