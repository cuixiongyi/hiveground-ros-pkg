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
 *       File: gesture_body_movement.h
 * Created on: Mar 11, 2013
 *     Author: Mahisorn Wongphati
 */

#ifndef GESTURE_BODY_MOVEMENT_H_
#define GESTURE_BODY_MOVEMENT_H_

#include <hg_user_interaction/gesture_detector.h>
#include <tf/tf.h>

namespace hg_user_interaction
{

class GestureBodyMovement : public GestureDetectorItem
{
  Q_OBJECT
public:
  enum
  {
    IDEL,
    ENTERING,
    ACTIVATING,
    ACTIVATED,
    LEAVING
  };

  GestureBodyMovement(ros::NodeHandle& nh_private);
  GestureBodyMovement(ros::NodeHandle& nh_private, const QRectF& rect);
  ~GestureBodyMovement();

  bool initialize();
  void addSkeletonsMessage(const kinect_msgs::SkeletonsConstPtr& skeletons);
  void addHandsMessage(const hg_object_tracking::HandsConstPtr& hands);

  void drawHistory(visualization_msgs::MarkerArray& marker_array, const std::string& frame_id);
  void drawResult(visualization_msgs::MarkerArray& marker_array, const std::string& frame_id);
  int lookForGesture(hg_user_interaction::Gesture& gesture);

  virtual int rtti() const
  {
    return RTTI;
  }

  VE_GETSETG(double, r1_, R1);
  VE_GETSETG(double, r2_, R2);
  VE_GETSETG(double, r3_, R3);
  VE_GETSETG(double, time_out_, TimeOut);
  VE_GETSETG(double, activating_time_, ActivatingTime);

protected:
  int getState(const tf::Vector3& vec_to_body);



protected:
  double r1_, r2_, r3_;
  double time_out_;
  double activating_time_;

  int current_state_;
  int last_state_;
  tf::Transform last_position_;
  tf::Transform activated_position_;
  tf::Vector3 center_position_;
  ros::Time start_leaving_time_;
  ros::Time start_activating_time_;
  bool is_moving_;
  tf::Vector3 three_axes_[3];
  bool skeleton_updated_;




private:
  static int RTTI;


};

}




#endif /* GESTURE_BODY_MOVEMENT_H_ */
