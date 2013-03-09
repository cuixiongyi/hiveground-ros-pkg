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

#ifndef GESTURE_HAND_PUSHPULL_H_
#define GESTURE_HAND_PUSHPULL_H_

#include <hg_user_interaction/gesture_detector.h>
#include <tf/tf.h>

namespace hg_user_interaction
{

class GestureDetectorHandPushPull : public GestureDetectorItem
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


  GestureDetectorHandPushPull(ros::NodeHandle& nh_private);
  GestureDetectorHandPushPull(ros::NodeHandle& nh_private, const QRectF& rect);
  ~GestureDetectorHandPushPull();

  bool initialize();
  void addSkeletonsMessage(const kinect_msgs::SkeletonsConstPtr& skeletons) { }
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
  int getStateAuto(int hand, const tf::Vector3& vec_to_hand);


protected:
  double r1_, r2_, r3_;
  double time_out_;
  double activating_time_;

  int num_hands_;
  bool left_hand_;
  int current_state_[2];
  int last_state_[2];
  tf::Transform last_hand_positions_[2];
  tf::Transform activated_hand_positions_[2];
  tf::Vector3 center_positions_[2];
  ros::Time start_leaving_time_[2];
  ros::Time start_activating_time_[2];
  bool is_moving_[2];
  tf::Vector3 three_axes_[3];



private:
  static int RTTI;

};




}



#endif /* GESTURE_HAND_PUSHPULL_H_ */
