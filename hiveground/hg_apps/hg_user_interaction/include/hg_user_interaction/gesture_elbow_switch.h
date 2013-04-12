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
 * gesture_elbow_switch.h
 *
 *  Created on: Apr 12, 2013
 *      Author: chang
 */

#ifndef GESTURE_ELBOW_SWITCH_H_
#define GESTURE_ELBOW_SWITCH_H_

#include <hg_user_interaction/gesture_detector.h>
#include <tf/tf.h>

namespace hg_user_interaction
{

class GestureDetectorElbowSwitch : public GestureDetectorItem
{
  Q_OBJECT
public:
  GestureDetectorElbowSwitch(ros::NodeHandle& nh_private);
  GestureDetectorElbowSwitch(ros::NodeHandle& nh_private, const QRectF& rect);
  ~GestureDetectorElbowSwitch();


  bool initialize();
  void addSkeletonsMessage(const kinect_msgs::SkeletonsConstPtr& skeletons);
  void addHandsMessage(const hg_object_tracking::HandsConstPtr& hands) { }

  void drawHistory(visualization_msgs::MarkerArray& marker_array, const std::string& frame_id);
  void drawResult(visualization_msgs::MarkerArray& marker_array, const std::string& frame_id);
  int lookForGesture(hg_user_interaction::Gesture& gesture);

  virtual int rtti() const
  {
    return RTTI;
  }

  VE_GETSETG(int, max_frame_, MaxFrame);
  VE_GETSETG(double, min_length_, MinLength);
  VE_GETSETG(double, max_detecting_rate_, MaxDetectRate);

protected:
  bool skeleton_updated_;
  std::list<tf::Vector3> l_elbow_path_;
  std::list<tf::Vector3> r_elbow_path_;

  int max_frame_;
  double min_length_;
  double max_detecting_rate_; //gesture / s
  ros::Time last_detected_time_;




private:
  static int RTTI;


};

}


#endif /* GESTURE_ELBOW_SWITCH_H_ */
