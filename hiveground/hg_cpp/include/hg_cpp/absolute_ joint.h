/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Imai Laboratory, Keio University.
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

#ifndef HG_ABSOLUTE_JOINT_H_
#define HG_ABSOLUTE_JOINT_H_

#include <hg_cpp/joint.h>
#include <std_msgs/Float64.h>

namespace hg_joint
{

/**
 * A absolute joint class.
 */
class AbsoluteJoint : public hg::Joint
{
public:

  /**
   * A constructor.
   */
  AbsoluteJoint();

  /**
   * A destructor.
   */
  virtual ~AbsoluteJoint();

  /**
   * An initializing function.
   */
  virtual void initilize(hg::ControllerNode* node, const std::string& name);

  /**
   * Load joint information from URDF.
   */
  virtual bool getJointInformationUrdf(const std::string& param);

  /**
   * Interpolate joint position after dt.
   */
  virtual double interpolate(double dt);

  /**
   * Interpolate joint position after dt with cubic spline.
   */
  virtual double interpolateCubic(double dt);

  /**
   * Set feedback data from sensor (encoder, camera, ...).
   */
  virtual void setFeedbackData(double feedback);

  /**
   * Set joint position.
   */
  virtual double setPosition(double position);

  /**
   * Set joint position with final time.
   */
  virtual double setPosition(double position, double final_time);

  /**
   * Set relative joint position.
   */
  virtual double setPositionRelative(double position);

  /**
   * Get a diagnostics message for this joint.
   */
  virtual diagnostic_msgs::DiagnosticStatus getDiagnostics();

  //callback
  virtual void callbackSetPosition(const std_msgs::Float64& position);
  virtual void callbackSetRelativePosition(const std_msgs::Float64& position);

  ros::Subscriber subscriber_joint_position_;
  ros::Subscriber subscriber_joint_position_relative_;



};



}

#endif
