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

#ifndef HG_RC7M_JOINT_H_
#define HG_RC7M_JOINT_H_

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>


#include <hg_cpp/hg_joint.h>

namespace hg_plugins
{

class RC7MJoint : public hg::Joint
{
public:
  /**
   * A default constructor.
   */
  RC7MJoint();

  /**
   * A destructor.
   */
  ~RC7MJoint();

  /**
   * An initializing function.
   */
  void initilize(hg::Node* node, const std::string& name);

  /**
   * Load joint information from URDF.
   */
  bool getJointInformationUrdf();
  /**
   * Interpolate joint position after dt.
   */
  double interpolate(double dt);

  /**
   * Set feedback data from sensor (encoder, camera, ...).
   */
  void setFeedbackData(double feedback);

  /**
   * Set joint position.
   */
  double setPosition(double position);
  double setPositionRelative(double position);

  /**
   * Get a diagnostics message for this joint.
   */
  diagnostic_msgs::DiagnosticStatus getDiagnostics();

  //callback
  void callbackJointPosition(const std_msgs::Float64& position);
  void callbackJointPositionRelative(const std_msgs::Float64& position);
  void callbackJointPositionDegree(const std_msgs::Float64& position);
  void callbackJointVelocity(const std_msgs::Float64& velocity);

  ros::Subscriber subscriber_joint_position_;
  ros::Subscriber subscriber_joint_position_relative_;
  ros::Subscriber subscriber_joint_position_degree_;
  ros::Subscriber subscriber_joint_velocity_;

  double t0, t1;
};

}



#endif
