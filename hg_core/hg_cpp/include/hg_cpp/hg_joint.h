/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Mahisorn Wongphati
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
 *      * Neither the name of the HiveGround Co.,Ltd. , nor the name of its
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
 */

#ifndef HG_JOINT_H_
#define HG_JOINT_H_


#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <urdf/model.h>

#include <hg_cpp/hg_node.h>
#include <hg_cpp/hg_controller.h>

#include <string>

namespace hg
{

/**
 * A joint abstract class.
 */
class Joint
{
public:

  /**
   * A constructor.
   */
  Joint(hg::Node* node, const std::string& name) :
      node_(node),
      last_update_(ros::Time::now()),
      name_(name),
      lower_limit_(0),
      upper_limit_(0),
      velocity_limit_(0),
      position_(0),
      velocity_(0)

  {
  }

  virtual ~Joint()
  {
  }

  virtual bool get_joint_info_urdf()
  {
    return false;
  }

  virtual double interpolate(double dt) = 0;

  virtual void set_feedback_data(double feedback) = 0;

  virtual double set_position(double position) = 0;

  virtual diagnostic_msgs::DiagnosticStatus get_diagnostics()
  {
    diagnostic_msgs::DiagnosticStatus message;
    message.name = name_;
    message.level = diagnostic_msgs::DiagnosticStatus::OK;
    message.message = "OK";
    return message;
  }

  hg::Node* node_;
  hg::Controller* controller_;
  ros::Time last_update_;

  std::string name_;
  int id_;
  double lower_limit_;
  double upper_limit_;
  double velocity_limit_;
  double position_;
  double velocity_;
};

}

#endif
