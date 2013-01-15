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

#ifndef HG_JOINT_H_
#define HG_JOINT_H_


#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <urdf/model.h>
#include <boost/thread.hpp>


namespace hg
{

class ControllerNode;
class Controller;

/**
 * A joint abstract class.
 */
class Joint
{
public:
  /**
   * A default constructor.
   */
  Joint()
    : last_update_(ros::Time::now()),
      position_(0),
      velocity_(0),
      touched_(false),
      desired_position_(0),
      last_commanded_position_(0)
  {
    //ROS_INFO_STREAM(__FUNCTION__);
  }

  /**
   * A destructor
   */
  virtual ~Joint()
  {
    //ROS_INFO_STREAM(__FUNCTION__);
  }

  /**
   * An initializing function.
   */
  virtual void initilize(hg::ControllerNode* node, const std::string& name)
  {
    //ROS_INFO_STREAM(__FUNCTION__);
    node_ = node;
    name_ = name;
  }

  /**
   * Load joint information from URDF.
   */
  virtual bool getJointInformationUrdf(const std::string& param)
  {
    return false;
  }

  /**
   * Interpolate joint position after dt.
   */
  virtual double interpolate(double dt) = 0;

  /**
   * Set feedback data from sensor (encoder, camera, ...).
   */
  virtual void setFeedbackData(double feedback) = 0;

  /**
   * Set joint position.
   */
  virtual double setPosition(double position) = 0;

  /**
   * Set relative joint position.
   */
  virtual double setPositionRelative(double position) = 0;

  /**
   * Get a diagnostics message for this joint.
   */
  virtual diagnostic_msgs::DiagnosticStatus getDiagnostics()
  {
    diagnostic_msgs::DiagnosticStatus message;
    message.name = name_;
    message.level = diagnostic_msgs::DiagnosticStatus::OK;
    message.message = "OK";
    return message;
  }


  hg::ControllerNode* node_;
  std::string name_;
  boost::recursive_mutex mutex_;

  hg::Controller* controller_;
  ros::Time last_update_;


  double position_;
  double velocity_;
  bool touched_;
  double desired_position_;
  double last_commanded_position_;

  boost::shared_ptr<const urdf::Joint> joint_info_;
};



}

#endif
