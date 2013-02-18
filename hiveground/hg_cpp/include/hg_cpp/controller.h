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

#ifndef HG_CONTROLLER_H_
#define HG_CONTROLLER_H_

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

namespace hg
{
class ControllerNode;
class Joint;
/**
 * A controller abstract class.
 */
class Controller
{
public:
  /**
   * A default constructor.
   */
  Controller()
    : pause_(true),
      rate_(10.0),
      is_active_(false)
  { }

  /**
   * A destructor.
   *
   */
  virtual ~Controller()
  { }

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
   * Start the controller, do any hardware setup needed.
   */
  virtual void startup() = 0;

  /**
     * Stop the controller, do any hardware shutdown needed.
   */
  virtual void shutdown() = 0;

  /**
   * Do any read/writes to device.
   */
  virtual void update() = 0;

  /**
   * Control loop.
   * Execute in separated thread or call in update() to use node mail loop.
   */
  virtual void control() = 0;

  /**
   * Is the controller actively sending commands to joints/robots?
   */
  virtual bool active() = 0;

  /**
   * Get a diagnostics message for this controller.
   */
  virtual diagnostic_msgs::DiagnosticStatus get_diagnostics()
  {
    diagnostic_msgs::DiagnosticStatus message;
    message.name = name_;
    message.level = diagnostic_msgs::DiagnosticStatus::OK;
    message.message = "OK";
    return message;
  }

  hg::ControllerNode* node_;
  std::string name_;
  bool pause_;
  double rate_;
  bool is_active_;
  std::vector<boost::shared_ptr<hg::Joint> > joints_;
};



}


#endif
