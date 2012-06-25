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

#ifndef HG_CONTROLLER_H_
#define HG_CONTROLLER_H_

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#include <hg_cpp/hg_node.h>

namespace hg
{

/**
 * A controller abstract class.
 */
class Controller
{
public:

  /**
   * A constructor.
   * @param nod Node instance.
   * @param name the controller name.
   */
  Controller(hg::Node* node, const std::string& name)
    : node_(node), name_(name)
  {
  }

  /**
   * A destructor.
   *
   */
  virtual ~Controller()
  {
  }
  ;

  /**
   * Start the controller, do any hardware setup needed.
   */
  virtual void startup() = 0;

  /**
   * Do any read/writes to device.
   */
  virtual void update() = 0;

  /**
   * Stop the controller, do any hardware shutdown needed.
   */
  virtual void shutdown() = 0;

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

  hg::Node* node_;
  std::string name_;
  bool pause_;



};



}


#endif
