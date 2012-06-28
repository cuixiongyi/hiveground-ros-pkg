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

#ifndef HG_RC7M_CONTROLLER_H_
#define HG_RC7M_CONTROLLER_H_

#include <boost/thread.hpp>

#include <hg_cpp/hg_controller.h>
#include <bcap/bcap.h>
#include <rc7m/rc7m_joint.h>

namespace hg_plugins
{

class RC7MController : public hg::Controller
{

public:
  /**
   * A default constructor.
   */
  RC7MController();

  /**
   * A destructor.
   *
   */
  ~RC7MController();

  /**
   * An initializing function.
   */
  void initilize(hg::Node* node, const std::string& name);

  /**
   * Start the controller, do any hardware setup needed.
   */
  void startup();

  /**
   * Do any read/writes to device.
   */
  void update();

  /**
   * Control loop of RC7M.
   * Execute in separated thread.
   */
  void control();

  /**
   * Stop the controller, do any hardware shutdown needed.
   */
  void shutdown();

  /**
   * Is the controller actively sending commands to joints/robots?
   */
  bool active();

private:
  /**
   * Turn motor on/off.
   */
  BCAP_HRESULT setMotor(bool on_off);

  /**
   * Get feedback from robot and update joints.
   */
  BCAP_HRESULT getJointFeedback(bool set_desired_position_ = false);


public:
  BCap bcap_;
  std::string ip_;
  int port_;
  uint32_t hController_;
  uint32_t hTask_;
  uint32_t hRobot_;
  uint32_t hPositionVariable;
  uint32_t hAngleVariable;
  bool motor_on_;
  bool slave_mode_on_;

  bool is_running_;
  boost::thread control_thread_;
  boost::mutex control_mutex_;
};

}//hg_plugins

#endif
