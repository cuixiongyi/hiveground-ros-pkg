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

#include <std_msgs/Bool.h>

#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include <queue>

#include <hg_cpp/follow_joint_controller.h>
#include <bcap/bcap_net.h>

namespace hg_controller
{

class RC7MController : public hg::FollowJointController
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
  void initilize(hg::ControllerNode* node, const std::string& name);

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
  void turnOnMotor(bool on);
  void getJointFeedback(std::vector<float>& joint_angle);
  void startSlaveMode(bool restart);
  void clearError(int code);

  void callbackSetMotor(const std_msgs::Bool& on);

private:
  boost::shared_ptr<BCapNet> bcap_;
  std::string address_;
  std::string port_;
  BCapNet::ConnectingMode mode_;
  int slave_mode_;


  bool is_running_;
  boost::thread control_thread_;
  boost::mutex control_mutex_;
  bool motor_on_;

  uint32_t h_controller_;
  uint32_t h_task_;
  uint32_t h_robot_;
  uint32_t h_joint_angle_variable_;

  ros::Subscriber subscriber_motor_;


};

}//hg_controller

#endif

