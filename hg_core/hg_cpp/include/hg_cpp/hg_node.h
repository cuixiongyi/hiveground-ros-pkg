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

#ifndef HG_NODE_H_
#define HG_NODE_H_

#include <signal.h>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <hg_cpp/hg_controller.h>
#include <hg_cpp/hg_joint.h>

namespace hg
{

/**
 * A node class.
 */
class Node
{
public:

  /**
   * A constructor.
   */
  Node();

  /**
   * A destructor.
   */
  virtual ~Node();

  /**
   * Main loop of the node.
   */
  virtual void run(sig_atomic_t volatile *is_shutdown);

  /**
   * Node message will be published from this function.
   */
  virtual void publish();


  ros::NodeHandle node_handle_; //!< ROS node handle?
  bool simulate_; //!< Is the node simulated?
  double loop_rate_; //!< mail loop frequency (Hz)
  double joint_publish_rate_; //!< joint state publish frequency (Hz)
  double diagnostic_publish_rate_; //!< diagnostic publish frequency (Hz)

  ros::Time next_joint_publish_time_;
  ros::Time next_diagnostic_publish_time_;

  pluginlib::ClassLoader<hg::Controller> controller_plugin_loader_;
  pluginlib::ClassLoader<hg::Joint> joint_plugin_loader_;
  std::vector<boost::shared_ptr<hg::Controller> > controllers_;
  std::vector<boost::shared_ptr<hg::Joint> > joints_;

  ros::Publisher publisher_joint_state_;
  ros::Publisher publisher_diagnostic_;
};

}


#endif /* HG_NODE_H_ */
