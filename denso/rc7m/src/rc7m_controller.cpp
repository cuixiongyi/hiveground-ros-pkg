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

#include <pluginlib/class_list_macros.h>
#include <rc7m/rc7m_controller.h>

PLUGINLIB_DECLARE_CLASS(hg_cpp, rc7m_controller, hg_plugins::RC7MController, hg::Controller)

using namespace hg_plugins;

RC7MController::RC7MController()
  : hg::Controller()
{
  //ROS_INFO_STREAM(__FUNCTION__);
}


RC7MController::~RC7MController()
{
  //ROS_INFO_STREAM(__FUNCTION__);
}

void RC7MController::initilize(hg::Node* node, const std::string& name)
{
  hg::Controller::initilize(node, name);

  ROS_INFO_STREAM("read " + name + " parameter from file");
  ROS_ASSERT(node_->node_handle_.getParam("controllers/" + name + "/rate", rate_));
  ROS_ASSERT(node_->node_handle_.getParam("controllers/" + name + "/ip", ip_));
  ROS_ASSERT(node_->node_handle_.getParam("controllers/" + name + "/port", port_));
}

void RC7MController::startup()
{

}

void RC7MController::update()
{

}

void RC7MController::shutdown()
{

}

bool RC7MController::active()
{
  return false;
}


