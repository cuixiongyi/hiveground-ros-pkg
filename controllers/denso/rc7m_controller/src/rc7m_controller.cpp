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

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <rc7m_controller/rc7m_controller.h>
#include <hg_cpp/controller_node.h>

using namespace hg_controller;

PLUGINLIB_DECLARE_CLASS(controller_plugins, RC7MController, hg_controller::RC7MController, hg::Controller)

RC7MController::RC7MController() :
    hg::FollowJointController(), motor_on_(false)
{

}

/**
 * A destructor.
 *
 */
RC7MController::~RC7MController()
{
}

/**
 * An initializing function.
 */
void RC7MController::initilize(hg::ControllerNode* node, const std::string& name)
{
  FollowJointController::initilize(node, name);

  ROS_ASSERT(node_->nh_private_.getParam("controllers/" + name_ + "/rate", rate_));
  ROS_ASSERT(node_->nh_private_.getParam("controllers/" + name_ + "/address", address_));
  node_->nh_private_.param("controllers/" + name_ + "/port", port_, std::string("5007"));
  ROS_ASSERT(node_->nh_private_.getParam("controllers/" + name_ + "/mode", mode_));
  ROS_INFO_STREAM("controller address " << address_);
  ROS_INFO_STREAM("controller port " << port_);
  ROS_INFO_STREAM("communication mode " << mode_);

  if (!node_->is_simulated_)
  {
    //bcap_ = boost::shared_ptr<BCapSerial>(new BCapNet(port_, baud_));
  }

  subscriber_motor_ = node_->nh_private_.subscribe(name_ + "/motor", 1, &RC7MController::callbackSetMotor, this);
}

/**
 * Start the controller, do any hardware setup needed.
 */
void RC7MController::startup()
{
  if (!node_->is_simulated_)
  {
    int mode = 0;
    long result = 0;

    BCAP_HRESULT hr;
    hr = bcap_->ControllerConnect("", "", "", "", &h_controller_);
    ROS_ASSERT(!FAILED(hr));

    hr = bcap_->ControllerGetRobot(h_controller_, "RC7MController", "$IsIDHandle$", &h_robot_);
    ROS_ASSERT(!FAILED(hr));

    turnOnMotor(false);

    mode = 258;
    hr = bcap_->RobotExecute2(h_robot_, "slvChangeMode", VT_I4, 1, &mode, &result);

    hr = bcap_->RobotGetVariable(h_robot_, "@CURRENT_ANGLE", "", &h_joint_angle_variable_);
    ROS_ASSERT(!FAILED(hr));

    std::vector<float> joint_angle;
    getJointFeedback(joint_angle);

    //update joint information
    std::vector<boost::shared_ptr<hg::Joint> >::iterator it;
    int i = 0;
    double radian = 0;
    for (it = joints_.begin(); it != joints_.end(); it++)
    {
      //convert to radian
      radian = (joint_angle[i] * M_PI) / 180.0;
      (*it)->desired_position_ = radian;
      (*it)->last_commanded_position_ = radian;
      (*it)->setFeedbackData(radian);
      i++;
    }
  }

  //create control thread
  control_thread_ = boost::thread(&RC7MController::control, this);
  is_running_ = true;

  //start action server
  action_server_ = hg::FollowJointTrajectoryActionServerPtr(
      new hg::FollowJointTrajectoryActionServer(
          node_->nh_private_, "follow_joint_trajectory",
          boost::bind(&FollowJointController::followJointGoalActionCallback, this, _1), false));
  action_server_->start();

}

/**
 * Do any read/writes to device.
 */
void RC7MController::update()
{

}

/**
 * Control loop of RC7M.
 * Execute in separated thread.
 */
void RC7MController::control()
{
  ros::Rate rate(rate_);
  std::vector<boost::shared_ptr<hg::Joint> >::iterator it;
  double command[7];
  float command_degree[7], command_result[7];
  std::vector<float> joint_angle;
  BCAP_HRESULT hr;
  while (is_running_)
  {
    ros::Time startTime = ros::Time(ros::WallTime::now().toSec());
    int i = 0;
    for (it = joints_.begin(); it != joints_.end(); it++)
    {

      command[i] = (*it)->interpolate(1.0 / rate_);
      command_degree[i] = (command[i] * 180.0) / M_PI;
      i++;
    }

    if (!node_->is_simulated_)
    {
      if (motor_on_)
      {
        boost::unique_lock<boost::mutex> lock(control_mutex_);
        hr = bcap_->RobotExecute2(h_robot_, "slvMove", VT_R4 | VT_ARRAY, 7, command_degree, command_result);
        ROS_ASSERT(!FAILED(hr));
      }

      getJointFeedback(joint_angle);
      //update joint information
      int i = 0;
      double radian = 0;
      for (it = joints_.begin(); it != joints_.end(); it++)
      {
        //convert to radian
        radian = (joint_angle[i] * M_PI) / 180.0;
        if (!motor_on_)
        {
          //revent abruptly move back to a position before turning off the motor
          (*it)->desired_position_ = radian;
          (*it)->last_commanded_position_ = radian;
        }

        (*it)->setFeedbackData(radian);
        i++;
      }
    }
    else
    {
      int i = 0;
      for (it = joints_.begin(); it != joints_.end(); it++)
      {
        (*it)->setFeedbackData(command[i]);
        i++;
      }
    }
    rate.sleep();
  }
}

/**
 * Stop the controller, do any hardware shutdown needed.
 */
void RC7MController::shutdown()
{
  is_running_ = false;
  control_thread_.join();

  if (!node_->is_simulated_)
  {
    int mode = 0;
    long result = 0;
    BCAP_HRESULT hr;
    hr = bcap_->RobotExecute2(h_robot_, "Motor", VT_I2, 1, &mode, &result);
    ROS_ASSERT(!FAILED(hr));

    hr = bcap_->RobotRelease(h_robot_);
    ROS_ASSERT(!FAILED(hr));

    hr = bcap_->ControllerDisconnect(h_controller_);
    ROS_ASSERT(!FAILED(hr));
  }
}

/**
 * Is the controller actively sending commands to joints/robots?
 */
bool RC7MController::active()
{
  return false;
}

void RC7MController::turnOnMotor(bool on)
{
  motor_on_ = on;
  int mode = motor_on_ ? 1 : 0;
  int result = 0;
  boost::unique_lock<boost::mutex> lock(control_mutex_);
  BCAP_HRESULT hr = bcap_->RobotExecute2(h_robot_, "Motor", VT_I2, 1, &mode, &result);
  ROS_ASSERT(!FAILED(hr));
}

void RC7MController::getJointFeedback(std::vector<float>& joint_angle)
{
  if (joint_angle.size() != 8)
    joint_angle.resize(8);
  BCAP_HRESULT hr;
  boost::unique_lock<boost::mutex> lock(control_mutex_);
  hr = bcap_->VariableGetValue(h_joint_angle_variable_, joint_angle.data());
  ROS_ASSERT(!FAILED(hr));
}

void RC7MController::callbackSetMotor(const std_msgs::Bool& on)
{
  if (!node_->is_simulated_)
  {
    turnOnMotor(on.data);
  }
  else
  {
    motor_on_ = on.data;
  }
}

