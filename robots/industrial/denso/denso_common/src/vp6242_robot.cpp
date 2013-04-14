/*
 * Copyright (c) 2013, HiveGround Co., Ltd.
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
 *      * Neither the name of the HiveGround Co., Ltd., nor the name of its
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
 *
 */


#include <denso_robots/vp6242_robot.h>
#include <sstream>

using namespace denso_common;
using namespace std;

VP6242Robot::VP6242Robot(const ros::NodeHandle& nh, const urdf::Model& urdf_model, const std::string& prefix)
  : nh_(nh),
    motor_on_(false),
    slave_mode_(0x102),
    pub_joint_state_(nh_, "/joint_states", 1)
{
  urdf_ = urdf_model;
  ROS_INFO_STREAM(urdf_.getName());

  joint_size_ = 6;
  for(int i = 1; i <= joint_size_; i++)
  {
    stringstream ss;
    ss << prefix << "joint" << i;
    boost::shared_ptr<const urdf::Joint> joint_info = urdf_.getJoint(ss.str());
    if(joint_info)
    {
      ROS_INFO_STREAM(joint_info->name << " lower limit: " << joint_info->limits->lower);
      ROS_INFO_STREAM(joint_info->name << " upper limit: " << joint_info->limits->upper);
      ROS_INFO_STREAM(joint_info->name << " velocity limit: " << joint_info->limits->velocity);
      ROS_INFO_STREAM(joint_info->name << " effort limit: " << joint_info->limits->effort);
      joint_name_.push_back(joint_info->name);
      joint_position_limit_upper_.push_back(joint_info->limits->upper);
      joint_position_limit_lower_.push_back(joint_info->limits->lower);
    }
  }

  if(joint_name_.empty())
  {
    ROS_FATAL("Got 0 joint for VP6242");
    return;
  }

  if(joint_name_.size() != 6)
  {
    ROS_FATAL("VP6242 has 6 DOF!");
    return;
  }

  joint_position_.resize(joint_size_);
  joint_position_last_.resize(joint_size_);
  joint_position_command_.resize(joint_size_);
  joint_velocity_.resize(joint_size_);
  joint_effort_.resize(joint_size_);
  command_degree_.resize(joint_size_);
  result_degree_.resize(joint_size_);

  pub_joint_state_.msg_.name.resize(joint_size_);
  pub_joint_state_.msg_.position.resize(joint_size_);
  pub_joint_state_.msg_.velocity.resize(joint_size_);
  pub_joint_state_.msg_.effort.resize(joint_size_);


  for(int i = 0; i < 6; i++)
  {
    js_interface_.registerJoint(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
    pj_interface_.registerJoint(js_interface_.getJointStateHandle(joint_name_[i]), &joint_position_command_[i]);
  }

  registerInterface(&js_interface_);
  registerInterface(&pj_interface_);
}

VP6242Robot::~VP6242Robot()
{

}

bool VP6242Robot::read()
{
  return true;
}

bool VP6242Robot::write()
{
  for(int i = 0; i < 6; i++)
  {
    if((joint_position_command_[i] > joint_position_limit_upper_[i]) ||
       (joint_position_command_[i] < joint_position_limit_lower_[i]))
    {
      ROS_ERROR("Joint %d out of limit", i);
      if(joint_position_command_[i] > joint_position_limit_upper_[i])
        joint_position_command_[i] = joint_position_limit_upper_[i];
      else
        joint_position_command_[i] = joint_position_limit_lower_[i];
    }
    command_degree_[i] = (joint_position_command_[i] * 180.0) / M_PI;
  }

  if(!setGetPosition(command_degree_, result_degree_))
    return false;

  double dt = (ros::Time::now() - last_update_).toSec();
  last_update_ = ros::Time::now();

  for (int i = 0; i < 6; i++)
  {
    joint_position_[i] = (result_degree_[i] * M_PI) / 180.0;
    joint_velocity_[i] = (joint_position_[i] - joint_position_last_[i]) / dt;
    joint_position_last_[i] = joint_position_[i];
  }

  static int count = 0;
  static double sum = 0;
  sum += dt;
  if(count++ == 1000)
  {
    ROS_INFO("Time: %f", sum);
    count = 0;
    sum = 0;
  }

  if((count % 10) == 0)
    publishJointState();

  return true;
}

bool VP6242Robot::start()
{
  if (!nh_.getParam("ip", ip_))
  {
    ROS_ERROR("%s needs ip setting", nh_.getNamespace().c_str());
    return false;
  }

  if (!nh_.getParam("port", port_))
  {
    ROS_ERROR("%s needs port setting", nh_.getNamespace().c_str());
    return false;
  }

  ROS_INFO("Connecting to %s:%s ...", ip_.c_str(), port_.c_str());
  bcap_ = boost::shared_ptr<BCapNet>(new BCapNet(ip_, port_,  BCapNet::BCAP_UDP));


  int mode = 0;
  uint16_t mode16 = 0;
  long result = 0;
  BCAP_HRESULT hr;

  hr = bcap_->ServiceStart();
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot start service", nh_.getNamespace().c_str());
    return false;
  }

  hr = bcap_->ControllerConnect("", "", "", "", &h_controller_);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot connect controller", nh_.getNamespace().c_str());
    return false;
  }

  mode16 = 2;
  result = 0;
  hr = bcap_->ControllerExecute2(h_controller_, "PutAutoMode", VT_I2, 1, &mode16, &result);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot execute PutAutoMode", nh_.getNamespace().c_str());
    return false;
  }

  result = 0;
  hr = bcap_->ControllerExecute2(h_controller_, "GetAutoMode", VT_EMPTY, 1, &mode, &result);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot execute GetAutoMode", nh_.getNamespace().c_str());
    return false;
  }
  if(result != 2)
  {
    ROS_ERROR("%s change mode failed", nh_.getNamespace().c_str());
    return false;
  }


  hr = bcap_->ControllerGetTask(h_controller_, "RobSlave", "", &h_task_);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot get RobSlave task", nh_.getNamespace().c_str());
    return false;
  }


  hr = bcap_->TaskStart(h_task_, 1, "");
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot start task RobSlave", nh_.getNamespace().c_str());
    return false;
  }


  //waiting for task to start
  ros::Duration(1.0).sleep();

  hr = bcap_->ControllerGetRobot(h_controller_, "ARM", "$IsIDHandle$", &h_robot_);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot get robot handle", nh_.getNamespace().c_str());
    return false;
  }


  hr = bcap_->RobotGetVariable(h_robot_, "@CURRENT_ANGLE", "", &h_joint_angle_variable_);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot get position variable handle", nh_.getNamespace().c_str());
    return false;
  }

  std::vector<float> joint_position;

  getJointPosition(joint_position);

  //update joint information
  for (int i = 0; i < 6; i++)
  {
    ROS_INFO("joint%d: %f", i, joint_position[i]);
    joint_position_command_[i] = (joint_position[i]  * M_PI) / 180.0;;
    joint_position_[i] = joint_position_command_[i];
    joint_position_last_[i] = joint_position_command_[i];
    joint_velocity_[i] = 0.0;
    joint_effort_[i] = 0.0;
  }

  last_update_ = ros::Time::now();
  last_published_joint_state_ = ros::Time::now();

  setMotor(true);



  result = 0;
  hr = bcap_->RobotExecute2(h_robot_, "slvChangeMode", VT_I4, 1, &slave_mode_, &result);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot execute slvChangeMode", nh_.getNamespace().c_str());
    return false;
  }


  result = 0;
  hr = bcap_->RobotExecute2(h_robot_, "slvGetMode", VT_EMPTY, 1, &mode, &result);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot execute  slvGetMode", nh_.getNamespace().c_str());
    return false;
  }
  if(result != slave_mode_)
  {
    ROS_ERROR("%s cannot change to slave mode", nh_.getNamespace().c_str());
    return false;
  }

  return true;
}

bool VP6242Robot::stop()
{
  if(!bcap_) return false;

  int mode = 0;
  long result = 0;
  BCAP_HRESULT hr;

  hr = bcap_->RobotExecute2(h_robot_, "slvChangeMode", VT_I4, 1, &mode, &result);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot execute slvChangeMode", nh_.getNamespace().c_str());
    return false;
  }

  setMotor(false);

  hr = bcap_->RobotRelease(h_robot_);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot release robot", nh_.getNamespace().c_str());
    return false;
  }

  hr = bcap_->TaskStop(h_task_, 1, "");
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot stop task", nh_.getNamespace().c_str());
    return false;
  }

  hr = bcap_->TaskRelease(h_task_);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot release task", nh_.getNamespace().c_str());
    return false;
  }

  uint16_t mode16 = 1;
  hr = bcap_->ControllerExecute2(h_controller_, "PutAutoMode", VT_I2, 1, &mode16, &result);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot execute PutAutoMode", nh_.getNamespace().c_str());
    return false;
  }

  result = 0;
  hr = bcap_->ControllerExecute2(h_controller_, "GetAutoMode", VT_EMPTY, 1, &mode, &result);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot execute GetAutoMode", nh_.getNamespace().c_str());
    return false;
  }
  if(result != 1)
  {
    ROS_ERROR("%s cannot change auto mode", nh_.getNamespace().c_str());
    return false;
  }

  hr = bcap_->ControllerDisconnect(h_controller_);
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot disconnect controller", nh_.getNamespace().c_str());
    return false;
  }

  hr = bcap_->ServiceStop();
  if(FAILED(hr))
  {
    ROS_ERROR("%s cannot stop service", nh_.getNamespace().c_str());
    return false;
  }

  bcap_.reset();
  return true;
}

bool VP6242Robot::setGetPosition(std::vector<float>& position, std::vector<float>& result)
{
  boost::mutex::scoped_lock lock(bcap_mutex_);
  if(position.size() != 6)
  {
    ROS_ERROR("VP6242 has 6 joints!");
    return false;
  }

  float out[7]; float in[7];
  out[6] = in[6] = 0;
  for(int i = 0; i < 6; i++)
  {
    out[i] = position[i];
  }

  BCAP_HRESULT hr;
  hr = bcap_->RobotExecute2(h_robot_, "slvMove", VT_R4 | VT_ARRAY, 7, out, in);
  if(!FAILED(hr))
  {
    result.resize(6);
    for(int i = 0; i < 6; i++)
    {
      result[i] = in[i];
    }
    return true;
  }
  return false;
}

bool VP6242Robot::getJointPosition(std::vector<float>& position)
{
  boost::mutex::scoped_lock lock(bcap_mutex_);
  position.resize(6);
  BCAP_HRESULT hr;
  float result[8];
  hr = bcap_->VariableGetValue(h_joint_angle_variable_, result);
  if(!FAILED(hr))
  {
    for(int i = 0; i < 6; i++)
    {
      position[i] = result[i];
    }
    return true;
  }
  return false;
}

bool VP6242Robot::setMotor(bool on)
{
  boost::mutex::scoped_lock lock(bcap_mutex_);
  motor_on_ = on;
  int mode = motor_on_ ? 1 : 0;
  int result = 0;
  BCAP_HRESULT hr = bcap_->RobotExecute2(h_robot_, "Motor", VT_I2, 1, &mode, &result);
  if(FAILED(hr)) return false;
  ros::Duration(5.0).sleep();
  return true;
}

void VP6242Robot::publishJointState()
{

  if (pub_joint_state_.trylock())
  {
    for (int i = 0; i < joint_size_; ++i)
    {
      pub_joint_state_.msg_.name[i] = joint_name_[i];
      pub_joint_state_.msg_.position[i] = joint_position_[i];
      pub_joint_state_.msg_.velocity[i] = joint_velocity_[i];
      pub_joint_state_.msg_.effort[i] = joint_effort_[i];
    }

    pub_joint_state_.msg_.header.stamp = ros::Time::now();
    pub_joint_state_.unlockAndPublish();
  }
}


