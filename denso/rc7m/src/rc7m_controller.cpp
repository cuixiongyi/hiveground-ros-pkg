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

RC7MController::RC7MController() :
    hg::Controller(), bcap_(false), //do not need CRC check in network mode
    is_running_(false)
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
  ROS_INFO_STREAM("control rate " << rate_);
  ROS_INFO_STREAM("controller IP " << ip_);
  ROS_INFO_STREAM("controller port " << port_);

  //add joints
  XmlRpc::XmlRpcValue joints;
  node_->node_handle_.getParam("controllers/" + name_ + "/joints", joints);
  ROS_ASSERT(joints.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < joints.size(); i++)
  {
    std::string joint_name;
    ROS_ASSERT(joints[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    joint_name = static_cast<std::string>(joints[i]);

    //search joints in node
    bool found = false;
    std::vector<boost::shared_ptr<hg::Joint> >::iterator it;
    for (it = node_->joints_.begin(); it != node_->joints_.end(); it++)
    {
      if ((*it)->name_ == joint_name)
      {
        //set joint controller
        (*it)->controller_ = this;

        //push into container
        joints_.push_back(*it);
        found = true;
      }
    }

    //suicide if any joint cannot be found
    if (!found)
    {
      ROS_FATAL_STREAM("cannot find " + joint_name + " instance in node");
      ROS_BREAK();
    }

    ROS_INFO_STREAM("added " + joint_name + " to " + name_);
  }

}

void RC7MController::startup()
{
  if (!node_->simulate_)
  {
    BCAP_HRESULT hr;

    //open connection
    ROS_INFO_STREAM(name_ + ": connect to: " + ip_ +":" << port_);
    hr = bcap_.bCap_Open(ip_, port_);
    if (FAILED(hr))
    {
      ROS_BREAK();
    }

    //get controller
    ROS_INFO_STREAM(name_ + ": get controller handle");
    hr = bcap_.bCap_ControllerConnect("", "", "", "", &hController_);
    if (FAILED(hr))
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }

    {
      //put controller in external auto mode
      ROS_INFO_STREAM(name_ + ": put controller into external auto execution mode");
      uint16_t mode = 2;
      long result = 0;
      hr = bcap_.bCap_ControllerExecute2(hController_, "PutAutoMode", VT_I2, 1, &mode, &result);
      if (FAILED(hr))
      {
        bcap_.bCap_Close();
        ROS_BREAK();
      }

      //check auto execution mode
      ROS_INFO_STREAM(name_ + ": check auto execution mode");
      mode = 0;
      result = 0;
      hr = bcap_.bCap_ControllerExecute2(hController_, "GetAutoMode", VT_EMPTY, 1, &mode, &result);
      if (FAILED(hr))
      {
        bcap_.bCap_Close();
        ROS_BREAK();
      }
      ROS_INFO_STREAM(name_ + ": auto execution mode " << result);
      if (result != 2)
      {
        bcap_.bCap_Close();
        ROS_BREAK();
      }
    }

    //get slave mode task
    ROS_INFO_STREAM(name_ + ": get slave mode task");
    hr = bcap_.bCap_ControllerGetTask(hController_, "RobSlave", "", &hTask_);
    if (FAILED(hr))
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }

    //start task
    ROS_INFO_STREAM(name_ + ": start slave mode task");
    hr = bcap_.bCap_TaskStart(hTask_, 1, "");
    if (FAILED(hr))
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }
    ROS_INFO_STREAM(name_ + ": waiting to for task execution");
    ros::Duration(1.0).sleep(); //waiting to the execution

    //get robot
    ROS_INFO_STREAM(name_ + ": get robot");
    hr = bcap_.bCap_ControllerGetRobot(hController_, name_, "$IsIDHandle$", &hRobot_);
    if (FAILED(hr))
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }

    //get variable handles
    ROS_INFO_STREAM(name_ + ": get robot variable handles");
    hr = bcap_.bCap_RobotGetVariable(hRobot_, "@CURRENT_POSITION", "", &hPositionVariable);
    if (FAILED(hr))
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }
    hr = bcap_.bCap_RobotGetVariable(hRobot_, "@CURRENT_ANGLE", "", &hAngleVariable);
    if (FAILED(hr))
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }

    //read joint angle (position)
    float angle_variables[8];
    hr = bcap_.bCap_VariableGetValue(hAngleVariable, angle_variables);
    if (FAILED(hr))
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }
    for(int i = 0; i < 8; i++)
    {
      ROS_INFO_STREAM(name_ + ": J" << i << " " << angle_variables[i] << " degree");
    }



    {
      //set slave mode
      int mode = 258;
      long result;
      ROS_INFO_STREAM(name_ + ": set robot to slave mode");
      hr = bcap_.bCap_RobotExecute2(hRobot_, "slvChangeMode", VT_I4, 1, &mode, &result);
      if (FAILED(hr))
      {
        bcap_.bCap_Close();
        ROS_BREAK();
      }

      //check slave mode
      mode = 0;
      result = 0;
      ROS_INFO_STREAM(name_ + ": check robot mode");
      hr = bcap_.bCap_RobotExecute2(hRobot_, "slvGetMode", VT_EMPTY, 1, &mode, &result);
      if (FAILED(hr))
      {
        bcap_.bCap_Close();
        ROS_BREAK();
      }
      ROS_INFO_STREAM(name_ + ": mode " << result);
      if (result != 258)
      {
        bcap_.bCap_Close();
        ROS_BREAK();
      }

    }
  }

  //create control thread
  control_thread_ = boost::thread(&RC7MController::control, this);
  is_running_ = true;

  ROS_INFO_STREAM(name_ + ": started");
}

void RC7MController::update()
{
  //ROS_INFO_STREAM(name_ + " updated");
}

void RC7MController::control()
{
  ros::Rate rate(rate_);
  std::vector<boost::shared_ptr<hg::Joint> >::iterator it;
  double command[7], command_degree[7];
  while (is_running_)
  {
    ROS_INFO_STREAM_THROTTLE(1.0, name_ << " " << __FUNCTION__);

    //get position of each joint
    int i = 0;
    for (it = joints_.begin(); it != joints_.end(); it++)
    {
      command[i] = (*it)->interpolate(1.0 / rate_);
      command_degree[i] = (command[i] * 180.0) / M_PI;
      i++;
    }

    if (!node_->simulate_)
    {
      //set next joint positions
      setJoint();

      //get feedback from robot's angle variable
      getJointFeedback();
    }
    else
    {
      int i = 0;
      for (it = joints_.begin(); it != joints_.end(); it++)
      {
        (*it)->set_feedback_data(command[i]);
        i++;
      }
    }
    rate.sleep();

  }

}

void RC7MController::shutdown()
{
  is_running_ = false;
  control_thread_.join();

  if (!node_->simulate_)
  {
    BCAP_HRESULT hr;
    //turn off motor

    //go back to normal mode
    int mode = 0;
    long result;
    ROS_INFO_STREAM(name_ + ": set robot to normal mode");
    hr = bcap_.bCap_RobotExecute2(hRobot_, "slvChangeMode", VT_I4, 1, &mode, &result);
    if (FAILED(hr))
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }

    //check slave mode
    mode = 0;
    result = 0;
    ROS_INFO_STREAM(name_ + ": check robot mode");
    hr = bcap_.bCap_RobotExecute2(hRobot_, "slvGetMode", VT_EMPTY, 1, &mode, &result);
    if (FAILED(hr))
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }
    ROS_INFO_STREAM(name_ + ": mode " << result);
    if (result != 0)
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }

    //release variables
    ROS_INFO_STREAM(name_ + ": get robot variable handles");
    hr = bcap_.bCap_VariableRelease(hPositionVariable);
    if (FAILED(hr))
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }
    hr = bcap_.bCap_VariableRelease(hAngleVariable);
    if (FAILED(hr))
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }

    //release robot
    ROS_INFO_STREAM(name_ + ": release robot");
    hr = bcap_.bCap_RobotRelease(hRobot_);
    if (FAILED(hr))
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }

    //stop slave task
    ROS_INFO_STREAM(name_ + ": stop slave task");
    hr = bcap_.bCap_TaskStop(hTask_, 1, "");
    if (FAILED(hr))
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }

    //release task
    ROS_INFO_STREAM(name_ + ": release task");
    hr = bcap_.bCap_TaskRelease(hTask_);
    if (FAILED(hr))
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }

    //disconnect controller
    ROS_INFO_STREAM(name_ + ": disconnect controller");
    hr = bcap_.bCap_ControllerDisconnect(hController_);
    if (FAILED(hr))
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }

    //close network connection
    ROS_INFO_STREAM(name_ + ": close network connection");
    hr = bcap_.bCap_Close();
    if (FAILED(hr))
    {
      ROS_BREAK();
    }
  }


  ROS_INFO_STREAM(name_ + " shutdown");
}

bool RC7MController::active()
{
  return false;
}

bool RC7MController::setMotor(bool on_off)
{
  if (node_->simulate_)
    return true;

  boost::unique_lock<boost::mutex> lock(control_mutex_);

  BCAP_HRESULT hr;
  int mode = (on_off) ? 1 : 0;
  long result;
  hr = bcap_.bCap_RobotExecute2(hRobot_, "Motor", VT_I2, 1, &mode, &result);
  if (FAILED(hr))
  {
    ROS_ERROR_STREAM(name_ + ": set motor fail");
    return false;
  }
  else
  {
    if (on_off)
      ROS_INFO_STREAM(name_ + ": set motor on");
    else
      ROS_INFO_STREAM(name_ + ": set motor off");
    return true;
  }
}

void RC7MController::setJoint()
{

}

void RC7MController::getJointFeedback()
{
  float angle_variables[8];
  BCAP_HRESULT hr;
  hr = bcap_.bCap_VariableGetValue(hAngleVariable, angle_variables);
  if (FAILED(hr))
  {
    bcap_.bCap_Close();
    ROS_BREAK();
  }

  std::vector<boost::shared_ptr<hg::Joint> >::iterator it;
  int i = 0;
  for (it = joints_.begin(); it != joints_.end(); it++)
  {
    //convert to radian
    (*it)->set_feedback_data((angle_variables[i] * M_PI)/180.0);
    i++;
  }
}

