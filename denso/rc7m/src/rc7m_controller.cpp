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
#include <algorithm>

PLUGINLIB_DECLARE_CLASS(hg_cpp, rc7m_controller, hg_plugins::RC7MController, hg::Controller)

using namespace hg_plugins;

#define USE_SLAVE_MODE 1
#define SLAVE_MODE 0x102

RC7MController::RC7MController() :
    hg::Controller(),
    bcap_(false, false), //no CRC / TCP
    motor_on_(false),
    slave_mode_on_(false),
    is_busy_(false),
    is_preempted_(false),
    is_running_(false),
    process_trajectory_(false)
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

    //set speed
    float speed[2];
    float speed_return[2];
    speed[0] = 100.0;
    speed[1] = 100.0;
    ROS_INFO_STREAM(name_ + ": Set robot external speed");
    hr = bcap_.bCap_RobotExecute2(hRobot_, "ExtSpeed", VT_R4|VT_ARRAY, 2, speed, speed_return);
    if(FAILED(hr))
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }

    float internal_speed = 100.0;
    float internal_speed_return;
    ROS_INFO_STREAM(name_ + ": Set robot internal speed");
    hr = bcap_.bCap_RobotExecute2(hRobot_, "Speed", VT_R4, 1, &internal_speed, &internal_speed_return);
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

    //read and set joint angle (position)
    hr = getJointFeedback(true);
    if (FAILED(hr))
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }
    std::vector<boost::shared_ptr<hg::Joint> >::iterator it;
    for (it = joints_.begin(); it != joints_.end(); it++)
    {
      ROS_INFO_STREAM(name_ << ": " << (*it)->name_ << " position " << (*it)->position_);
      ROS_INFO_STREAM(name_ << ": " << (*it)->name_ << " desired position_ " << (*it)->desired_position_);
    }

    //read current joint position


    //turn on motor
    ROS_INFO_STREAM(name_ + ": turn on motor");
    hr = setMotor(true);
    if (FAILED(hr))
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }
#if USE_SLAVE_MODE
    {
      //set slave mode
      int mode = SLAVE_MODE;
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
      if (result != SLAVE_MODE)
      {
        bcap_.bCap_Close();
        ROS_BREAK();
      }
      slave_mode_on_ = true;
    }
#endif
  }

  //create control thread
  control_thread_ = boost::thread(&RC7MController::control, this);
  //control_thread_.
  is_running_ = true;


  //start action server
  action_server_ = FollowJointTrajectoryActionServerPtr(
      new FollowJointTrajectoryActionServer(node_->node_handle_, "follow_joint_trajectory",
                                            boost::bind(&RC7MController::followJointGoalActionCallback, this, _1), false));
  action_server_->start();


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
  double command[7];
  float command_degree[7], command_result[7];

  //ROS_INFO("control() loop rate: %f", rate_);

  while (is_running_)
  {
    ros::Time startTime = ros::Time(ros::WallTime::now().toSec());
    int i = 0;
    for (it = joints_.begin(); it != joints_.end(); it++)
    {
#if USE_SLAVE_MODE
      command[i] = (*it)->interpolate(1.0 / rate_);
      //command[i] = (*it)->desired_position_;
#else
      command[i] = (*it)->desired_position_;
#endif

      //check real robot <-> URDF joint offset
      if ((*it)->position_offset_ != 0.0)
      {
        command_degree[i] = ((command[i] - (*it)->position_offset_) * 180.0) / M_PI;
      }
      else
      {
        command_degree[i] = (command[i] * 180.0) / M_PI;
      }

      i++;
    }

    if (!node_->simulate_)
    {
      BCAP_HRESULT hr;
      if (motor_on_)
      {
#if USE_SLAVE_MODE


        hr = bcap_.bCap_RobotExecute2(hRobot_, "slvMove", VT_R4 | VT_ARRAY, 7, command_degree, command_result);
        if (FAILED(hr))
        {
          ROS_ERROR_STREAM_THROTTLE(1.0, name_ + " slvMode fail!");
        }

#else
        long lComp = 1; //MOVE P
        std::stringstream ss;
        ss << "J(";
        for(int j = 0; j < (i-1); j++)
        {
          ss << command_degree[j] << ",";
        }
        ss << command_degree[i-1] << ")";
        //ROS_INFO_STREAM(ss.str());
        hr = bcap_.bCap_RobotMove(hRobot_, lComp, ss.str(), "");
        if (FAILED(hr))
        {
          ROS_ERROR_STREAM_THROTTLE(1.0, name_ + " Robot_Mode fail!");
          return;
        }
#endif
      }

      //get feedback from robot's angle variable
      hr = getJointFeedback();
      if (FAILED(hr))
      {
        ROS_ERROR_STREAM_THROTTLE(1.0, name_ + " get joint feedback fail!");
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
    ROS_DEBUG_STREAM_THROTTLE(1.0, "slvMove Time: " << ros::Time(ros::WallTime::now().toSec()) - startTime);

  }

}

void RC7MController::shutdown()
{
  is_running_ = false;
  control_thread_.join();

  if (!node_->simulate_)
  {
    BCAP_HRESULT hr;


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
    slave_mode_on_ = false;

    //turn off motor
    ROS_INFO_STREAM(name_ + ": turn off motor");
    hr = setMotor(false);
    if (FAILED(hr))
    {
      bcap_.bCap_Close();
      ROS_BREAK();
    }

    //release variables
    ROS_INFO_STREAM(name_ + ": release robot variable handles");
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

bool RC7MController::setJointsSpeed(double speed)
{
  std::vector<boost::shared_ptr<hg::Joint> >::iterator it;
  for (it = joints_.begin(); it != joints_.end(); it++)
  {
    if(speed <= (*it)->parameter_velocity_limit_)
      (*it)->velocity_limit_ = speed;
    else
      ROS_WARN_STREAM("Cannot set " << (*it)->name_ << " velocity");
  }
  return true;
}

BCAP_HRESULT RC7MController::setMotor(bool on_off)
{
  if (node_->simulate_)
    return BCAP_S_OK;

  boost::unique_lock<boost::mutex> lock(control_mutex_);

  BCAP_HRESULT hr = BCAP_E_FAIL;
  int mode = (on_off) ? 1 : 0;
  long result;
  hr = bcap_.bCap_RobotExecute2(hRobot_, "Motor", VT_I2, 1, &mode, &result);
  if(SUCCEEDED(hr))
  {
    ROS_INFO_STREAM(name_ + ": waiting for motor to turn on/off");
    ros::Duration(5.0).sleep(); //waiting for motor to turn on/off
    motor_on_ = on_off;
  }
  return hr;
}

BCAP_HRESULT RC7MController::getJointFeedback(bool set_desired_position_)
{
  float angle_variables[8];
  BCAP_HRESULT hr = BCAP_E_FAIL;

  boost::unique_lock<boost::mutex> lock(control_mutex_);

  hr = bcap_.bCap_VariableGetValue(hAngleVariable, angle_variables);
  if (FAILED(hr))
  {
    return hr;
  }



  std::vector<boost::shared_ptr<hg::Joint> >::iterator it;
  int i = 0;
  double radian = 0;
  for (it = joints_.begin(); it != joints_.end(); it++)
  {
    //convert to radian
    radian = (angle_variables[i] * M_PI)/180.0;

    //check real robot <-> URDF joint offset
    if((*it)->position_offset_ != 0.0)
    {
      radian += (*it)->position_offset_;
    }


    if(set_desired_position_)
    {
      (*it)->desired_position_ = radian;
      (*it)->last_commanded_position_ = radian;
    }
    (*it)->setFeedbackData(radian);
    i++;
  }

  return hr;
}

void RC7MController::followJointGoalActionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
  //ROS_INFO_STREAM(name_  << __FUNCTION__);
  action_goal_ = goal;

  //get trajectory
  trajectory_msgs::JointTrajectory trajectory = action_goal_->trajectory;

  control_msgs::FollowJointTrajectoryResult result;
  static trajectory_msgs::JointTrajectoryPoint last_point;

  //check joint names
  for (size_t i = 0; i < trajectory.joint_names.size(); i++)
  {
    std::vector<boost::shared_ptr<hg::Joint> >::iterator it;
    bool found = false;
    for (it = joints_.begin(); it != joints_.end(); it++)
    {
      if((*it)->name_ == trajectory.joint_names[i])
      {
        found = true;
      }
    }
    if (!found)
    {
      ROS_ERROR("Trajectory joint names do not match with controlled joints");
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      action_server_->setAborted(result, "Trajectory joint names do not match with controlled joints");
      return;
    }
  }

  if(trajectory.points.size() == 0)
  {
    ROS_ERROR("Trajectory empty");
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL ;
    action_server_->setAborted(result, "Trajectory empty");
    return;
  }

  ROS_INFO("Got trajectory with %d points", (int)trajectory.points.size());

  ros::Time trajectory_start_time = trajectory.header.stamp;
  //check points
  if (is_preempted_)
  {
#if 1
    //search for nearest in new trajectory due to unpredictable delay in trajectory filter
    std::vector<boost::shared_ptr<hg::Joint> >::iterator it;
    std::vector<double> joint_position;
    for (it = joints_.begin(); it != joints_.end(); it++)
    {
      joint_position.push_back((*it)->position_);
      ROS_INFO("J %f", (*it)->position_);
    }

    std::vector<double> sum_square_error(trajectory.points.size());
    double error;
    for(int i = 0; i < (int)trajectory.points.size(); i++)
    {
      sum_square_error[i] = 0;
      for(int j = 0; j < joint_position.size(); j++)
      {
        error = trajectory.points[i].positions[j] - joint_position[j];
        sum_square_error[i] += error * error;
      }
    }

    int min_index = std::min_element(sum_square_error.begin(), sum_square_error.end()) - sum_square_error.begin();
    printf("min sum_square_error: %f @ %d\n", sum_square_error[min_index], min_index);
    printf("joint : %f6.3 %f6.3 %f6.3 %f6.3 %f6.3 %f6.3\n",
           joint_position[0],
           joint_position[1],
           joint_position[2],
           joint_position[3],
           joint_position[4],
           joint_position[5]);
    printf("preempted point : %f6.3 %f6.3 %f6.3 %f6.3 %f6.3 %f6.3\n",
           last_trajectory_.points[preempted_point_].positions[0],
           last_trajectory_.points[preempted_point_].positions[1],
           last_trajectory_.points[preempted_point_].positions[2],
           last_trajectory_.points[preempted_point_].positions[3],
           last_trajectory_.points[preempted_point_].positions[4],
           last_trajectory_.points[preempted_point_].positions[5]);
    printf("start point : %f6.3 %f6.3 %f6.3 %f6.3 %f6.3 %f6.3\n",
           trajectory.points[min_index].positions[0],
           trajectory.points[min_index].positions[1],
           trajectory.points[min_index].positions[2],
           trajectory.points[min_index].positions[3],
           trajectory.points[min_index].positions[4],
           trajectory.points[min_index].positions[5]);

    //is_preempted_ = false;
    new_start_point_ = min_index;
    is_preempted_ = false;
    /*
    if(new_start_point_ >= trajectory.points.size())
      new_start_point_ = trajectory.points.size() - 1;
    */
#else
    new_start_point_ = 0;
    is_preempted_ = false;
#endif
  }
  else
  {
    new_start_point_ = 0;
  }

  //wait for start time
  while (ros::Time::now() < trajectory_start_time)
  {
    ros::Duration(0.001).sleep();
  }

  ros::Duration skip_duration = trajectory.points[new_start_point_].time_from_start;
  bool success = true;
  ros::Time last_time = trajectory_start_time + skip_duration;
  for (size_t i = new_start_point_; i < trajectory.points.size(); i++)
  {
    trajectory_msgs::JointTrajectoryPoint point = trajectory.points[i];
    ros::Time end_time = trajectory_start_time + (point.time_from_start - skip_duration);
    //ROS_INFO_STREAM("dt: " << (end_time - last_time).toSec());
    //last_time = end_time;
    std::vector<boost::shared_ptr<hg::Joint> >::iterator it = joints_.begin();
    for (size_t k = 0; k < point.positions.size(); k++, it++)
    {
      //while((*it)->touched_) { ros::Duration(0.0001).sleep(); }
      (*it)->setPosition(point.positions[k]);
    }
    last_point = point;

    while ((ros::Time::now() < end_time) && (!action_server_->isPreemptRequested()))
    {
      ros::Duration(0.0001).sleep();
    }

    if (action_server_->isPreemptRequested() || !ros::ok())
    {
      //ROS_INFO("Preempted!!! @ point %d %f", (int)i, last_point.positions[0]);
      printf("Preempted!! @ point %d : %f6.3 %f6.3 %f6.3 %f6.3 %f6.3 %f6.3\n", (int) i,
             last_point.positions[0],
             last_point.positions[1],
             last_point.positions[2],
             last_point.positions[3],
             last_point.positions[4],
             last_point.positions[5]);
      is_preempted_ = true;
      preempted_point_ = i;
      process_trajectory_ = false;
      action_server_->setPreempted();
      success = false;
      break;
    }
  }

  last_trajectory_ = trajectory;


  if(success)
  {
    is_preempted_ = false;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    action_server_->setSucceeded(result);
  }
}
