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
#include <ve026a_controller/ve026a_controller.h>
#include <algorithm>

PLUGINLIB_DECLARE_CLASS(hg_cpp, ve026a_controller, hg_plugins::VE062AController, hg::Controller)

using namespace hg_plugins;

#define USE_SLAVE_MODE 1
#define SLAVE_MODE 0x102

VE062AController::VE062AController() :
    hg::Controller(),
    bcap_(),
    motor_on_(false),
    slave_mode_on_(false),
    is_preempted_(false),
    is_running_(false),
    process_trajectory_(false)
{
  //ROS_INFO_STREAM(__FUNCTION__);
}

VE062AController::~VE062AController()
{
  //ROS_INFO_STREAM(__FUNCTION__);
}

void VE062AController::initilize(hg::Node* node, const std::string& name)
{
  hg::Controller::initilize(node, name);

  ROS_INFO_STREAM("read " + name + " parameter from file");
  ROS_ASSERT(node_->node_handle_.getParam("controllers/" + name + "/rate", rate_));
  ROS_ASSERT(node_->node_handle_.getParam("controllers/" + name + "/port", port_));
  ROS_INFO_STREAM("control rate " << rate_);
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

    ROS_INFO_STREAM("added " + joint_name + " to115200, 8, 1, SerialPort::PARITY_NONE " + name_);
  }

}

void VE062AController::startup()
{
  if (!node_->simulate_)
  {
    BCAP_HRESULT hr;

    //open connection
    ROS_INFO_STREAM(name_ << ": connect to: " << port_);
    int error = bcap_.SerialPort::Open(port_);
    if(error) ROS_BREAK();
    error = bcap_.Initialise(115200, 8, 1, SerialPort::PARITY_NONE);
    if(error) ROS_BREAK();


    ROS_INFO_STREAM(name_ + ": initializing ...");

    //connect controller
    hr = bcap_.ControllerConnect("b-CAP", "CaoProv.DENSO.VRC", "localhost", "@EventDisable=false,Wpj=*", &hController_);
    if (FAILED(hr))
    {
      ROS_ERROR_STREAM(name_ + ": cannot connect to controller");
      bcap_.SerialPort::Close();
      ROS_BREAK();
    }
    else
    {
      ROS_INFO_STREAM(name_ + ": controller connected");
    }
/*
      //get robot
      hr = bcap_serial_.bCap_ControllerGetRobot(b_cap_controller_, "VE026A", "$IsIDHandle$", &b_cap_robot_);
      if (FAILED(hr))
      {
        ROS_ERROR_STREAM(name_ + ": get robot fail");
        return;
      }
      else
      {
        ROS_INFO_STREAM(name_ + ": get robot successful");
      }

      //turn off motor
      if (!set_motor(false))
        return;

      //set slave mode
      int mode = 258;
      long result;
      hr = bcap_serial_.bCap_RobotExecute2(b_cap_robot_, "slvChangeMode", VT_I4, 1, &mode, &result);
      if (FAILED(hr))
      {
        ROS_ERROR_STREAM(name_ + ": cannot change to slave mode");
        return;
      }
      else
      {
        ROS_INFO_STREAM(name_ + ": change robot slave mode successful");
      }

      float joints[8];
      float results[8];
      memset(joints, 0, sizeof(float) * 8);
      memset(results, 0, sizeof(float) * 8);

      //get joint positions
      if (!set_joints(joints, results))
        return;

      if (!set_joints(results, joints))
        return;

      //setup current position and desired position
      JointMap::iterator itr;
      int i;
      for (itr = joints_.begin(), i = 0; itr != joints_.end(); itr++, i++)
      {
        itr->second->set_feedback_data(joints[i] * DEG_TO_RAD);
        itr->second->set_position(joints[i] * DEG_TO_RAD);
      }

      //turn on motor
      if (!set_motor(true))
        return;

      ROS_INFO_STREAM(name_ + " is initialized");
      is_initialized_ = true;
    }
    else
    {
      ROS_WARN_STREAM(name_ + " is already initialized");
    }
  */


  }



    /*
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
*/
  //create control thread
  control_thread_ = boost::thread(&VE062AController::control, this);
  //control_thread_.
  is_running_ = true;


  //start action server
  action_server_ = FollowJointTrajectoryActionServerPtr(
      new FollowJointTrajectoryActionServer(node_->node_handle_, "follow_joint_trajectory",
                                            boost::bind(&VE062AController::followJointGoalActionCallback, this, _1), false));
  action_server_->start();


  ROS_INFO_STREAM(name_ + ": started");

}

void VE062AController::update()
{
  //ROS_INFO_STREAM(name_ + " updated");
}

void VE062AController::control()
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


        hr = bcap_.RobotExecute2(hRobot_, "slvMove", VT_R4 | VT_ARRAY, 7, command_degree, command_result);
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


void VE062AController::shutdown()
{
  is_running_ = false;
  control_thread_.join();
  ROS_INFO_STREAM(name_ + " shutdown");
}

bool VE062AController::active()
{
  return is_active_;
}

bool VE062AController::setJointsSpeed(double speed)
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

BCAP_HRESULT VE062AController::setMotor(bool on_off)
{
  if (node_->simulate_)
    return BCAP_S_OK;

  boost::unique_lock<boost::mutex> lock(control_mutex_);

  BCAP_HRESULT hr = BCAP_E_FAIL;
  int mode = (on_off) ? 1 : 0;
  long result;
  hr = bcap_.RobotExecute2(hRobot_, "Motor", VT_I2, 1, &mode, &result);
  if(SUCCEEDED(hr))
  {
    ROS_INFO_STREAM(name_ + ": waiting for motor to turn on/off");
    ros::Duration(5.0).sleep(); //waiting for motor to turn on/off
    motor_on_ = on_off;
  }
  return hr;
}

BCAP_HRESULT VE062AController::getJointFeedback(bool set_desired_position_)
{
  float angle_variables[8];
  BCAP_HRESULT hr = BCAP_E_FAIL;

  boost::unique_lock<boost::mutex> lock(control_mutex_);

  hr = bcap_.VariableGetValue(hAngleVariable, angle_variables);
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

void VE062AController::followJointGoalActionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
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

  ROS_DEBUG("Got trajectory with %d points", (int)trajectory.points.size());

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
      ROS_DEBUG("J %f", (*it)->position_);
    }

    std::vector<double> sum_square_error(trajectory.points.size());
    double error;
    for(int i = 0; i < (int)trajectory.points.size(); i++)
    {
      sum_square_error[i] = 0;
      for(int j = 0; j < (int)joint_position.size(); j++)
      {
        error = trajectory.points[i].positions[j] - joint_position[j];
        sum_square_error[i] += error * error;
      }
    }

    int min_index = std::min_element(sum_square_error.begin(), sum_square_error.end()) - sum_square_error.begin();

    ROS_DEBUG("min sum_square_error: %f @ %d\n", sum_square_error[min_index], min_index);
    ROS_DEBUG("joint : %f6.3 %f6.3 %f6.3 %f6.3 %f6.3 %f6.3\n",
           joint_position[0],
           joint_position[1],
           joint_position[2],
           joint_position[3],
           joint_position[4],
           joint_position[5]);
    ROS_DEBUG("preempted point : %f6.3 %f6.3 %f6.3 %f6.3 %f6.3 %f6.3\n",
           last_trajectory_.points[preempted_point_].positions[0],
           last_trajectory_.points[preempted_point_].positions[1],
           last_trajectory_.points[preempted_point_].positions[2],
           last_trajectory_.points[preempted_point_].positions[3],
           last_trajectory_.points[preempted_point_].positions[4],
           last_trajectory_.points[preempted_point_].positions[5]);
    ROS_DEBUG("start point : %f6.3 %f6.3 %f6.3 %f6.3 %f6.3 %f6.3\n",
           trajectory.points[min_index].positions[0],
           trajectory.points[min_index].positions[1],
           trajectory.points[min_index].positions[2],
           trajectory.points[min_index].positions[3],
           trajectory.points[min_index].positions[4],
           trajectory.points[min_index].positions[5]);

    new_start_point_ = min_index;
    is_preempted_ = false;

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

  is_active_ = true;
  ros::Duration skip_duration = trajectory.points[new_start_point_].time_from_start;
  bool success = true;
  ros::Time last_time = trajectory_start_time + skip_duration;
  for (size_t i = new_start_point_; i < trajectory.points.size(); i++)
  {
    trajectory_msgs::JointTrajectoryPoint point = trajectory.points[i];
    ros::Time end_time = trajectory_start_time + (point.time_from_start - skip_duration);
    std::vector<boost::shared_ptr<hg::Joint> >::iterator it = joints_.begin();
    for (size_t k = 0; k < point.positions.size(); k++, it++)
    {
      (*it)->setPosition(point.positions[k]);
    }
    last_point = point;

    while ((ros::Time::now() < end_time) && (!action_server_->isPreemptRequested()))
    {
      ros::Duration(0.0001).sleep();
    }

    if (action_server_->isPreemptRequested() || !ros::ok())
    {
      ROS_DEBUG("Preempted!! @ point %d : %f6.3 %f6.3 %f6.3 %f6.3 %f6.3 %f6.3\n", (int) i,
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
  is_active_ = false;
  last_trajectory_ = trajectory;


  if(success)
  {
    is_preempted_ = false;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    action_server_->setSucceeded(result);
  }
}




#if 0
#include <hg_cpp/denso/denso_ve026a_controller.h>
#include <boost/any.hpp>


using namespace std;
using namespace hg;

#define PI	3.141592
#define DEG_TO_RAD (PI/180.0)
#define RAD_TO_DEG (180.0f/PI)

DensoVe026a_BCapController::DensoVe026a_BCapController(hg::HgROS* hg_ros, const std::string& name)
		: Controller(hg_ros, name),
		  is_initialized_(false),
		  is_motor_on_(false),
		  listener_(hg_ros->node_handle_)
{
	string port_param_name = "controllers/" + name_ + "/port";

	if(!node_handle_.hasParam(port_param_name))
	{
		ROS_ERROR_STREAM(name_ + ": couldn't find " + port_param_name +
				"please check if ALL parameters have been set correctly");
	}
	node_handle_.param<std::string>(
			port_param_name, port_name_, "/dev/ttyUSB0");
	ROS_INFO_STREAM(name_ + ": serial port: " + port_name_);

	//control rate
	node_handle_.param<double>(
			"controllers/" + name_ + "/control_rate", control_rate_, 100);
	ROS_INFO_STREAM(name_ + ": control_rate: " << control_rate_);


	//add joints
	XmlRpc::XmlRpcValue joints;
	node_handle_.getParam("controllers/" + name_ + "/joints", joints);
	ROS_ASSERT(joints.getType() == XmlRpc::XmlRpcValue::TypeArray);
	for(int i = 0; i < joints.size(); i++)
	{
		std::string joint_name;
		ROS_ASSERT(joints[i].getType() == XmlRpc::XmlRpcValue::TypeString);
		joint_name = static_cast<std::string>(joints[i]);
		JointMap::iterator itr = hg_ros->joints_.find(joint_name);
		if(itr != hg_ros->joints_.end())
		{
			joints_.insert(*itr);
			ROS_INFO_STREAM(name_ << " added joint: " << joint_name);
		}
		else
		{
			ROS_WARN_STREAM(name_ << " joint: " << joint_name << " not found");
		}
	}

	//set joint controller
	for(JointMap::iterator itr = joints_.begin(); itr != joints_.end(); itr++)
	{
		itr->second->controller_ = this;
	}

	//ROS_INFO("Waiting for arm_kinematics services...");
	//ros::service::waitForService("/arm_kinematics/get_ik");
	//ros::service::waitForService("/arm_kinematics/get_ik_solver_info");

	//get_ik_client_ = node_handle_.serviceClient<kinematics_msgs::GetPositionIK>
			//("/arm_kinematics/get_ik");
	//get_ik_solver_info_client_ =
			//node_handle_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>
				//("/arm_kinematics/get_ik_solver_info");



	//ROS_INFO("arm_kinematics services connected");

	//action server
	//get name
	trajectory_is_executing_ = false;
	node_handle_.getParam("controllers/" + name_ + "/action_name", action_name_);
	cout << action_name_ << endl;

	//create server
	action_server_  = boost::shared_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> >
	(new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(
			node_handle_,
			action_name_,
			false));

	//set callback
	action_server_->registerGoalCallback(boost::bind(&DensoVe026a_BCapController::action_callback, this));

	motor_on_off_command_ = node_handle_.subscribe(name + "/motor_on_off",
				1, &DensoVe026a_BCapController::set_motor_callback, this);




}

DensoVe026a_BCapController::~DensoVe026a_BCapController()
{

}

void DensoVe026a_BCapController::startup()
{
	if(!hg_ros_->simulate_)
	{
		int error = bcap_serial_.Open(port_name_);
		if(error) return;
		error = bcap_serial_.Initialise(115200, 8, 1, SerialPort::PARITY_NONE);
		if(error) return;

		ROS_INFO_STREAM(name_ + ": serial port connected");
	}

	initialize_ve026a();

	if(!is_initialized_)
		return;


	//start action server
	//move_arm_action_server_.start();
	action_server_->start();


	//create control thread
	control_thread_ = boost::thread(&DensoVe026a_BCapController::control_loop, this);
	follow_control_thread_ = boost::thread(&DensoVe026a_BCapController::execute_trajectory, this);

	//start control loop
	is_running_ = true;


	ROS_INFO_STREAM(name_ << " started");
}

void DensoVe026a_BCapController::update()
{

}

void DensoVe026a_BCapController::shutdown()
{
	set_motor(false);
	is_running_ = false;
	control_thread_.join();
}

bool DensoVe026a_BCapController::active()
{
	return false;
}


void DensoVe026a_BCapController::action_callback()
{
	action_goal_ = action_server_->acceptNewGoal();
	ROS_INFO_STREAM(name_ + ": Action goal received");

	trajectory_msgs::JointTrajectory trajectory = action_goal_->trajectory;

	control_msgs::FollowJointTrajectoryResult result;

	//check joint names
	for(size_t i = 0; i < trajectory.joint_names.size(); i++)
	{
		JointMap::iterator itr = joints_.find(trajectory.joint_names[i]);
		if(itr == joints_.end())
		{
			result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
			action_server_->setAborted(result, "Trajectory joint names does not match action controlled joints");
			return;
		}
	}


	//check points
	if(trajectory.points.size() == 0)
	{
		result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL ;
		action_server_->setAborted(result, "Trajectory empty");
		return;
	}


	ROS_INFO_STREAM("Trajectort points: " << trajectory.points.size());
	for(int i = 0; i < trajectory.points.size(); i++)
	{
		trajectory_msgs::JointTrajectoryPoint point =  trajectory.points[i];
		std::vector<double> target_positions = point.positions;
		ros::Time end_time = ros::Time::now() + point.time_from_start;
		//ROS_INFO("Trajectory[%d]: end-time: %6.3f", i, end_time.toSec());
	}

	boost::unique_lock<boost::mutex> lock(mutex_);
	trajectory_queue_.push(trajectory);
	condition_.notify_one();

	//action_server_->setAborted(result, "Test");
	//execute_trajectory(trajectory);
}

void DensoVe026a_BCapController::command_callback(const trajectory_msgs::JointTrajectory& message)
{
}

void DensoVe026a_BCapController::execute_trajectory()
{
	control_msgs::FollowJointTrajectoryResult result;
	while(is_running_)
	{

		// Acquire lock on the queue
		boost::unique_lock<boost::mutex> lock(mutex_);

		// When there is no data, wait till someone fills it.
		// Lock is automatically released in the wait and obtained
		// again after the wait
		while (trajectory_queue_.size()==0) condition_.wait(lock);

		ROS_INFO("Got a new trajectory!!!");
		// Retrieve the data from the queue
		trajectory_msgs::JointTrajectory trajectory = trajectory_queue_.front();
		trajectory_queue_.pop();


		ros::Time time = ros::Time::now();
		ros::Time start_time = trajectory.header.stamp;

		//ROS_INFO("%6.3f %6.3f", time.toSec(), start_time.toSec());

		std::vector<double> last_positions;
		for(JointMap::iterator itr = joints_.begin(); itr != joints_.end(); itr++)
		{
			last_positions.push_back(itr->second->position_);
			cout << "last position of "
				 << itr->first << ":"
				 << itr->second->position_
				 << endl;

		}

		for(int i = 0; i < trajectory.points.size(); i++)
		{
			while((ros::Time::now() + ros::Duration(0.01)) < start_time)
			{
				//wait for time
				ros::Duration(0.01).sleep();
			}


			trajectory_msgs::JointTrajectoryPoint point =  trajectory.points[i];
			std::vector<double> target_positions = point.positions;
			//for(int j = 0; j < target_positions.size(); j++)
			//{
				//cout << "target joint[" << j << "]" <<  " position: " << target_positions[j] << endl;
			//}

			ros::Time end_time = start_time + point.time_from_start;
			//ROS_INFO("Trajectory[%d]: end-time: %6.3f", i, end_time.toSec());

			std::vector<double> error, velocity;
			error.resize(target_positions.size());
			velocity.resize(target_positions.size());
			double command;
			ros::Rate rate(50.0);
			while((ros::Time::now()+ros::Duration(0.01)) < end_time)
			{
				JointMap::iterator itr = joints_.begin();
				for(int k = 0; k < error.size(); k++, itr++)
				{
					error[k] = target_positions[k] - last_positions[k];
					velocity[k] =
							fabs(error[k] / (50.0 * (end_time - ros::Time::now()).toSec()));
					//ROS_INFO_THROTTLE(1, "v: %f", velocity[k]);

					if(fabs(error[k]) > 0.001)
					{
						command = error[k];
						if(command > velocity[k])
						{
							command = velocity[k];
						}
						else if(command > velocity[k])
						{
							command = -velocity[k];
						}
						last_positions[k] += command;
						//ROS_INFO_THROTTLE(1, "cmd: %f", last_positions[k]);
						itr->second->set_position(last_positions[k]);
					}
					else
					{
						//reached
						velocity[k] = 0;
					}

				}

				rate.sleep();
			}
		}
		ROS_INFO("Trajectory done!!!");
		result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
		action_server_->setSucceeded(result, "test!");
	}
}

void DensoVe026a_BCapController::control_loop()
{
	ros::Rate loop_rate(control_rate_);
	float command[7], result[8];
	memset(command, 0, sizeof(command));
	memset(result, 0, sizeof(result));
	int i;
	JointMap::iterator itr;
	while(is_running_)
	{
		//ROS_INFO_STREAM(name_ + ": set command");
		for(itr = joints_.begin(), i = 0; itr != joints_.end(); itr++, i++)
		{
			command[i] = itr->second->interpolate(1.0/control_rate_) * RAD_TO_DEG;
		}

		if(!hg_ros_->simulate_)
		{
			//update joint positions
			if(!set_joints(command, result))
			{
				return;
			}

			//ROS_INFO_STREAM(name_ + ": set feedback");
			for(itr = joints_.begin(), i = 0; itr != joints_.end(); itr++, i++)
			{
				itr->second->set_feedback_data(result[i] * DEG_TO_RAD);
				if(!is_motor_on_)
				{
					itr->second->set_position(result[i] * DEG_TO_RAD);
				}
			}
		}
		else
		{
			for(itr = joints_.begin(), i = 0; itr != joints_.end(); itr++, i++)
			{
				itr->second->set_feedback_data(command[i] * DEG_TO_RAD);
				if(!is_motor_on_)
				{
					itr->second->set_position(command[i] * DEG_TO_RAD);
				}
			}
		}
		loop_rate.sleep();
	}

	condition_.notify_all();
}

void DensoVe026a_BCapController::initialize_ve026a()
{
	if(hg_ros_->simulate_)
	{
		is_initialized_ = true;
		return;
	}

	if(!is_initialized_)
	{
		ROS_INFO_STREAM(name_ + ": initializing ...");
		BCAP_HRESULT hr;

		//connect controller
		hr = bcap_serial_.bCap_ControllerConnect(
			"b-CAP",
			"CaoProv.DENSO.VRC",
			"localhost",
			"@EventDisable=false,Wpj=*",
			b_cap_controller_);
		if(FAILED(hr))
		{
			ROS_ERROR_STREAM(name_ + ": controller connect fail");
			return;
		}
		else
		{
			ROS_INFO_STREAM(name_ + ": controller connected");
		}

		//get robot
		hr = bcap_serial_.bCap_ControllerGetRobot(
			b_cap_controller_,
			"VE026A",
			"$IsIDHandle$",
			&b_cap_robot_);
		if(FAILED(hr))
		{
			ROS_ERROR_STREAM(name_ + ": get robot fail");
			return;
		}
		else
		{
			ROS_INFO_STREAM(name_ + ": get robot successful");
		}

		//turn off motor
		if(!set_motor(false))
			return;

		//set slave mode
		int mode = 258;
		long result;
		hr = bcap_serial_.bCap_RobotExecute2(
			b_cap_robot_,
			"slvChangeMode",
			VT_I4,
			1,
			&mode,
			&result);
		if(FAILED(hr))
		{
			ROS_ERROR_STREAM(name_ + ": cannot change to slave mode");
			return;
		}
		else
		{
			ROS_INFO_STREAM(name_ + ": change robot slave mode successful");
		}


		float joints[8];
		float results[8];
		memset(joints, 0, sizeof(float)*8);
		memset(results, 0, sizeof(float)*8);

		//get joint positions
		if(!set_joints(joints, results))
			return;

		if(!set_joints(results, joints))
			return;

		//setup current position and desired position
		JointMap::iterator itr;
		int i;
		for(itr = joints_.begin(), i = 0; itr != joints_.end(); itr++, i++)
		{
			itr->second->set_feedback_data(joints[i] * DEG_TO_RAD);
			itr->second->set_position(joints[i] * DEG_TO_RAD);
		}



		//turn on motor
		if(!set_motor(true))
			return;

		ROS_INFO_STREAM(name_ + " is initialized");
		is_initialized_ = true;
	}
	else
		ROS_WARN_STREAM(name_ + " is already initialized");

}

bool DensoVe026a_BCapController::set_motor(bool on_off)
{
	if(hg_ros_->simulate_) return true;

	boost::unique_lock<boost::mutex> lock(mutex_control_);

	BCAP_HRESULT hr;
	int mode = (on_off) ? 1 : 0;
	long result;
	hr = bcap_serial_.bCap_RobotExecute2(
		b_cap_robot_,
		"Motor",
		VT_I2,
		1,
		&mode,
		&result);
	if(FAILED(hr))
	{
		is_motor_on_ = false;
		ROS_ERROR_STREAM(name_ + ": set motor fail");
		return false;
	}
	else
	{
		is_motor_on_ = on_off;
		if(on_off)
			ROS_INFO_STREAM(name_ + ": set motor on");
		else
			ROS_INFO_STREAM(name_ + ": set motor off");
		return true;
	}
}

void DensoVe026a_BCapController::set_motor_callback(const std_msgs::BoolConstPtr& flag)
{
	if(flag->data)
	{
		ROS_INFO_STREAM("get motor on command");
		set_motor(true);
	}
	else
	{
		ROS_INFO_STREAM("get motor off command");
		set_motor(false);
	}
}



bool DensoVe026a_BCapController::set_joints(float* position, float* result)
{
	if(hg_ros_->simulate_) return true;

	boost::unique_lock<boost::mutex> lock(mutex_control_);

	BCAP_HRESULT hr;
	hr = bcap_serial_.bCap_RobotExecute2(
		b_cap_robot_,
		"slvMove",
		VT_R4 | VT_ARRAY,
		7,
		position,
		result);
	if(FAILED(hr))
	{
		ROS_ERROR_STREAM(name_ + ": set joints fail");
		return false;
	}
	else
	{
		return true;
	}
}
#endif
