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
#include <hg_cpp/absolute_ joint.h>
#include <hg_cpp/controller_node.h>

using namespace hg_joint;

PLUGINLIB_DECLARE_CLASS(joint_plugins, AbsoluteJoint, hg_joint::AbsoluteJoint, hg::Joint)

AbsoluteJoint::AbsoluteJoint() :
    hg::Joint()
{

}

AbsoluteJoint::~AbsoluteJoint()
{

}

void AbsoluteJoint::initilize(hg::ControllerNode* node, const std::string& name)
{
  hg::Joint::initilize(node, name);

  joint_info_ = node_->urdf_model_.getJoint(name_);
  if (joint_info_ == NULL)
  {
    ROS_FATAL("Cannot get joint %s info", name_.c_str());
    ROS_BREAK();
  }
  ROS_DEBUG_STREAM(joint_info_->name << " lower limit: " << joint_info_->limits->lower);
  ROS_DEBUG_STREAM(joint_info_->name << " upper limit: " << joint_info_->limits->upper);
  ROS_DEBUG_STREAM(joint_info_->name << " velocity limit: " << joint_info_->limits->velocity);
  ROS_DEBUG_STREAM(joint_info_->name << " effort limit: " << joint_info_->limits->effort);

  //start the joint at 0 or its lower limit or its upper limit
  if (joint_info_->limits->upper < 0)
  {
    position_ = joint_info_->limits->upper;
  }
  else if (joint_info_->limits->lower > 0)
  {
    position_ = joint_info_->limits->lower;
  }
  desired_position_ = position_;
  last_commanded_position_ = position_;

  //subscribers
  subscriber_joint_position_ = node_->nh_private_.subscribe(name_ + "/set/position", 1,
                                                            &AbsoluteJoint::callbackSetPosition, this);
  subscriber_joint_position_relative_ = node_->nh_private_.subscribe(name_ + "/set/relative_position", 1,
                                                                     &AbsoluteJoint::callbackSetRelativePosition, this);

}

bool AbsoluteJoint::getJointInformationUrdf(const std::string& param)
{
  return false;
}

double AbsoluteJoint::interpolate(double dt)
{
  boost::recursive_mutex::scoped_lock(mutex_);
  if (touched_)
  {
    double command = desired_position_ - last_commanded_position_;

    //check velocity limit
    double move_limit_dt = joint_info_->limits->velocity * dt;

    if (command > move_limit_dt)
    {
      command = move_limit_dt;
    }
    else if (command < -move_limit_dt)
    {
      command = -move_limit_dt;
    }

    //check position limit
    command = last_commanded_position_ + command;

    if (command > joint_info_->limits->upper)
    {
      command = joint_info_->limits->upper;
      ROS_WARN_STREAM_THROTTLE(1.0, name_ + " is reached upper limit");
    }
    else if (command < joint_info_->limits->lower)
    {
      command = joint_info_->limits->lower;
      ROS_WARN_STREAM_THROTTLE(1.0, name_ + " is reached lower limit");
    }

    //store last command
    last_commanded_position_ = command;

    //reach target
    if (last_commanded_position_ == desired_position_)
    {
      touched_ = false; //stop the movement
    }
    return command;
  }
  else
  {
    return desired_position_; //stop and hold position
  }
}

double AbsoluteJoint::interpolateCubic(double dt)
{
  boost::recursive_mutex::scoped_lock(mutex_);

  if(touched_)
  {
    if((coefficients_.size() != 4) || (final_time_ = 0) || (current_time_ >= final_time_))
    {
      //return current position
      ROS_WARN_THROTTLE(1.0, "invalid cubic spline state");
      return position_;
    }

    double command = coefficients_[0] +
                      (coefficients_[1] * current_time_) +
                      (coefficients_[2] * pow(current_time_, 2)) +
                      (coefficients_[3] * pow(current_time_, 3));

    current_time_ += dt;

    //check velocity limit
    double move_limit_dt = joint_info_->limits->velocity * dt;

    if (command > move_limit_dt)
    {
      command = move_limit_dt;
    }
    else if (command < -move_limit_dt)
    {
      command = -move_limit_dt;
    }

    //check position limit
    command = last_commanded_position_ + command;

    if (command > joint_info_->limits->upper)
    {
      command = joint_info_->limits->upper;
      ROS_WARN_STREAM_THROTTLE(1.0, name_ + " is reached upper limit");
    }
    else if (command < joint_info_->limits->lower)
    {
      command = joint_info_->limits->lower;
      ROS_WARN_STREAM_THROTTLE(1.0, name_ + " is reached lower limit");
    }

    //store last command
    last_commanded_position_ = command;

    //reach target
    if (last_commanded_position_ == desired_position_)
    {
      touched_ = false; //stop the movement
    }
    return command;
  }
  else
  {
    return desired_position_; //stop and hold position
  }

  return 0;
}

void AbsoluteJoint::setFeedbackData(double feedback)
{
  //set new position from feedback data
  position_ = feedback;

  //compute joint velocity in unit/sec
  ros::Time t = ros::Time::now();
  if((t - last_update_).toSec() > 0.01)
  {
    velocity_ = (position_ - last_position_) / (t - last_update_).toSec();
    //save last updated time
    last_update_ = t;
    last_position_ = position_;
  }
}

double AbsoluteJoint::setPosition(double position)
{
  //check position limit
  if ((position > joint_info_->limits->upper) || (position < joint_info_->limits->lower))
  {
    ROS_WARN_STREAM_THROTTLE(
        1.0,
        name_ + " position out of range [" << joint_info_->limits->lower << ", " << joint_info_->limits->upper << "]");
    return position_;
  }

  mutex_.lock();
  desired_position_ = position;
  touched_ = true;
  mutex_.unlock();

  return desired_position_;
}

double AbsoluteJoint::setPosition(double position, double final_time)
{
  //check position limit
  if ((position > joint_info_->limits->upper) || (position < joint_info_->limits->lower))
  {
    ROS_WARN_STREAM_THROTTLE(
        1.0,
        name_ + " position out of range [" << joint_info_->limits->lower << ", " << joint_info_->limits->upper << "]");

    //reset coefficients
    coefficients_.clear();
    final_time_ = 0;
    return position_;
  }

  mutex_.lock();
  double q0, q1, v0, v1;
  q0 = position_; //current position
  q1 = position; //desired position
  v0 = velocity_; //computed feedback
  v1 = 0.0; //stop at desired position

  //reset time
  current_time_ = 0;
  final_time_ = final_time;

  solveCubicSpline(q0, q1, v0, v1, final_time_, coefficients_);
  desired_position_ = position;
  touched_ = true;

  mutex_.unlock();

  ROS_INFO("q0: %f, q1 %f, v0: %f v1: %f \n "
           "coeff[0]: %f \n"
           "coeff[1]: %f \n"
           "coeff[2]: %f \n"
           "coeff[3]: %f \n", q0, q1, v0, v1,
           coefficients_[0],
           coefficients_[1],
           coefficients_[2],
           coefficients_[3]);

  return desired_position_;
}

double AbsoluteJoint::setPositionRelative(double position)
{
  double temp = position_ + position;
  return setPosition(temp);
}

diagnostic_msgs::DiagnosticStatus AbsoluteJoint::getDiagnostics()
{
  return hg::Joint::getDiagnostics();
}

void AbsoluteJoint::callbackSetPosition(const std_msgs::Float64& position)
{
  //check controller status
  if (!controller_->active())
  {
    setPosition(position.data);
  }
  else
  {
    ROS_WARN("controller is active");
  }
}

void AbsoluteJoint::callbackSetRelativePosition(const std_msgs::Float64& position)
{
  //check controller status
  if (!controller_->active())
  {
    setPositionRelative(position.data);
  }
  else
  {
    ROS_WARN("controller is active");
  }
}
