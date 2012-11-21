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
#include <hg_cpp/hg_node.h>
#include <rc7m/rc7m_joint.h>


PLUGINLIB_DECLARE_CLASS(hg_cpp, rc7m_joint, hg_plugins::RC7MJoint, hg::Joint)

using namespace hg_plugins;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

RC7MJoint::RC7MJoint()
  : hg::Joint()
{
}

RC7MJoint::~RC7MJoint()
{
}

void RC7MJoint::initilize(hg::Node* node, const std::string& name)
{
  hg::Joint::initilize(node, name);

  //read parameters
  ROS_INFO_STREAM("read " + name + " parameter from file");
  ROS_ASSERT(node_->node_handle_.getParam("joints/" + name_ + "/upper_limit", upper_limit_));
  ROS_ASSERT(node_->node_handle_.getParam("joints/" + name_ + "/lower_limit", lower_limit_));
  ROS_ASSERT(node_->node_handle_.getParam("joints/" + name_ + "/position_offset", position_offset_));
  ROS_ASSERT(node_->node_handle_.getParam("joints/" + name_ + "/velocity_limit", velocity_limit_));

  ROS_INFO("velocity_limit_ %f", velocity_limit_);
  parameter_velocity_limit_ = velocity_limit_;

  acceleration_limit_ = 6.0;
  t_to_zero_ = t_acc_ = t_const_ = t_dacc_ = current_velocity_ = total_time_ = 0.0;

  //set position according to limit
  if(lower_limit_ > 0)
    position_ = desired_position_ = last_commanded_position_ = lower_limit_;
  else if(upper_limit_ < 0)
    position_ = desired_position_ = last_commanded_position_ = upper_limit_;

  //subscribers
  subscriber_joint_position_ =
      node_->node_handle_.subscribe(name_ + "/command/position", 1, &RC7MJoint::callbackJointPosition, this);
  subscriber_joint_position_relative_ =
        node_->node_handle_.subscribe(name_ + "/command/position_relative", 1, &RC7MJoint::callbackJointPositionRelative, this);
  subscriber_joint_position_degree_ =
      node_->node_handle_.subscribe(name_ + "/command/position_degree", 1, &RC7MJoint::callbackJointPositionDegree, this);
  subscriber_joint_velocity_ =
      node_->node_handle_.subscribe(name_ + "/command/velocity", 1, &RC7MJoint::callbackJointVelocity, this);

}

bool RC7MJoint::getJointInformationUrdf()
{
  return false;
}

double RC7MJoint::interpolate(double dt)
{
  //if position updated
  if (touched_)
  {
    mutex_.lock();
    double command = desired_position_ - last_commanded_position_;
    mutex_.unlock();

    //check velocity limit
    double move_limit_dt = velocity_limit_ * dt;

    if (command > move_limit_dt)
    {
      command = move_limit_dt;
      //ROS_WARN_STREAM_THROTTLE(1.0, name_ + " is reached velocity limit (+)");
    }
    else if (command < -move_limit_dt)
    {
      command = -move_limit_dt;
      //ROS_WARN_STREAM_THROTTLE(1.0, name_ + " is reached velocity limit (-)");
    }

    //check position limit
    command = last_commanded_position_ + command;

    if (command > upper_limit_)
    {
      command = upper_limit_;
      ROS_WARN_STREAM(name_ + " is reached upper limit");
    }
    else if (command < lower_limit_)
    {
      command = lower_limit_;
      ROS_WARN_STREAM(name_ + " is reached lower limit");
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
    return desired_position_; //do not move and hold position
  }
}

double RC7MJoint::interpolate2(double dt)
{
  if(touched_)
  {
    double command = 0;
    if(t_to_zero_ != 0.0)
    {
      //if(name_ == "J1")
        //ROS_INFO("to zero");
      //deaccelerate to zero
      command = (velocity_ * dt) + (0.5 * ((t_to_zero_ >= 0) ? acceleration_limit_ : -acceleration_limit_ ) * dt * dt);
      //current_velocity_ += ((t_to_zero_ >= 0) ? acceleration_limit_ : -acceleration_limit_)*dt;
      total_time_ += dt;
      if(total_time_ >= fabs(t_to_zero_))
      {
        total_time_ = 0.0;
        t_to_zero_ = 0.0;
      }
    }
    else if(t_acc_ != 0.0)
    {
      //if(name_ == "J1")
        //ROS_INFO("acc");
      command = (velocity_ * dt) + (0.5 * ((t_acc_ >= 0) ? acceleration_limit_ : -acceleration_limit_ ) * dt * dt);
      //current_velocity_ += ((t_acc_ >= 0) ? acceleration_limit_ : -acceleration_limit_)*dt;
      total_time_ += dt;
      if(total_time_ >= fabs(t_acc_))
      {
        total_time_ = 0.0;
        t_acc_ = 0.0;
      }
    }
    else if (t_const_ != 0.0)
    {
      //if(name_ == "J1")
        //ROS_INFO("const");
      command = (velocity_ * dt);
      total_time_ += dt;
      if (total_time_ >= fabs(t_const_))
      {
        total_time_ = 0.0;
        t_const_ = 0.0;
      }
    }
    else if (t_dacc_ != 0.0)
    {
      //if(name_ == "J1")
        //ROS_INFO("dacc");
      command = (velocity_ * dt) + (0.5 * ((t_dacc_ >= 0) ? acceleration_limit_ : -acceleration_limit_) * dt * dt);
      //current_velocity_ += ((t_dacc_ >= 0) ? acceleration_limit_ : -acceleration_limit_)*dt;
      total_time_ += dt;
      if (total_time_ >= fabs(t_dacc_))
      {
        total_time_ = 0.0;
        t_dacc_ = 0.0;
      }
    }



    command = last_commanded_position_ + command;

    //if(name_ == "J1")
      //ROS_INFO("command %f %f", command, velocity_);

    if (command > upper_limit_)
    {
      command = upper_limit_;
      ROS_WARN_STREAM_THROTTLE(1.0, name_ + " is reached upper limit");
    }
    else if (command < lower_limit_)
    {
      command = lower_limit_;
      ROS_WARN_STREAM_THROTTLE(1.0, name_ + " is reached lower limit");
    }

    last_commanded_position_ = command;

    //ROS_INFO("error %f ", fabs(last_commanded_position_ - desired_position_));
    if(fabs(last_commanded_position_ - desired_position_) < 0.001)
    {
      //ROS_INFO("reached");
      t_to_zero_ = t_acc_ = t_const_ = t_dacc_ = current_velocity_ = total_time_ = 0.0;
      touched_ = false; //stop the movement
    }

    return command;
  }
  else
  {
    return desired_position_;
  }
}

void RC7MJoint::setFeedbackData(double feedback)
{
  //save current position
  double last_position = position_;

  //set new position from feedback data
  position_ = feedback;

  //compute joint velocity in unit/sec
  ros::Time t = ros::Time::now();
  velocity_ = ((position_ - last_position) * 1.0e9)/(t - last_update_).toNSec();
  //if(name_ == "J1")
  //{
    //ROS_INFO("%6.3f %6.3f %6.3f", position_, last_position, velocity_);
  //}

  //save last updated time
  last_update_ = t;
}

double RC7MJoint::setPosition(double position)
{
  //check position limit
  if ((position > upper_limit_) || (position < lower_limit_))
  {
    ROS_WARN_STREAM(name_ + " position out of range [" << lower_limit_ << ", " << upper_limit_ << "]");
    return position_;
  }

  double s = position - position_;
  double u = velocity_;
  int target_direction = (s >= 0) ? 1 : -1;
  int velocity_direction = (u >= 0) ? 1 : -1;


  if(target_direction != velocity_direction)
  {
    t_to_zero_ = -u / acceleration_limit_;
    u = 0;
  }

  double max_t1 = ((target_direction * velocity_limit_) - u) / (target_direction * acceleration_limit_);
  double max_s1 = (u * max_t1) + (0.5 * (target_direction*acceleration_limit_) * (max_t1 * max_t1));

  //ROS_INFO("%s: ds: %6.3f v: %6.3f  max_t1: %6.3f max_s1: %6.3f", name_.c_str(), s, u, max_t1, max_s1);

  if(fabs(max_s1) >= fabs(s/2))
  {
    double b = u;
    double a = 0.5 * target_direction * acceleration_limit_;
    double c = -s/2;

    //b^2 - 4ac
    double b2_4ac = (b*b) - (4*a*c);
    double t0 = (-u + sqrt(b2_4ac)) / (2*a);
    double t1 = (-u - sqrt(b2_4ac)) / (2*a);
    if(t0 >= 0)
      t_acc_ = t0;
    else
      t_acc_ = t1;

    t_const_ = 0.0;
    t_dacc_ = (u + (target_direction*acceleration_limit_)*t_acc_)/(target_direction*acceleration_limit_);
    //ROS_INFO("%s: b2_4ac: %6.3f t0: %6.3f t1: %6.3f", name_.c_str(), b2_4ac, t0, t1);
  }
  else
  {
    //Trapezoidal profile
    t_acc_ = max_t1;
    t_dacc_ = velocity_limit_/acceleration_limit_;
    double s_dacc = 0.5 * (target_direction) * velocity_limit_ * t_dacc_;
    double s_const = s - (s_dacc + max_s1);
    t_const_ = s_const / velocity_limit_;
  }

  t_acc_ *= target_direction;
  t_dacc_ *= -target_direction;


  //ROS_INFO("%s: t_to_zero: %6.3f t_acc: %6.3f t_const: %6.3f t_dacc: %6.3f", name_.c_str(), t_to_zero_, t_acc_, t_const_, t_dacc_);

  mutex_.lock();
  total_time_ = 0.0;
  desired_position_ = position;
  touched_ = true;
  mutex_.unlock();
  return position;
}


double RC7MJoint::setPositionRelative(double position)
{
  double temp = position_ + position;
  return setPosition(temp);
}

diagnostic_msgs::DiagnosticStatus RC7MJoint::getDiagnostics()
{
  return hg::Joint::getDiagnostics();
}

void RC7MJoint::callbackJointPosition(const std_msgs::Float64& position)
{
  //check controller status
  if(!controller_->active())
  {
    setPosition(position.data);
  }
  else
  {
    ROS_WARN("controller is active");
  }
}

void RC7MJoint::callbackJointPositionRelative(const std_msgs::Float64& position)
{
  //check controller status
  if(!controller_->active())
  {
    setPositionRelative(position.data);
  }
  else
  {
    ROS_WARN("controller is active");
  }
}



void RC7MJoint::callbackJointPositionDegree(const std_msgs::Float64& position)
{
  //check controller status
  if(!controller_->active())
  {
    ROS_INFO("start");
    setPosition((position.data * M_PI) / 180.0);
  }
  else
  {
    ROS_WARN("controller is active");
  }
}

void RC7MJoint::callbackJointVelocity(const std_msgs::Float64& velocity)
{
  //check controller status
}

