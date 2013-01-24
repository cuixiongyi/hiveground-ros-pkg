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

#include <hg_cpp/follow_joint_controller.h>
#include <hg_cpp/joint.h>
#include <hg_cpp/controller_node.h>
#include <spline_smoother/SplineTrajectory.h>
#include <spline_smoother/cubic_parameterized_trajectory.h>

using namespace hg;

void FollowJointController::initilize(hg::ControllerNode* node, const std::string& name)
{
  hg::Controller::initilize(node, name);

  XmlRpc::XmlRpcValue joints;
  node_->nh_private_.getParam("controllers/" + name_ + "/joints", joints);
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

void FollowJointController::followJointGoalActionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
  control_msgs::FollowJointTrajectoryResult result;
  if (goal->trajectory.points.size() == 0)
  {
    ROS_ERROR("Trajectory is empty");
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    action_server_->setAborted(result, "Trajectory empty");
    return;
  }

  trajectory_msgs::JointTrajectory trajectory = goal->trajectory;
  std::vector<boost::shared_ptr<hg::Joint> >::iterator joint_it;

  int num_joints = trajectory.joint_names.size();
  for (int i = 0; i < num_joints; i++)
  {
    bool found = false;
    for (joint_it = joints_.begin(); joint_it != joints_.end(); joint_it++)
    {
      if ((*joint_it)->name_ == trajectory.joint_names[i])
      {
        found = true;
      }
    }
    if (!found)
    {
      ROS_ERROR("Trajectory joints do not match with controller joints");
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      action_server_->setAborted(result, "Trajectory joint names do not match with controlled joints");
      return;
    }
  }


  int num_traj = trajectory.points.size();
  ROS_DEBUG("Got trajectory with %d points", num_traj);
  ros::Time trajectory_start_time = trajectory.header.stamp;
  ROS_DEBUG_STREAM("Time to start: " << trajectory_start_time);
  double time_before_start = (trajectory.header.stamp - ros::Time::now()).toSec();
  ROS_DEBUG_STREAM("Time before start: " << time_before_start);

  //check velocity at each point
  for (int i = 0; i < num_traj; i++)
  {
    if (trajectory.points[i].velocities.empty())
      trajectory.points[i].velocities.resize(num_joints, 0);
  }

  //check current position with start point position
  trajectory_msgs::JointTrajectoryPoint start_point;
  std::vector<double> positions_diff;
  std::vector<double> velocities_diff;
  start_point.positions.resize(num_joints, 0);
  start_point.velocities.resize(num_joints, 0);
  positions_diff.resize(num_joints, 0);
  velocities_diff.resize(num_joints, 0);
  bool different_start_position = false;
  for(int i = 0; i < num_joints; i++)
  {
    start_point.positions[i] = joints_[i]->position_;
    start_point.velocities[i] = joints_[i]->velocity_;
    positions_diff[i] = start_point.positions[i] - trajectory.points[0].positions[i];
    velocities_diff[i] = start_point.velocities[i] - trajectory.points[0].velocities[i];
    ROS_DEBUG("Joint %d diff: [p]:%f [v]:%f", i, positions_diff[i], velocities_diff[i]);
    if(positions_diff[i] != 0 || velocities_diff[i] != 0)
    {
      different_start_position = true;
    }
  }

  if(is_preempted_)
  {
    preempted_point_.time_from_start.fromSec(0);
    trajectory.points.insert(trajectory.points.begin(), preempted_point_);
    is_preempted_ = false;
  }
  else
  {
    if(different_start_position)
    {
      //add current position to the trajectory
      ROS_DEBUG("Trajectory did not start from current state");
      trajectory.points.insert(trajectory.points.begin(), start_point);
    }
  }


  std::vector<arm_navigation_msgs::JointLimits> limits;
  arm_navigation_msgs::JointLimits limit;
  for (int i = 0; i < num_joints; i++)
  {
    limit.has_position_limits = 1;
    limit.max_position = joints_[i]->joint_info_->limits->upper;
    limit.min_position = joints_[i]->joint_info_->limits->lower;
    limit.has_velocity_limits = 1;
    limit.max_velocity = joints_[i]->joint_info_->limits->velocity;
    limit.has_acceleration_limits = 1;
    limit.max_acceleration = 4.0;
    limits.push_back(limit);
  }

  spline_smoother::SplineTrajectory spline_trajectory;
  spline_smoother::CubicTrajectory trajectory_generator;
  trajectory_generator.parameterize(trajectory, limits, spline_trajectory);



  ROS_DEBUG("trajectory size %lu spline size:%lu", trajectory.points.size(), spline_trajectory.segments.size());


  trajectory.points.clear();
  double time = 0;
  for (size_t i = 0; i < spline_trajectory.segments.size(); i++)
  {
    int step = (spline_trajectory.segments[i].duration.toSec() * rate_) + 0.5;
    ROS_DEBUG("time %f total step: %d", spline_trajectory.segments[i].duration.toSec(), step);
    double a0, a1, a2, a3;
    for (int k = 0; k < step; k++)
    {
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions.resize(num_joints);
      point.velocities.resize(num_joints);
      point.accelerations.resize(num_joints);
      double t = k / rate_;
      for (int j = 0; j < num_joints; j++)
      {
        a0 = spline_trajectory.segments[i].joints[j].coefficients[0];
        a1 = spline_trajectory.segments[i].joints[j].coefficients[1];
        a2 = spline_trajectory.segments[i].joints[j].coefficients[2];
        a3 = spline_trajectory.segments[i].joints[j].coefficients[3];
        point.positions[j] = a0 + (a1 * t) + (a2 * t * t) + (a3 * t * t * t);
        point.velocities[j] = a1 + (2 * a2 * t) + (3 * a3 * t * t);
        point.accelerations[j] = 2 * a2 + (6 * a3 * t);
      }
      point.time_from_start = ros::Duration(t + time);
      trajectory.points.push_back(point);

    }
    time += spline_trajectory.segments[i].duration.toSec();
  }

  ROS_DEBUG("trajectory size %lu", trajectory.points.size());



  try
  {
    is_active_ = true;

    //wait for the start time
    while (ros::Time::now() < trajectory_start_time)
    {
      ros::Duration(0.001).sleep();
    }

    ros::Duration skip_duration = trajectory.points[0].time_from_start;
    bool success = true;
    std::vector<boost::shared_ptr<hg::Joint> >::iterator it;
    trajectory_msgs::JointTrajectoryPoint point;

    for (size_t i = 0; i < trajectory.points.size(); i++)
    {
      ros::Time end_time = trajectory_start_time + (point.time_from_start - skip_duration);

      point = trajectory.points[i];
      it = joints_.begin();
      for (size_t k = 0; k < point.positions.size(); k++, it++)
      {
        (*it)->setPosition(point.positions[k]);
      }

      while ((ros::Time::now() < end_time) && (!action_server_->isPreemptRequested()))
      {
        ros::Duration(0.0001).sleep();
      }

      if (action_server_->isPreemptRequested() || !ros::ok())
      {
        ROS_DEBUG_STREAM("Preempted!! @ point" << i << ":" << point);
        action_server_->setPreempted();
        success = false;
        is_preempted_ = true;
        preempted_point_ = point;
        break;
      }

    }
    is_active_ = false;

    if (success)
    {
      result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
      action_server_->setSucceeded(result);
    }

  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    action_server_->setAborted(result);
  }
}

double FollowJointController::calculateMinimumTime(const trajectory_msgs::JointTrajectoryPoint &start,
                                                          const trajectory_msgs::JointTrajectoryPoint &end,
                                                          const std::vector<arm_navigation_msgs::JointLimits> &limits)
{
  double minJointTime(MAX_ALLOWABLE_TIME);
  double segmentTime(0);
  int num_joints = (int) start.positions.size();

  for(int i = 0; i < num_joints; i++)
  {
    minJointTime = minSegmentTime(start.positions[i],end.positions[i],start.velocities[i],end.velocities[i],limits[i]);
    if(segmentTime < minJointTime)
      segmentTime = minJointTime;
  }
  return segmentTime;
}

double FollowJointController::minSegmentTime(const double &q0,
                                                   const double &q1,
                                                   const double &v0,
                                                   const double &v1,
                                                   const arm_navigation_msgs::JointLimits &limit)
{
  //    double dq = jointDiff(q0,q1,limit);
  double dq = q1-q0;
  double vmax = limit.max_velocity;
  if( q0 == q1 && fabs(v0-v1) == 0.0)
  {
    return 0.0;
  }
  dq = q1-q0;
  double v = vmax;
  double solution;
  std::vector<double> solution_vec;

  double a1 = 3*(v0+v1)*v - 3* (v0+v1)*v0 + (2*v0+v1)*(2*v0+v1);
  double b1 = -6*dq*v + 6 * v0 *dq - 6*dq*(2*v0+v1);
  double c1 = 9*dq*dq;

  double a2 = 3*(v0+v1)*v + 3* (v0+v1)*v0 - (2*v0+v1)*(2*v0+v1);
  double b2 = -6*dq*v - 6 * v0 *dq + 6*dq*(2*v0+v1);
  double c2 = -9*dq*dq;

  std::vector<double> t1,t2,t3,t4,t5,t6;

  if(quadSolve(a1,b1,c1,t1))
    for(unsigned int i=0; i < t1.size(); i++)
      solution_vec.push_back(t1[i]);

  if(quadSolve(a2,b2,c2,t2))
    for(unsigned int i=0; i < t2.size(); i++)
      solution_vec.push_back(t2[i]);
  double amax = -1.0;

  if(limit.has_acceleration_limits)
  {
    amax = limit.max_acceleration;
    double a3 = amax/2.0;
    double b3 = 2*v0+v1;
    double c3 = -3*dq;
    if(quadSolve(a3,b3,c3,t3))
      for(unsigned int i=0; i < t3.size(); i++)
        solution_vec.push_back(t3[i]);

    double a4 = amax/2.0;
    double b4 = -(2*v0+v1);
    double c4 = 3*dq;
    if(quadSolve(a4,b4,c4,t4))
      for(unsigned int i=0; i < t4.size(); i++)
        solution_vec.push_back(t4[i]);


    double a5 = amax;
    double b5 = (-2*v0-4*v1);
    double c5 = 6*dq;
    if(quadSolve(a5,b5,c5,t5))
      for(unsigned int i=0; i < t5.size(); i++)
        solution_vec.push_back(t5[i]);

    double a6 = amax;
    double b6 = (2*v0+4*v1);
    double c6 = -6*dq;
    if(quadSolve(a6,b6,c6,t6))
      for(unsigned int i=0; i < t6.size(); i++)
        solution_vec.push_back(t6[i]);
  }
  std::vector<double> positive_durations, valid_durations;
  for(unsigned int i=0; i < solution_vec.size(); i++)
  {
    if(solution_vec[i] > 0)
      positive_durations.push_back(solution_vec[i]);
  }

  for(unsigned int i=0; i < positive_durations.size(); i++)
  {
    ROS_DEBUG("Positive duration: %f",positive_durations[i]);
    if(validSolution(q0,q1,v0,v1,positive_durations[i],vmax,amax))
      valid_durations.push_back(positive_durations[i]);
  }

  ROS_DEBUG("valid size: %d",(int)valid_durations.size());
  std::sort(valid_durations.begin(),valid_durations.end());
  if(!valid_durations.empty())
    solution = valid_durations.front();
  else
    solution = 0.025;

  ROS_DEBUG(" ");
  ROS_DEBUG(" ");
  ROS_DEBUG(" ");
  return solution;
}

bool FollowJointController::validSolution(const double &q0,
                                                const double &q1,
                                                const double &v0,
                                                const double &v1,
                                                const double &dT,
                                                const double &vmax,
                                                const double &amax)
{
  if (dT == 0.0)
    return false;
  //  double a0 = q0;
  double a1 = v0;
  double a2 = (3*(q1-q0)-(2*v0+v1)*dT)/(dT*dT);
  double a3 = (2*(q0-q1)+(v0+v1)*dT)/(dT*dT*dT);

  double max_accn = fabs(2*a2);
  if(fabs(2*a2+6*a3*dT) > max_accn)
    max_accn = fabs(2*a2+6*a3*dT);

  bool max_vel_exists = false;
  double max_vel = 0.0;

  if(fabs(a3) > 0.0)
  {
    double max_vel_time = (-2*a2)/(6*a3);
    if (max_vel_time >= 0 && max_vel_time < dT)
    {
      max_vel_exists = true;
      max_vel = a1-(a2*a2)/(a3*3.0);
    }
  }

  if(amax > 0 && max_accn-amax > 1e-2)
  {
    ROS_DEBUG("amax allowed: %f, max_accn: %f",amax,max_accn);
    return false;
  }
  if(max_vel_exists)
    if(fabs(max_vel)-vmax > 1e-2)
    {
      ROS_DEBUG("vmax allowed: %f, max_vel: %f",vmax,max_vel);
      return false;
    }
  return true;
}

bool FollowJointController::quadSolve(const double &a,
                                           const double &b,
                                           const double &c,
                                           double &solution)
{
  double t1(0.0), t2(0.0);
  //  double eps = 2.2e-16;
  if (fabs(a) > 0.0)
  {
    double discriminant = b*b-4*a*c;
    if (discriminant >= 0)
    {
      t1 = (-b + sqrt(discriminant))/(2*a);
      t2 = (-b - sqrt(discriminant))/(2*a);
      ROS_DEBUG("t1:%f t2:%f",t1,t2);
      solution = std::max(t1,t2);
      ROS_DEBUG("Solution: %f",solution);
      return true;
    }
    else
      return false;
  }
  else
  {
    if(fabs(b) == 0.0)
      return false;
    t1 = -c/b;
    t2 = t1;
    solution = t1;
    ROS_DEBUG("Solution: %f",solution);
    return true;
  }
}

bool FollowJointController::quadSolve(const double &a,
                                           const double &b,
                                           const double &c,
                                           std::vector<double> &solution)
{
  double t1(0.0), t2(0.0);
  double eps = 2.2e-16;
  if (fabs(a) > eps)
  {
    double discriminant = b*b-4*a*c;
    if (discriminant >= 0)
    {
      t1 = (-b + sqrt(discriminant))/(2*a);
      t2 = (-b - sqrt(discriminant))/(2*a);
      ROS_DEBUG("t1:%f t2:%f",t1,t2);
      solution.push_back(t1);
      solution.push_back(t2);
      return true;
    }
    else
    {
      ROS_DEBUG("Discriminant: %f",discriminant);
      return false;
    }
  }
  else
  {
    if(fabs(b) == 0.0)
      return false;
    t1 = -c/b;
    t2 = t1;
    solution.push_back(t1);
    solution.push_back(t2);
    return true;
  }
}
