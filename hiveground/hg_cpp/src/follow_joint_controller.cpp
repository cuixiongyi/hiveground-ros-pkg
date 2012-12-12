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

using namespace hg;


void FollowJointController::followJointGoalActionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
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
      ROS_ERROR("Trajectory joints do not match with controller joints");
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      action_server_->setAborted(result, "Trajectory joint names do not match with controlled joints");
      return;
    }
  }

  if(trajectory.points.size() == 0)
  {
    ROS_ERROR("Trajectory is empty");
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL ;
    action_server_->setAborted(result, "Trajectory empty");
    return;
  }

  ROS_DEBUG("Got trajectory with %d points", (int)trajectory.points.size());

  ros::Time trajectory_start_time = trajectory.header.stamp;

  //wait for start time
  while (ros::Time::now() < trajectory_start_time)
  {
    ros::Duration(0.001).sleep();
  }

  is_active_ = true;
  ros::Duration skip_duration = trajectory.points[0].time_from_start;
  bool success = true;
  ros::Time last_time = trajectory_start_time + skip_duration;
  for (size_t i = 0; i < trajectory.points.size(); i++)
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
      action_server_->setPreempted();
      success = false;
      break;
    }
  }
  is_active_ = false;

  if(success)
  {
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    action_server_->setSucceeded(result);
  }
}
