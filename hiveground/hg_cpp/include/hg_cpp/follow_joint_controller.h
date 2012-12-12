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

#ifndef HG_FOLLOW_JOINT_CONTROLLER_H_
#define HG_FOLLOW_JOINT_CONTROLLER_H_

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <boost/thread.hpp>
#include <queue>

#include <hg_cpp/controller.h>

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

namespace hg
{

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> FollowJointTrajectoryActionServer;
typedef boost::shared_ptr<FollowJointTrajectoryActionServer> FollowJointTrajectoryActionServerPtr;


class FollowJointController : public hg::Controller
{
protected:
  //action server
  virtual void followJointGoalActionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

public:
  FollowJointTrajectoryActionServerPtr action_server_;
  control_msgs::FollowJointTrajectoryGoalConstPtr action_goal_;
};


}


#endif
