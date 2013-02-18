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

#include <hg_cartesian_trajectory/cartesian_trajectory.h>
#include <spline_smoother/cubic_parameterized_trajectory.h>
#include <spline_smoother/linear_trajectory.h>
#include <spline_smoother/lspb_trajectory.h>
#include <spline_smoother/clamped_cubic_spline_smoother.h>


using namespace hg_cartesian_trajectory;

CartesianTrajectoryPlanner::CartesianTrajectoryPlanner()
  : PlanningBase()
{

}

CartesianTrajectoryPlanner::~CartesianTrajectoryPlanner()
{
  joint_state_monitor.stop();
}

bool CartesianTrajectoryPlanner::run()
{
  return PlanningBase::run();
}

bool CartesianTrajectoryPlanner::initialize(const std::string& param_server_prefix)
{
  if(!PlanningBase::initialize(param_server_prefix))
    return false;

  nh_private_.getParam("time_step", time_step_);
  ROS_INFO("time_step: %f", time_step_);

//  const KinematicModelGroupConfigMap &group_config_map =
//      collision_models_interface_->getKinematicModel()->getJointModelGroupConfigMap();
//  for (KinematicModelGroupConfigMap::const_iterator it = group_config_map.begin(); it != group_config_map.end(); it++)
//  {
//    std::string arm_controller_name = collision_models_interface_->getKinematicModel()->getRobotName()
//        + "/follow_joint_trajectory";
//    action_client_map_[it->first] = FollowJointTrajectoryClientPtr(
//        new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(arm_controller_name, true));
//    while (ros::ok() && !action_client_map_[it->first]->waitForServer(ros::Duration(1.0))) { }
//  }

  service_ = nh_private_.advertiseService("plan_cartesian_path", &CartesianTrajectoryPlanner::executeCartesianTrajectoryPlanner, this);

  return true;
}

bool CartesianTrajectoryPlanner::executeCartesianTrajectoryPlanner(HgCartesianTrajectory::Request &request,
                                                                            HgCartesianTrajectory::Response &respond)
{
  switch(request.type)
  {
    case HgCartesianTrajectory::Request::SIMPLE_IK:
      ROS_INFO_STREAM("Working");
      planSimpleIKTrajectory(request, respond);
      break;
    case HgCartesianTrajectory::Request::LINE:
    case HgCartesianTrajectory::Request::CIRCLE:
    case HgCartesianTrajectory::Request::PATH:
      ROS_WARN("not yet implement planning mode");
      respond.error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::PLANNING_FAILED;
      break;
  }
  return (respond.error_code.val == arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS);
}

bool CartesianTrajectoryPlanner::runIK(const std::string& group_name,
                                            const arm_navigation_msgs::RobotState& robot_state,
                                            geometry_msgs::PoseStamped pose,
                                            std::vector<double>& start_position,
                                            std::vector<double>& solution)
{
  kinematics_msgs::GetPositionIK::Request ik_request;
  kinematics_msgs::GetPositionIK::Response ik_response;

  ik_request.ik_request.ik_link_name = tip_link_map_[group_name];
  ik_request.ik_request.pose_stamped.header.frame_id = collision_models_interface_->getWorldFrameId();
  ik_request.ik_request.pose_stamped.header.stamp = ros::Time::now();
  ik_request.ik_request.pose_stamped = pose;
  ik_request.ik_request.ik_seed_state.joint_state.position = start_position;
  ik_request.ik_request.ik_seed_state.joint_state.name = robot_state.joint_state.name;
  ik_request.timeout = ros::Duration(1.0);

  ROS_DEBUG(
      "request pose: (%0.3f %0.3f %0.3f) (%0.3f %0.3f %0.3f %0.3f)",
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
      pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

  bool ik_service_call = ik_none_collision_client_map_[group_name].call(ik_request, ik_response);
  if (!ik_service_call)
  {
    ROS_ERROR("IK service call failed!");
    return 0;
  }

  if (ik_response.error_code.val == ik_response.error_code.SUCCESS)
  {
    solution = ik_response.solution.joint_state.position;
    for(int i = 0; i < (int)solution.size(); i++)
    {
      ROS_DEBUG("Joint %s solution angles [%d]: %f", robot_state.joint_state.name[i].c_str(), i, solution[i]);
    }
    ROS_DEBUG("IK service call succeeded");
    return 1;
  }
  ROS_DEBUG("IK service call error code: %d", ik_response.error_code.val);
  return 0;
}

void CartesianTrajectoryPlanner::planSimpleIKTrajectory(HgCartesianTrajectory::Request &request,
                                                                HgCartesianTrajectory::Response &respond)
{
  int trajectory_length = request.poses.size();
  ROS_DEBUG("trajectory_length: %d", trajectory_length);

  //IK takes in Cartesian poses stamped with the frame they belong to
  geometry_msgs::PoseStamped stamped_pose;
  stamped_pose.header = request.header;
  stamped_pose.header.stamp = ros::Time::now();
  bool success = false;
  std::vector<std::vector<double> > joint_trajectory;

  std::vector<double> last_position = request.motion_plan_request.start_state.joint_state.position;

  //find IK solutions for each point along the trajectory
  //and stick them into joint_trajectory
  for (int i = 0; i < trajectory_length; i++)
  {
    stamped_pose.pose = request.poses[i];
    std::vector<double> trajectory_point;
    success = runIK(request.motion_plan_request.group_name,
                    request.motion_plan_request.start_state,
                    stamped_pose,
                    last_position,
                    trajectory_point);
    if (!success)
    {
      ROS_ERROR("IK solution not found for trajectory point number %d!\n", i);
      respond.error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::NO_IK_SOLUTION;
      return;
    }
    joint_trajectory.push_back(trajectory_point);
    last_position = trajectory_point;
  }

  trajectory_msgs::JointTrajectory trajectory;

  //get the current joint angles
  std::vector<double> current_position = request.motion_plan_request.start_state.joint_state.position;

  //fill the goal message with the desired joint trajectory
  trajectory.points.resize(trajectory_length + 1);

  //set the first trajectory point to the current position
  trajectory.joint_names = request.motion_plan_request.start_state.joint_state.name;
  trajectory.points[0].positions = current_position;
  trajectory.points[0].velocities.resize(current_position.size(), 0);

  //make the first trajectory point start 0.25 seconds from when we run
  trajectory.points[0].time_from_start = ros::Duration(0.25);

  //fill in the rest of the trajectory
  for (int i = 0; i < trajectory_length; i++)
  {
    //fill in the joint positions (velocities of 0 mean that the arm
    //will try to stop briefly at each waypoint)
    trajectory.points[i + 1].positions = joint_trajectory[i];
    trajectory.points[i + 1].velocities.resize(joint_trajectory.size(), 0);
    trajectory.points[i + 1].time_from_start = ros::Duration(0);
  }

  //when to start the trajectory
  trajectory.header.stamp = ros::Time::now() + ros::Duration(0.25);
  trajectory.header.frame_id = collision_models_interface_->getWorldFrameId();

  kinematics_msgs::GetKinematicSolverInfo::Request ik_info_request;
  kinematics_msgs::GetKinematicSolverInfo::Response ik_info_respond;
  if (!ik_info_client_map_[request.motion_plan_request.group_name].call(ik_info_request, ik_info_respond))
  {
    ROS_ERROR("Cannot get IK solver information");
    respond.error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::PLANNING_FAILED;
    return;
  }

  spline_smoother::CubicParameterizedTrajectory trajectory_generator;
  spline_smoother::SplineTrajectory spline_trajectory;
  for(size_t i = 0; i < ik_info_respond.kinematic_solver_info.limits.size(); i++)
  {
    ik_info_respond.kinematic_solver_info.limits[i].has_velocity_limits = 1;
    ik_info_respond.kinematic_solver_info.limits[i].max_velocity = 1.0;
  }

  trajectory_generator.parameterize(trajectory, ik_info_respond.kinematic_solver_info.limits, spline_trajectory);

  ROS_DEBUG("trajectory size %lu spline size:%lu", trajectory.points.size(), spline_trajectory.segments.size());
  trajectory.points.clear();
  double time = 0;
  for(size_t i = 0; i < spline_trajectory.segments.size(); i++)
  {
    int step = (spline_trajectory.segments[i].duration.toSec() / time_step_) + 0.5;
    ROS_INFO("time %f total step: %d", spline_trajectory.segments[i].duration.toSec(), step);

    int n_joint = spline_trajectory.segments[i].joints.size();
    for (int k = 0; k < step; k++)
    {
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions.resize(n_joint);
      point.velocities.resize(n_joint);
      point.accelerations.resize(n_joint);
      double t = time_step_ * k;
      for (int j = 0; j < n_joint; j++)
      {
        double a0 = spline_trajectory.segments[i].joints[j].coefficients[0];
        double a1 = spline_trajectory.segments[i].joints[j].coefficients[1];
        double a2 = spline_trajectory.segments[i].joints[j].coefficients[2];
        double a3 = spline_trajectory.segments[i].joints[j].coefficients[3];
        point.positions[j] = a0 + (a1 * t) + (a2 * t * t) + (a3 * t * t * t);
        point.velocities[j] = a1 + (2 * a2 * t) + (3 * a3 * t * t);
        point.accelerations[j] = 2 * a2 + (6 * a3 * t);

        if((k % 100) == 0)
        {
          ROS_DEBUG("Step %d J%d %f %f %f", k, j+1, point.positions[j], point.velocities[j], point.accelerations[j]);
        }
      }
      if((k % 100) == 0)
      {
        ROS_DEBUG("================");
      }
      point.time_from_start = ros::Duration(t + time);
      trajectory.points.push_back(point);


    }
    time += spline_trajectory.segments[i].duration.toSec();
  }

  ROS_DEBUG("trajectory size %lu", trajectory.points.size());
  respond.trajectory.joint_trajectory = trajectory;
  respond.error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS;

  /*
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  action_client_map_[request.motion_plan_request.group_name]->sendGoal(goal);
  action_client_map_[request.motion_plan_request.group_name]->waitForResult();
  if(action_client_map_[request.motion_plan_request.group_name]->getState() ==
      actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_DEBUG("Hooray, the arm finished the trajectory!");
  }
  else
  {
    ROS_DEBUG("The arm failed to execute the trajectory.");
  }
  */
}



