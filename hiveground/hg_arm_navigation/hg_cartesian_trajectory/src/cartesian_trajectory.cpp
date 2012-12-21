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

using namespace hg_cartesian_trajectory;

CartesianTrajectoryPlanner::CartesianTrajectoryPlanner()
  : nh_(), nh_private_("~")
{
  //ROS_INFO_STREAM("namespace: " << nh_.getNamespace());
  //ROS_INFO_STREAM("namespace: " << nh_private_.getNamespace());
  collision_models_interface_ =
      boost::shared_ptr<planning_environment::CollisionModelsInterface>(new planning_environment::CollisionModelsInterface("robot_description"));
}

CartesianTrajectoryPlanner::~CartesianTrajectoryPlanner()
{
  joint_state_monitor.stop();
}

void CartesianTrajectoryPlanner::run()
{
  if(!initialize(nh_private_.getNamespace()))
    return;
  if (!collision_models_interface_->loadedModels())
  {
    ROS_ERROR("Collision models not loaded.");
    return;
  }
}

bool CartesianTrajectoryPlanner::initialize(const std::string& param_server_prefix)
{
  std::vector<std::string> group_names;
  if(!getGroupNamesFromParamServer(param_server_prefix,group_names))
  {
    ROS_ERROR("Could not find groups for planning under %s",param_server_prefix.c_str());
    return false;
  }

  display_trajectory_publisher_ =
      nh_private_.advertise<arm_navigation_msgs::DisplayTrajectory>("joint_trajectory_display", 1);
  display_trajectory_path_publisher_ =
      nh_private_.advertise<nav_msgs::Path>("joint_path_display", 1);

  const KinematicModelGroupConfigMap &group_config_map =
      collision_models_interface_->getKinematicModel()->getJointModelGroupConfigMap();
  for (KinematicModelGroupConfigMap::const_iterator it = group_config_map.begin(); it != group_config_map.end(); it++)
  {
    std::string ik_service_name = collision_models_interface_->getKinematicModel()->getRobotName() + "_" + it->first + "_kinematics/";
    std::string ik_collision_aware_name = ik_service_name + "get_constraint_aware_ik";
    std::string ik_none_collision_aware_name = ik_service_name + "get_ik";
    ROS_DEBUG_STREAM("Tip link: " << it->second.tip_link_);
    ROS_DEBUG_STREAM(ik_service_name);
    ROS_DEBUG_STREAM(ik_collision_aware_name);
    ROS_DEBUG_STREAM(ik_none_collision_aware_name);

    while (!ros::service::waitForService(ik_collision_aware_name, ros::Duration(1.0))) { }
    while (!ros::service::waitForService(ik_none_collision_aware_name, ros::Duration(1.0))) { }
    tip_link_map_[it->first] = it->second.tip_link_;
    ik_client_map_[it->first] = nh_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>(ik_collision_aware_name, true);
    ik_none_collision_client_map_[it->first] = nh_.serviceClient<kinematics_msgs::GetPositionIK>(ik_none_collision_aware_name, true);

    std::string arm_controller_name = collision_models_interface_->getKinematicModel()->getRobotName() + "/follow_joint_trajectory";
    action_client_map_[it->first] =
        FollowJointTrajectoryClientPtr(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(arm_controller_name, true));
    while(ros::ok() && !action_client_map_[it->first]->waitForServer(ros::Duration(1.0))) { }
  }

  service_ = nh_private_.advertiseService("plan_cartesian_path", &CartesianTrajectoryPlanner::executeCartesianTrajectoryPlanner, this);

  return true;
}

bool CartesianTrajectoryPlanner::getGroupNamesFromParamServer(const std::string &param_server_prefix,
                                                                      std::vector<std::string> &group_names)
{
  XmlRpc::XmlRpcValue group_list;
  if(!nh_.getParam(param_server_prefix+"/groups", group_list))
  {
    ROS_ERROR("Could not find parameter %s on param server",(param_server_prefix+"/groups").c_str());
    return false;
  }
  if(group_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Group list should be of XmlRpc Array type");
    return false;
  }
  for (int32_t i = 0; i < group_list.size(); ++i)
  {
    if(group_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("Group names should be strings");
      return false;
    }
    group_names.push_back(static_cast<std::string>(group_list[i]));
    ROS_INFO("Adding group: %s",group_names.back().c_str());
  }
  return true;
};

bool CartesianTrajectoryPlanner::executeCartesianTrajectoryPlanner(HgCartesianTrajectory::Request &request,
                                                                            HgCartesianTrajectory::Response &respond)
{
  switch(request.type)
  {
    case HgCartesianTrajectory::Request::SIMPLE_IK:
      ROS_INFO_STREAM("Working");
      //respond.error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS;
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

//run inverse kinematics on a PoseStamped (7-dof pose
//(position + quaternion orientation) + header specifying the
//frame of the pose)
//tries to stay close to double start_angles[7]
//returns the solution angles in double solution[7]
bool CartesianTrajectoryPlanner::runIk(const std::string& group_name,
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
  ik_request.timeout = ros::Duration(5.0);

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
    success = runIk(request.motion_plan_request.group_name,
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
  double time_from_start = 0.25;
  for (int i = 0; i < trajectory_length; i++)
  {
    //fill in the joint positions (velocities of 0 mean that the arm
    //will try to stop briefly at each waypoint)
    trajectory.points[i + 1].positions = joint_trajectory[i];
    trajectory.points[i + 1].velocities.resize(joint_trajectory.size(), 0);

    //compute a desired time for this trajectory point using a max
    //joint velocity
    double max_joint_move = 0;
    for (size_t j = 0; j < joint_trajectory.size(); j++)
    {
      double joint_move = (trajectory.points[i + 1].positions[j] - trajectory.points[i].positions[j]);
      if (joint_move > max_joint_move)
        max_joint_move = joint_move;
    }
    double seconds = max_joint_move / HG_MAX_SIMPLE_IK_JOINT_VEL;
    ROS_INFO("max_joint_move: %0.3f, seconds: %0.3f", max_joint_move, seconds);
    time_from_start += seconds;
    trajectory.points[i + 1].time_from_start = ros::Duration(time_from_start);
  }

  //when to start the trajectory
  trajectory.header.stamp = ros::Time::now() + ros::Duration(0.25);
  trajectory.header.frame_id = collision_models_interface_->getWorldFrameId();
  respond.trajectory.joint_trajectory = trajectory;
  respond.error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS;

  spline_smoother::CubicParameterizedTrajectory trajectory_generator;
  std::vector<arm_navigation_msgs::JointLimits> joint_limits;
  spline_smoother::SplineTrajectory spline_trajectory;
  arm_navigation_msgs::JointLimits joint_limit;
  joint_limit.has_position_limits = true;
  joint_limit.has_velocity_limits = true;
  joint_limit.has_acceleration_limits = true;
  joint_limit.max_velocity = HG_MAX_SIMPLE_IK_JOINT_VEL;
  joint_limit.max_acceleration = 1.0;
  joint_limit.joint_name = "J1";
  joint_limit.min_position = -2.792526;
  joint_limit.min_position = 2.792526;
  joint_limits.push_back(joint_limit);

  joint_limit.joint_name = "J2";
  joint_limit.min_position = -2.094394;
  joint_limit.min_position = 2.094394;
  joint_limits.push_back(joint_limit);

  joint_limit.joint_name = "J3";
  joint_limit.min_position = 0.331612;
  joint_limit.min_position = 2.792526;
  joint_limits.push_back(joint_limit);

  joint_limit.joint_name = "J4";
  joint_limit.min_position = -2.792526;
  joint_limit.min_position = 2.792526;
  joint_limits.push_back(joint_limit);

  joint_limit.joint_name = "J5";
  joint_limit.min_position = -2.094394;
  joint_limit.min_position = 2.094394;
  joint_limits.push_back(joint_limit);

  joint_limit.joint_name = "J6";
  joint_limit.min_position = -6.283185;
  joint_limit.min_position = 6.283185;
  joint_limits.push_back(joint_limit);



  trajectory_generator.parameterize(trajectory, joint_limits, spline_trajectory);
  ROS_INFO("trajectory size %d spline size:%d", trajectory.points.size(), spline_trajectory.segments.size());
  trajectory.points.clear();
  double time = 0;
  for(int i = 0; i < spline_trajectory.segments.size(); i++)
  {
    int step = (spline_trajectory.segments[i].duration.toSec() / 0.008) + 0.5;
    ROS_INFO("time %f total step: %d", spline_trajectory.segments[i].duration.toSec(), step);

    int n_joint = spline_trajectory.segments[i].joints.size();
    for (int k = 0; k < step; k++)
    {
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions.resize(n_joint);
      point.velocities.resize(n_joint);
      point.accelerations.resize(n_joint);
      double t = 0.008 * k;
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
          ROS_INFO("Step %d J%d %f %f %f", k, j+1, point.positions[j], point.velocities[j], point.accelerations[j]);
        }
      }
      if((k % 100) == 0)
      {
        ROS_INFO("================");
      }
      point.time_from_start = ros::Duration(t + time);
      trajectory.points.push_back(point);


    }
    time += spline_trajectory.segments[i].duration.toSec();
  }

  ROS_INFO("trajectory size %d", trajectory.points.size());

  if(display_trajectory_publisher_.getNumSubscribers() != 0)
  {
    arm_navigation_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.model_id = request.motion_plan_request.group_name;
    display_trajectory.trajectory.joint_trajectory = trajectory;
    display_trajectory.robot_state.joint_state = joint_state_monitor.getJointStateRealJoints();
    display_trajectory_publisher_.publish(display_trajectory);
  }

  if(display_trajectory_path_publisher_.getNumSubscribers() != 0)
  {
    nav_msgs::Path path;
    path.header = trajectory.header;
    for (int i = 0; i < trajectory_length; i++)
    {
      stamped_pose.pose = request.poses[i];
      path.poses.push_back(stamped_pose);
    }
    display_trajectory_path_publisher_.publish(path);
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  action_client_map_[request.motion_plan_request.group_name]->sendGoal(goal);
  action_client_map_[request.motion_plan_request.group_name]->waitForResult();
  if(action_client_map_[request.motion_plan_request.group_name]->getState() ==
      actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the arm finished the trajectory!");
  }
  else
  {
    ROS_INFO("The arm failed to execute the trajectory.");
  }
}



