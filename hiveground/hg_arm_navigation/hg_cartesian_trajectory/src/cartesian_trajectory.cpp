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

using namespace hg_cartesian_trajectory;

CartesianTrajectoryPlanner::CartesianTrajectoryPlanner()
  : nh_(), nh_private_("~")
{
  //ROS_INFO_STREAM("namespace: " << nh_.getNamespace());
  //ROS_INFO_STREAM("namespace: " << nh_private_.getNamespace());
  collision_models_interface_ = new planning_environment::CollisionModelsInterface("robot_description");
}

CartesianTrajectoryPlanner::~CartesianTrajectoryPlanner()
{
  delete collision_models_interface_;
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

  const KinematicModelGroupConfigMap &group_config_map =
      collision_models_interface_->getKinematicModel()->getJointModelGroupConfigMap();
  for (KinematicModelGroupConfigMap::const_iterator it = group_config_map.begin(); it != group_config_map.end(); it++)
  {
    std::string ik_service_name = collision_models_interface_->getKinematicModel()->getRobotName() + "_" + it->first + "_kinematics/";
    std::string ik_collision_aware_name = ik_service_name + "get_constraint_aware_ik";
    std::string ik_none_collision_aware_name = ik_service_name + "get_ik";
    ROS_INFO_STREAM("Group: " << it->first);
    ROS_INFO_STREAM("tip link: " << it->second.tip_link_);
    ROS_INFO_STREAM(ik_service_name);
    ROS_INFO_STREAM(ik_collision_aware_name);
    ROS_INFO_STREAM(ik_none_collision_aware_name);

    while (!ros::service::waitForService(ik_collision_aware_name, ros::Duration(1.0))) { }
    while (!ros::service::waitForService(ik_none_collision_aware_name, ros::Duration(1.0))) { }
    tip_link_map_[it->first] = it->second.tip_link_;
    ik_client_map_[it->first] = nh_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>(ik_collision_aware_name, true);
    ik_none_collision_client_map_[it->first] = nh_.serviceClient<kinematics_msgs::GetPositionIK>(ik_none_collision_aware_name, true);
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
                                            std::vector<double>& last_position,
                                            std::vector<double>& solution)
{
  kinematics_msgs::GetPositionIK::Request ik_request;
  kinematics_msgs::GetPositionIK::Response ik_response;

  ik_request.timeout = ros::Duration(5.0);
  ik_request.ik_request.ik_seed_state.joint_state = robot_state.joint_state;
  ik_request.ik_request.ik_seed_state.joint_state.position = last_position;
  ik_request.ik_request.ik_link_name = tip_link_map_[group_name];
  ik_request.ik_request.pose_stamped = pose;
  ROS_INFO(
      "request pose: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

  bool ik_service_call = ik_none_collision_client_map_[group_name].call(ik_request, ik_response);
  if (!ik_service_call)
  {
    ROS_ERROR("IK service call failed!");
    return 0;
  }
  if (ik_response.error_code.val == ik_response.error_code.SUCCESS)
  {
    solution = ik_response.solution.joint_state.position;
    ROS_INFO(
        "solution angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", solution[0], solution[1], solution[2], solution[3], solution[4], solution[5], solution[6]);
    ROS_INFO("IK service call succeeded");
    return 1;
  }
  ROS_INFO("IK service call error code: %d", ik_response.error_code.val);
  return 0;
}

void CartesianTrajectoryPlanner::planSimpleIKTrajectory(HgCartesianTrajectory::Request &request,
                                                                HgCartesianTrajectory::Response &respond)
{
  int trajectory_length = request.poses.size();

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
      double joint_move = fabs(trajectory.points[i + 1].positions[j] - trajectory.points[i].positions[j]);
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
  respond.trajectory.joint_trajectory = trajectory;
  respond.error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS;
}



