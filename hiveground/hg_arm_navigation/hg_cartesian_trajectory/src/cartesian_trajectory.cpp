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
    //ROS_INFO("Group %d name: %s with base link %s tip link: %s", i, it->first.c_str(), it->second.base_link_.c_str(), it->second.tip_link_.c_str());
    //ik_client_map_[it->first] = nh_.serviceClient(/kinematics_msgs::GetPositionIK, )
    std::string ik_service_name = collision_models_interface_->getKinematicModel()->getRobotName() + "_" + it->first + "_kinematics/";
    std::string ik_collision_aware_name = ik_service_name + "get_constraint_aware_ik";
    std::string ik_none_collision_aware_name = ik_service_name + "get_ik";
    ROS_INFO_STREAM(ik_service_name);
    ROS_INFO_STREAM(ik_collision_aware_name);
    ROS_INFO_STREAM(ik_none_collision_aware_name);

    while (!ros::service::waitForService(ik_collision_aware_name, ros::Duration(1.0))) { }
    while (!ros::service::waitForService(ik_none_collision_aware_name, ros::Duration(1.0))) { }
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
      genSimpleIKTrajectory(request, respond);
      break;
    case HgCartesianTrajectory::Request::LINE:
    case HgCartesianTrajectory::Request::CIRCLE:
    case HgCartesianTrajectory::Request::PATH:
      ROS_WARN("not yet implement cartesian tarjectory");
      respond.success = 0;
      break;
  }
  return respond.success;
}

void CartesianTrajectoryPlanner::genSimpleIKTrajectory(HgCartesianTrajectory::Request &request,
                                                              HgCartesianTrajectory::Response &respond)
{
  int trajectory_length = request.poses.size();
  int i, j;

  //IK takes in Cartesian poses stamped with the frame they belong to
  geometry_msgs::PoseStamped stamped_pose;
  stamped_pose.header = request.header;
  stamped_pose.header.stamp = ros::Time::now();
  bool success;
  std::vector<double *> joint_trajectory;

  //get the current joint angles (to find ik solutions close to)
  //double last_angles[7];
  //get_current_joint_angles(last_angles);
  respond.success = 1;
}

