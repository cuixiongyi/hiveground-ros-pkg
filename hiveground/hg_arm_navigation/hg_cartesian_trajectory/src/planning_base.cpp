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
#ifndef PLANNING_BASE_CPP_
#define PLANNING_BASE_CPP_

#include <hg_cartesian_trajectory/planning_base.h>

using namespace hg_cartesian_trajectory;

PlanningBase::PlanningBase()
  : nh_(), nh_private_("~")
{
  collision_models_interface_ =
        boost::shared_ptr<planning_environment::CollisionModelsInterface>(new planning_environment::CollisionModelsInterface("robot_description"));
}

PlanningBase::~PlanningBase()
{

}

void PlanningBase::run()
{
  if (!initialize(nh_private_.getNamespace()))
    return;
  if (!collision_models_interface_->loadedModels())
  {
    ROS_ERROR("Collision models not loaded.");
    return;
  }
}

bool PlanningBase::initialize(const std::string& param_server_prefix)
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

    std::string ik_service_name = collision_models_interface_->getKinematicModel()->getRobotName() + "_" + it->first
        + "_kinematics/";
    std::string ik_info_name = ik_service_name + "get_ik_solver_info";
    std::string ik_collision_aware_name = ik_service_name + "get_constraint_aware_ik";
    std::string ik_none_collision_aware_name = ik_service_name + "get_ik";
    ROS_INFO_STREAM("Tip link: " << it->second.tip_link_);
    ROS_DEBUG_STREAM(ik_service_name);
    ROS_DEBUG_STREAM(ik_collision_aware_name);
    ROS_DEBUG_STREAM(ik_none_collision_aware_name);

    while (!ros::service::waitForService(ik_info_name, ros::Duration(1.0))) { }
    while (!ros::service::waitForService(ik_collision_aware_name, ros::Duration(1.0))) { }
    while (!ros::service::waitForService(ik_none_collision_aware_name, ros::Duration(1.0))) { }
    //ROS_INFO_STREAM(it->second.subgroups_.size());
    //ROS_INFO_STREAM(it->second.joints_.size());
    //joints_map_[it->first] = it->second.joints_;
    //ROS_INFO_STREAM("joint count: " << it->second.joints_.size());
    //for(int i = 0; i < it->second.joints_.size(); i++)
    //{
      //ROS_INFO_STREAM(it->second.joints_[i]);
    //}
    tip_link_map_[it->first] = it->second.tip_link_;
    ik_info_client_map_[it->first] = nh_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>(ik_info_name, true);
    ik_client_map_[it->first] = nh_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>(ik_collision_aware_name, true);
    ik_none_collision_client_map_[it->first] = nh_.serviceClient<kinematics_msgs::GetPositionIK>(ik_none_collision_aware_name, true);
  }

  return true;
}

bool PlanningBase::getGroupNamesFromParamServer(const std::string &param_server_prefix,
                                                      std::vector<std::string> &group_names)
{
  XmlRpc::XmlRpcValue group_list;
  if (!nh_.getParam(param_server_prefix + "/groups", group_list))
  {
    ROS_ERROR("Could not find parameter %s on param server", (param_server_prefix+"/groups").c_str());
    return false;
  }
  if (group_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Group list should be of XmlRpc Array type");
    return false;
  }
  for (int32_t i = 0; i < group_list.size(); ++i)
  {
    if (group_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("Group names should be strings");
      return false;
    }
    group_names.push_back(static_cast<std::string>(group_list[i]));
    ROS_INFO("Adding group: %s", group_names.back().c_str());
  }
  return true;
}

#endif /* PLANNING_BASE_CPP_ */
