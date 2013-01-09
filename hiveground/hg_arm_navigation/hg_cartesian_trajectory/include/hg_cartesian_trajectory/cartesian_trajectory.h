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

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <planning_environment/models/collision_models_interface.h>
#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/model_utils.h>
#include <planning_environment/monitors/joint_state_monitor.h>
#include <arm_navigation_msgs/DisplayTrajectory.h>
#include <nav_msgs/Path.h>

#include <hg_cartesian_trajectory/planning_base.h>
#include <hg_cartesian_trajectory/HgCartesianTrajectory.h>

#define HG_MAX_SIMPLE_IK_JOINT_VEL 1.0 //radians/sec

namespace hg_cartesian_trajectory
{

typedef boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> > FollowJointTrajectoryClientPtr;
//typedef std::map<std::string, planning_models::KinematicModel::GroupConfig> KinematicModelGroupConfigMap;

class CartesianTrajectoryPlanner : public PlanningBase
{
public:
  CartesianTrajectoryPlanner();
  ~CartesianTrajectoryPlanner();

  void run();

  bool initialize(const std::string& param_server_prefix);


  bool executeCartesianTrajectoryPlanner(HgCartesianTrajectory::Request &request,
                                              HgCartesianTrajectory::Response &respond);

protected:
  bool runIk(const std::string& group_name,
              const arm_navigation_msgs::RobotState& robot_state,
              geometry_msgs::PoseStamped pose,
              std::vector<double>& start_position,
              std::vector<double>& solution);
  void planSimpleIKTrajectory(HgCartesianTrajectory::Request &request,
                                  HgCartesianTrajectory::Response &respond);


protected:
  std::map<std::string, FollowJointTrajectoryClientPtr> action_client_map_;
  ros::ServiceServer service_;
  planning_environment::JointStateMonitor joint_state_monitor;
  ros::Publisher display_trajectory_publisher_;
  ros::Publisher display_trajectory_path_publisher_;
};

}
