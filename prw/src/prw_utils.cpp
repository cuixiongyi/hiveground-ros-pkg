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

#include <prw/prw_utils.h>

using namespace hg;
using namespace std;
using namespace planning_environment;
using namespace planning_models;
using namespace kinematics_msgs;
using namespace arm_navigation_msgs;
using namespace visualization_msgs;
using namespace interactive_markers;
using namespace control_msgs;
using namespace geometry_msgs;

static const string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
static const string PLANNER_SERVICE_NAME = "/ompl_planning/plan_kinematic_path";
static const string TRAJECTORY_FILTER_SERVICE_NAME = "/trajectory_filter_server/filter_trajectory_with_constraints";

WorkspaceEditor::WorkspaceEditor()
{

}

WorkspaceEditor::WorkspaceEditor(const WorkspaceEditorParameters& parameter)

{
  ROS_INFO_STREAM(__FUNCTION__ << " Initializing...");
  robot_state_ = NULL;
  collision_aware_ = true;
  constrain_rp_ = false;
  is_joint_control_active_ = false;
  is_ik_control_active_ = true;
  last_collision_objects_id_ = 0;

  string robot_description_name = nh_.resolveName("robot_description", true);
  cm_ = new CollisionModels("robot_description");
  marker_publisher_ = nh_.advertise<Marker>("prw_workspace_editor", 128);
  marker_array_publisher_ = nh_.advertise<MarkerArray>("prw_workspace_editor_array", 128);
  process_ik_controller_feedback_ptr_ = boost::bind(&WorkspaceEditor::processIKControllerFeedback, this, _1);
  process_menu_feedback_ptr_ = boost::bind(&WorkspaceEditor::processMenuFeedback, this, _1);
  process_marker_feedback_ptr_ = boost::bind(&WorkspaceEditor::processMarkerFeedback, this, _1);

  //Subscribers
  joint_state_subscriber_ = nh_.subscribe("joint_states", 25, &WorkspaceEditor::jointStateCallback, this);

  //Clients
  while (!ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME, ros::Duration(1.0)))
  {
  }
  get_planning_scene_client_ = nh_.serviceClient<arm_navigation_msgs::GetPlanningScene>(SET_PLANNING_SCENE_DIFF_NAME);

  while (!ros::service::waitForService(PLANNER_SERVICE_NAME, ros::Duration(1.0)))
  {
  }
  planner_client_ = nh_.serviceClient<GetMotionPlan>(PLANNER_SERVICE_NAME, true);

  while (!ros::service::waitForService(TRAJECTORY_FILTER_SERVICE_NAME, ros::Duration(1.0)))
  {
  }
  trajectory_filter_client_ = nh_.serviceClient<FilterJointTrajectoryWithConstraints>(TRAJECTORY_FILTER_SERVICE_NAME,
                                                                                      true);

  const map<string, KinematicModel::GroupConfig>& group_config_map =
      cm_->getKinematicModel()->getJointModelGroupConfigMap();

  for (map<string, KinematicModel::GroupConfig>::const_iterator it = group_config_map.begin();
      it != group_config_map.end(); it++)
  {
    group_map_[it->first].name_ = it->first;
    group_map_[it->first].ik_link_name_ = it->second.tip_link_;

    ROS_INFO_STREAM(group_map_[it->first].name_);
    ROS_INFO_STREAM(group_map_[it->first].ik_link_name_);

    string ik_service_name = cm_->getKinematicModel()->getRobotName() + "_" + it->first + "_kinematics/";
    string coll_aware_name = ik_service_name + "get_constraint_aware_ik";
    string non_coll_aware_name = ik_service_name + "get_ik";
    string arm_controller_name = cm_->getKinematicModel()->getRobotName() + "/follow_joint_trajectory";

    while (!ros::service::waitForService(coll_aware_name, ros::Duration(1.0)))
    {
    }
    while (!ros::service::waitForService(non_coll_aware_name, ros::Duration(1.0)))
    {
    }

    group_map_[it->first].ik_collision_aware_client_ = nh_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>(
        coll_aware_name, true);

    group_map_[it->first].ik_non_collision_aware_client_ = nh_.serviceClient<kinematics_msgs::GetPositionIK>(
        non_coll_aware_name, true);

    group_map_[it->first].arm_controller_.reset(
        new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(arm_controller_name, true));
    while(ros::ok() && !group_map_[it->first].arm_controller_->waitForServer(ros::Duration(1.0))) { }


  }

  robot_state_ = new KinematicState(cm_->getKinematicModel());
  robot_state_->setKinematicStateToDefault();
  sendPlanningScene();

  //send planning scene

  //Publishers

  //Marker & Interactive Marker

  interactive_marker_server_.reset(new InteractiveMarkerServer("prw_visualizer_controls", "", false));

  unsigned int cmd = 0;
  for (PlanningGroupDataMap::iterator it = group_map_.begin(); it != group_map_.end(); it++)
  {
    // These positions will be reset by main()
    makeIKControllerMarker(tf::Transform(tf::Quaternion(0.0f, 0.0f, 0.0f, 1.0f), tf::Vector3(0.0f, 0.0f, 0.0f)),
                           it->first, it->first, true, 0.5f);
    cmd++;
  }

  makeTopLevelMenu();

  interactive_marker_server_->applyChanges();

  //waiting for joint information
  while (robot_state_joint_values_.empty() && ros::ok())
  {
    ROS_INFO("waiting for joint information");
    ros::Duration(1.0).sleep();
  }

  selectPlanningGroup(0);
  solveIKForEndEffectorPose(*getPlanningGroup(0));


  Pose pose;
  pose.position.x = 2.0f;
  pose.position.z = 1.0f;
  pose.position.y = 0.0f;
  pose.orientation.x = 0.0f;
  pose.orientation.y = 0.0f;
  pose.orientation.z = 0.0f;
  pose.orientation.w = 1.0f;
  std_msgs::ColorRGBA color;
  color.a = 0.5;
  color.r = 1.0;
  color.g = 0.0;
  color.b = 0.0;

  //createCollisionPole(0, polePose);
  createCollisionObject(pose, Pole, color, 0.05, 1.0, 0, true);
  pose.position.y = 1.0;
  color.g = 1.0;
  createCollisionObject(pose, Box, color, 0.1, 0.1, 0.1, true);
  pose.position.y = -1.0;
  color.g = 0.0;
  color.b = 1.0;
  createCollisionObject(pose, Sphere, color, 0.1, 0.0, 0.0, true);

  sendPlanningScene();


  ROS_INFO_STREAM(__FUNCTION__ << " Initialized");
}

void WorkspaceEditor::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state)
{
  if (robot_state_ == NULL)
    return;

  std::map<std::string, double> joint_state_map;
  std::map<std::string, double> joint_velocity_map;

  //message already been validated in kmsm
  if (joint_state->velocity.size() == joint_state->position.size())
  {
    for (unsigned int i = 0; i < joint_state->position.size(); ++i)
    {
      joint_state_map[joint_state->name[i]] = joint_state->position[i];
      joint_velocity_map[joint_state->name[i]] = joint_state->velocity[i];
    }
  }
  else
  {
    for (unsigned int i = 0; i < joint_state->position.size(); ++i)
    {
      joint_state_map[joint_state->name[i]] = joint_state->position[i];
      joint_velocity_map[joint_state->name[i]] = 0.0;
    }
  }
  //getting base transform
  lockScene();
  std::vector<planning_models::KinematicState::JointState*>& joint_state_vector = robot_state_->getJointStateVector();
  for (std::vector<planning_models::KinematicState::JointState*>::iterator it = joint_state_vector.begin();
      it != joint_state_vector.end(); it++)
  {
    bool tfSets = false;
    //see if we need to update any transforms
    std::string parent_frame_id = (*it)->getParentFrameId();
    std::string child_frame_id = (*it)->getChildFrameId();
    if (!parent_frame_id.empty() && !child_frame_id.empty())
    {
      std::string err;
      ros::Time tm;
      tf::StampedTransform transf;
      bool ok = false;
      if (transform_listener_.getLatestCommonTime(parent_frame_id, child_frame_id, tm, &err) == tf::NO_ERROR)
      {
        ok = true;
        try
        {
          transform_listener_.lookupTransform(parent_frame_id, child_frame_id, tm, transf);
        }
        catch (tf::TransformException& ex)
        {
          ROS_ERROR(
              "Unable to lookup transform from %s to %s.  Exception: %s", parent_frame_id.c_str(), child_frame_id.c_str(), ex.what());
          ok = false;
        }
      }
      else
      {
        ROS_DEBUG(
            "Unable to lookup transform from %s to %s: no common time.", parent_frame_id.c_str(), child_frame_id.c_str());
        ok = false;
      }
      if (ok)
      {
        tfSets = (*it)->setJointStateValues(transf);
      }
    }
    (*it)->setJointStateValues(joint_state_map);
  }
  robot_state_->updateKinematicLinks();
  robot_state_->getKinematicStateValues(robot_state_joint_values_);

  //TODO
  //Add collision check here


  unlockScene();
}

void WorkspaceEditor::sendPlanningScene()
{
  ROS_INFO("Sending Planning Scene....");
  lockScene();

  arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;
  arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;

  vector<string> removals;
  // Handle additions and removals of planning scene objects.
  for (CollisionObjectMap::const_iterator it = collision_objects_.begin();
      it != collision_objects_.end(); it++)
  {
    string name = it->first;
    arm_navigation_msgs::CollisionObject object = it->second;

    // Add or remove objects.
    if (object.operation.operation != arm_navigation_msgs::CollisionObjectOperation::REMOVE)
    {
      ROS_INFO("Adding Collision Object %s", object.id.c_str());
      planning_scene_req.planning_scene_diff.collision_objects.push_back(object);
    }
    else
    {
      removals.push_back(it->first);
    }
  }

  // Delete collision poles from the map which were removed.
  for (size_t i = 0; i < removals.size(); i++)
  {
    collision_objects_.erase(removals[i]);
  }


  //get current robot state
  convertKinematicStateToRobotState(*robot_state_, ros::Time::now(), cm_->getWorldFrameId(),
                                    planning_scene_req.planning_scene_diff.robot_state);

  KinematicState* startState = NULL;
  KinematicState* endState = NULL;
  map<string, double> startStateValues;
  map<string, double> endStateValues;

  //copy current group state
  if (current_group_name_ != "")
  {
    startState = group_map_[current_group_name_].start_state_;
    endState = group_map_[current_group_name_].end_state_;

    if (startState != NULL)
    {
      startState->getKinematicStateValues(startStateValues);
    }
    if (endState != NULL)
    {
      endState->getKinematicStateValues(endStateValues);
    }
  }

  //delete stored state
  deleteKinematicStates();

  if (robot_state_ != NULL)
  {
    ROS_INFO("Reverting planning scene to default.");
    cm_->revertPlanningScene(robot_state_);
    robot_state_ = NULL;
  }

  //request planning scene
  if (!get_planning_scene_client_.call(planning_scene_req, planning_scene_res))
  {
    ROS_WARN("Can't get planning scene");
    unlockScene();
    return;
  }

  //get current robot state
  robot_state_ = cm_->setPlanningScene(planning_scene_res.planning_scene);

  if (robot_state_ == NULL)
  {
    ROS_ERROR("Something wrong with planning scene");
    unlockScene();
    return;
  }

  //update current group information
  if (current_group_name_ != "")
  {
    ROS_INFO("Resetting state...");
    group_map_[current_group_name_].setState(StartPosition, new KinematicState(robot_state_->getKinematicModel()));
    group_map_[current_group_name_].setState(EndPosition, new KinematicState(robot_state_->getKinematicModel()));

    startState = group_map_[current_group_name_].start_state_;
    endState = group_map_[current_group_name_].end_state_;

    if (startState != NULL)
    {
      //update state with latest joint value
      startState->setKinematicState(robot_state_joint_values_);
    }

    if (endState != NULL)
    {
      //use previously saved joint value
      endState->setKinematicState(endStateValues);
    }
  }

  unlockScene();
  ROS_INFO("Planning scene sent.");
}

void WorkspaceEditor::refreshEnvironment()
{
  PlanningGroupData& gc = group_map_[current_group_name_];
  sendPlanningScene();
  moveEndEffectorMarkers(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);

  tf::Transform cur = toBulletTransform(last_ee_poses_[current_group_name_]);
  setNewEndEffectorPosition(gc, cur, collision_aware_);
}

void WorkspaceEditor::deleteKinematicStates()
{
  for (PlanningGroupDataMap::iterator it = group_map_.begin(); it != group_map_.end(); it++)
  {
    it->second.reset();
  }
}

void WorkspaceEditor::selectPlanningGroup(unsigned int entry)
{
  ROS_INFO("Selecting planning group %u", entry);
  lockScene();
  vector<string> names;
  for (PlanningGroupDataMap::iterator it = group_map_.begin(); it != group_map_.end(); it++)
  {
    names.push_back(it->first);
  }
  string old_group_name = current_group_name_;

  current_group_name_ = names[entry];

  //use old state is exist
  if (group_map_[current_group_name_].end_state_ != NULL)
  {
    group_map_[current_group_name_].setState(EndPosition,
                                             new KinematicState(*group_map_[current_group_name_].end_state_));
  }
  else
  {
    group_map_[current_group_name_].setState(EndPosition, new KinematicState(*robot_state_));
  }

  group_map_[current_group_name_].setState(StartPosition, new KinematicState(*robot_state_));

  moveEndEffectorMarkers(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);

  // If we previously selected a planning group, deselect its marker.
  if (is_ik_control_active_ && old_group_name != "" && selectableMarkerExists(old_group_name + "_selectable"))
  {
    PlanningGroupData& gc = group_map_[old_group_name];
    tf::Transform cur = robot_state_->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform();
    deselectMarker(selectable_markers_[old_group_name + "_selectable"], cur);

    if (is_joint_control_active_)
    {
      //deleteJointMarkers(group_map_[old_group_name]);
    }
  }
  else if (is_joint_control_active_)
  {
    //deleteJointMarkers(group_map_[old_group_name]);
  }

  // Select the new planning group's marker.
  if (is_ik_control_active_ && selectableMarkerExists(current_group_name_ + "_selectable"))
  {
    PlanningGroupData& gc = group_map_[current_group_name_];
    tf::Transform cur = gc.end_state_->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform();
    selectMarker(selectable_markers_[current_group_name_ + "_selectable"], cur);
    //createSelectableJointMarkers(gc);
  }
  else if (is_joint_control_active_)
  {
    //GroupCollection& gc = group_map_[current_group_name_];
    //createSelectableJointMarkers(gc);
  }

  interactive_marker_server_->applyChanges();

  unlockScene();
  ROS_INFO("Planning group selected.");
}

void WorkspaceEditor::moveEndEffectorMarkers(double vx, double vy, double vz, double vr, double vp, double vw,
                                             bool coll_aware)
{
  lockScene();
  PlanningGroupData& gc = group_map_[current_group_name_];
  tf::Transform cur = gc.end_state_->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform();
  double mult = 0.1;

  tf::Vector3& curOrigin = cur.getOrigin();
  tf::Vector3 newOrigin(curOrigin.x() + (vx * mult), curOrigin.y() + (vy * mult), curOrigin.z() + (vz * mult));
  cur.setOrigin(newOrigin);

  tfScalar roll, pitch, yaw;

  cur.getBasis().getRPY(roll, pitch, yaw);
  roll += vr * mult;
  pitch += vp * mult;
  yaw += vw * mult;

  if (roll > 2 * M_PI)
  {
    roll -= 2 * M_PI;
  }
  else if (roll < -2 * M_PI)
  {
    roll += 2 * M_PI;
  }

  if (pitch > 2 * M_PI)
  {
    pitch -= 2 * M_PI;
  }
  else if (pitch < -2 * M_PI)
  {
    pitch += 2 * M_PI;
  }

  cur.getBasis().setRPY(roll, pitch, yaw);

  setNewEndEffectorPosition(gc, cur, coll_aware);

  unlockScene();
}

void WorkspaceEditor::setNewEndEffectorPosition(PlanningGroupData& gc, tf::Transform& cur, bool coll_aware)
{
  if (!gc.end_state_->updateKinematicStateWithLinkAt(gc.ik_link_name_, cur))
  {
    ROS_INFO_STREAM(__FUNCTION__ << " Problem");
  }

  if (solveIKForEndEffectorPose(gc, coll_aware, constrain_rp_))
  {
    gc.good_ik_solution_ = true;
    gc.last_good_state_ = cur;
  }
  else
  {
    gc.good_ik_solution_ = false;
  }
}

bool WorkspaceEditor::solveIKForEndEffectorPose(PlanningGroupData& gc, bool coll_aware, bool constrain_pitch_and_roll,
                                                double change_redundancy)
{
  kinematics_msgs::PositionIKRequest ik_request;

  ik_request.ik_link_name = gc.ik_link_name_;
  ik_request.pose_stamped.header.frame_id = cm_->getWorldFrameId();
  ik_request.pose_stamped.header.stamp = ros::Time::now();
  tf::poseTFToMsg(gc.end_state_->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform(),
                  ik_request.pose_stamped.pose);
  convertKinematicStateToRobotState(*gc.end_state_, ros::Time::now(), cm_->getWorldFrameId(), ik_request.robot_state);
  ik_request.ik_seed_state = ik_request.robot_state;

  map<string, double> joint_values;
  vector<string> joint_names;

  if (coll_aware)
  {
    kinematics_msgs::GetConstraintAwarePositionIK::Request ik_req;
    kinematics_msgs::GetConstraintAwarePositionIK::Response ik_res;
    if (constrain_pitch_and_roll)
    {
      arm_navigation_msgs::Constraints goal_constraints;
      goal_constraints.orientation_constraints.resize(1);
      arm_navigation_msgs::Constraints path_constraints;
      path_constraints.orientation_constraints.resize(1);
      determinePitchRollConstraintsGivenState(gc, *gc.start_state_, goal_constraints.orientation_constraints[0],
                                              path_constraints.orientation_constraints[0]);
      arm_navigation_msgs::ArmNavigationErrorCodes err;
      if (!cm_->isKinematicStateValid(*gc.end_state_, std::vector<std::string>(), err, goal_constraints,
                                      path_constraints))
      {
        ROS_INFO_STREAM("Violates rp constraints");
        return false;
      }
      ik_req.constraints = goal_constraints;
    } //constrain_pitch_and_roll

    ik_req.ik_request = ik_request;
    ik_req.timeout = ros::Duration(0.2);
    if (!gc.ik_collision_aware_client_.call(ik_req, ik_res))
    {
      ROS_INFO("Problem with ik service call");
      return false;
    }
    if (ik_res.error_code.val != ik_res.error_code.SUCCESS)
    {
      ROS_DEBUG_STREAM("Call yields bad error code " << ik_res.error_code.val);
      return false;
    }
    joint_names = ik_res.solution.joint_state.name;
    gc.joint_names_.clear();
    gc.joint_names_ = joint_names;
    for (unsigned int i = 0; i < ik_res.solution.joint_state.name.size(); i++)
    {
      joint_values[ik_res.solution.joint_state.name[i]] = ik_res.solution.joint_state.position[i];
    }

  }
  else
  {
    kinematics_msgs::GetPositionIK::Request ik_req;
    kinematics_msgs::GetPositionIK::Response ik_res;
    ik_req.ik_request = ik_request;
    ik_req.timeout = ros::Duration(0.2);
    if (!gc.ik_non_collision_aware_client_.call(ik_req, ik_res))
    {
      ROS_INFO("Problem with ik service call");
      return false;
    }
    if (ik_res.error_code.val != ik_res.error_code.SUCCESS)
    {
      ROS_DEBUG_STREAM("Call yields bad error code " << ik_res.error_code.val);
      return false;
    }
    for (unsigned int i = 0; i < ik_res.solution.joint_state.name.size(); i++)
    {
      joint_values[ik_res.solution.joint_state.name[i]] = ik_res.solution.joint_state.position[i];
    }

  }

  lockScene();
  gc.end_state_->setKinematicState(joint_values);
  unlockScene();

  //createSelectableJointMarkers(gc);

  if (coll_aware)
  {
    Constraints emp_con;
    ArmNavigationErrorCodes error_code;

    if (!cm_->isKinematicStateValid(*gc.end_state_, joint_names, error_code, emp_con, emp_con, true))
    {
      ROS_INFO_STREAM("Problem with response");
    }
  }

  return true;
}

void WorkspaceEditor::determinePitchRollConstraintsGivenState(
    const PlanningGroupData& gc, const planning_models::KinematicState& state,
    arm_navigation_msgs::OrientationConstraint& goal_constraint,
    arm_navigation_msgs::OrientationConstraint& path_constraint) const
{
  tf::Transform cur = state.getLinkState(gc.ik_link_name_)->getGlobalLinkTransform();
  //tfScalar roll, pitch, yaw;
  //cur.getBasis().getRPY(roll,pitch,yaw);
  goal_constraint.header.frame_id = cm_->getWorldFrameId();
  goal_constraint.header.stamp = ros::Time::now();
  goal_constraint.link_name = gc.ik_link_name_;
  tf::quaternionTFToMsg(cur.getRotation(), goal_constraint.orientation);
  goal_constraint.absolute_roll_tolerance = 0.04;
  goal_constraint.absolute_pitch_tolerance = 0.04;
  goal_constraint.absolute_yaw_tolerance = M_PI;
  path_constraint.header.frame_id = cm_->getWorldFrameId();
  path_constraint.header.stamp = ros::Time::now();
  path_constraint.link_name = gc.ik_link_name_;
  tf::quaternionTFToMsg(cur.getRotation(), path_constraint.orientation);
  path_constraint.type = path_constraint.HEADER_FRAME;
  path_constraint.absolute_roll_tolerance = 0.1;
  path_constraint.absolute_pitch_tolerance = 0.1;
  path_constraint.absolute_yaw_tolerance = M_PI;
}

bool WorkspaceEditor::planToEndEffectorState(PlanningGroupData& gc, bool show, bool play)
{
  MotionPlanRequest motion_plan_request;
  motion_plan_request.group_name = gc.name_;
  motion_plan_request.num_planning_attempts = 1;
  motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  if (!constrain_rp_)
  {
    const KinematicState::JointStateGroup* jsg = gc.end_state_->getJointStateGroup(gc.name_);
    motion_plan_request.goal_constraints.joint_constraints.resize(jsg->getJointNames().size());
    vector<double> joint_values;
    jsg->getKinematicStateValues(joint_values);
    for (unsigned int i = 0; i < jsg->getJointNames().size(); i++)
    {
      motion_plan_request.goal_constraints.joint_constraints[i].joint_name = jsg->getJointNames()[i];
      motion_plan_request.goal_constraints.joint_constraints[i].position = joint_values[i];
      motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.01;
      motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.01;
    }
  }
  else
  {
    motion_plan_request.group_name += "_cartesian";
    motion_plan_request.goal_constraints.position_constraints.resize(1);
    motion_plan_request.goal_constraints.orientation_constraints.resize(1);
    geometry_msgs::PoseStamped end_effector_wrist_pose;
    tf::poseTFToMsg(gc.end_state_->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform(),
                    end_effector_wrist_pose.pose);
    end_effector_wrist_pose.header.frame_id = cm_->getWorldFrameId();
    arm_navigation_msgs::poseStampedToPositionOrientationConstraints(
        end_effector_wrist_pose, gc.ik_link_name_, motion_plan_request.goal_constraints.position_constraints[0],
        motion_plan_request.goal_constraints.orientation_constraints[0]);
    motion_plan_request.path_constraints.orientation_constraints.resize(1);
    determinePitchRollConstraintsGivenState(gc, *robot_state_,
                                            motion_plan_request.goal_constraints.orientation_constraints[0],
                                            motion_plan_request.path_constraints.orientation_constraints[0]);
  }
  convertKinematicStateToRobotState(*robot_state_, ros::Time::now(), cm_->getWorldFrameId(),
                                    motion_plan_request.start_state);
  GetMotionPlan::Request plan_req;
  plan_req.motion_plan_request = motion_plan_request;
  GetMotionPlan::Response plan_res;
  if (!planner_client_.call(plan_req, plan_res))
  {
    ROS_INFO("Something wrong with planner client");
    return false;
  }

  if (gc.trajectory_data_map_.find("planner") != gc.trajectory_data_map_.end())
  {
    TrajectoryData& disp = gc.trajectory_data_map_["planner"];
    if (plan_res.error_code.val != plan_res.error_code.SUCCESS)
    {
      disp.trajectory_error_code_ = plan_res.error_code;
      ROS_INFO_STREAM("Bad planning error code " << plan_res.error_code.val);
      gc.trajectory_data_map_["planner"].reset();
      return false;
    }
    disp.reset();
    disp.joint_trajectory_ = plan_res.trajectory.joint_trajectory;
    disp.has_joint_trajectory_ = true;
    disp.show_joint_trajectory_ = show;
    disp.play_joint_trajectory_ = play;
    disp.state_ = new KinematicState(*robot_state_);

    vector<ArmNavigationErrorCodes> trajectory_error_codes;

    cm_->isJointTrajectoryValid(*disp.state_, disp.joint_trajectory_, last_motion_plan_request_.goal_constraints,
                                last_motion_plan_request_.path_constraints, disp.trajectory_error_code_,
                                trajectory_error_codes, false);

    if (disp.trajectory_error_code_.val != disp.trajectory_error_code_.SUCCESS)
    {
      disp.trajectory_bad_point_ = trajectory_error_codes.size() - 1;
    }
    else
    {
      disp.trajectory_bad_point_ = -1;
    }

    last_motion_plan_request_ = motion_plan_request;
    return true;
  }
  else
  {
    return false;
  }
}

bool WorkspaceEditor::filterPlannerTrajectory(PlanningGroupData& gc, bool show, bool play)
{
  FilterJointTrajectoryWithConstraints::Request filter_req;
  FilterJointTrajectoryWithConstraints::Response filter_res;

  convertKinematicStateToRobotState(*robot_state_, ros::Time::now(), cm_->getWorldFrameId(),
                                    filter_req.start_state);
  TrajectoryData& planner_disp = gc.trajectory_data_map_["planner"];
  filter_req.trajectory = planner_disp.joint_trajectory_;
  filter_req.group_name = gc.name_;

  filter_req.goal_constraints = last_motion_plan_request_.goal_constraints;
  filter_req.path_constraints = last_motion_plan_request_.path_constraints;
  filter_req.allowed_time = ros::Duration(2.0);

  if (!trajectory_filter_client_.call(filter_req, filter_res))
  {
    ROS_INFO("Problem with trajectory filter");
    gc.trajectory_data_map_["filter"].reset();
    return false;
  }
  TrajectoryData& filter_disp = gc.trajectory_data_map_["filter"];
  if (filter_res.error_code.val != filter_res.error_code.SUCCESS)
  {
    filter_disp.trajectory_error_code_ = filter_res.error_code;
    ROS_INFO_STREAM("Bad trajectory_filter error code " << filter_res.error_code.val);
    gc.trajectory_data_map_["filter"].reset();
    return false;
  }

  filter_disp.reset();
  filter_disp.joint_trajectory_ = filter_res.trajectory;
  filter_disp.has_joint_trajectory_ = true;
  filter_disp.show_joint_trajectory_ = show;
  filter_disp.play_joint_trajectory_ = play;
  filter_disp.state_ = new KinematicState(*robot_state_);

  vector<ArmNavigationErrorCodes> trajectory_error_codes;

  cm_->isJointTrajectoryValid(*filter_disp.state_, filter_disp.joint_trajectory_, last_motion_plan_request_.goal_constraints,
                              last_motion_plan_request_.path_constraints, filter_disp.trajectory_error_code_,
                              trajectory_error_codes, false);

  if (filter_disp.trajectory_error_code_.val != filter_disp.trajectory_error_code_.SUCCESS)
  {
    filter_disp.trajectory_bad_point_ = trajectory_error_codes.size() - 1;
  }
  else
  {
    filter_disp.trajectory_bad_point_ = -1;
  }
  return true;
}

void WorkspaceEditor::moveThroughTrajectory(PlanningGroupData& gc, const string& source_name, int step)
{
  lockScene();
  TrajectoryData& disp = gc.trajectory_data_map_[source_name];
  unsigned int tsize = disp.joint_trajectory_.points.size();
  if(tsize == 0 || disp.state_ == NULL)
  {
    unlockScene();
    return;
  }
  if((int)disp.current_trajectory_point_ + step < 0)
  {
    disp.current_trajectory_point_ = 0;
  }
  else
  {
    disp.current_trajectory_point_ = ((int)disp.current_trajectory_point_) + step;
  }
  if(disp.current_trajectory_point_ >= tsize - 1)
  {
    disp.current_trajectory_point_ = tsize - 1;
    disp.play_joint_trajectory_ = false;
    disp.show_joint_trajectory_ = false;
  }
  map<string, double> joint_values;
  for(unsigned int i = 0; i < disp.joint_trajectory_.joint_names.size(); i++)
  {
    joint_values[disp.joint_trajectory_.joint_names[i]]
        = disp.joint_trajectory_.points[disp.current_trajectory_point_].positions[i];
  }
  disp.state_->setKinematicState(joint_values);
  unlockScene();
}

void WorkspaceEditor::controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                                             const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  ROS_INFO("done!");
  //refreshEnvironment();

}

void WorkspaceEditor::sendMarkers()
{
  lockScene();
  MarkerArray arr;

  std_msgs::ColorRGBA stat_color_;
  stat_color_.a = 0.6;
  stat_color_.r = 0.1;
  stat_color_.g = 0.8;
  stat_color_.b = 0.3;

  std_msgs::ColorRGBA attached_color_;
  attached_color_.a = 1.0;
  attached_color_.r = 0.6;
  attached_color_.g = 0.4;
  attached_color_.b = 0.3;

  cm_->getAllCollisionSpaceObjectMarkers(*robot_state_, arr, "", stat_color_, attached_color_, ros::Duration(0.1));

  if (!current_group_name_.empty())
  {
    std_msgs::ColorRGBA bad_color;
    bad_color.a = 0.5;
    bad_color.r = 0.9;
    bad_color.g = 0.0;
    bad_color.b = 0.0;

    std_msgs::ColorRGBA collision_color;
    bad_color.a = 1.0;
    bad_color.r = 1.0;
    bad_color.g = 0.5;
    bad_color.b = 0.0;

    PlanningGroupData& gc = group_map_[current_group_name_];
    const KinematicModel* kinematic_model = cm_->getKinematicModel();


    if(is_ik_control_active_)
    {
      if (gc.end_state_ != NULL)
      {
        cm_->getGroupAndUpdatedJointMarkersGivenState(*gc.end_state_, arr, current_group_name_, gc.end_color_,
                                                     gc.start_color_, ros::Duration(0.1));
      }
      else
      {
        ROS_ERROR("End state invalid!");
      }
    }

    if (!gc.good_ik_solution_ && gc.end_state_ != NULL)
    {
      vector<string> lnames =
                    kinematic_model->getChildLinkModelNames(kinematic_model->getLinkModel(gc.ik_link_name_));
      cm_->getRobotMarkersGivenState(*gc.end_state_, arr, bad_color, current_group_name_, ros::Duration(0.2), &lnames);
      cm_->getAllCollisionPointMarkers(*robot_state_, arr, bad_color, ros::Duration(.2));
    }


    for (TrajectoryDataMap::iterator it = gc.trajectory_data_map_.begin(); it != gc.trajectory_data_map_.end(); it++)
    {

      if (it->second.play_joint_trajectory_)
      {
        moveThroughTrajectory(gc, it->first, 5);
      }


      if (it->second.show_joint_trajectory_)
      {
        const vector<const KinematicModel::LinkModel*>& updated_links =
            kinematic_model->getModelGroup(gc.name_)->getUpdatedLinkModels();
        vector<string> lnames;
        lnames.resize(updated_links.size());
        for (unsigned int i = 0; i < updated_links.size(); i++)
        {
          lnames[i] = updated_links[i]->getName();
        }

        cm_->getAttachedCollisionObjectMarkers(*(it->second.state_), arr, it->first + "_trajectory", it->second.color_,
                                               ros::Duration(0.1));

        cm_->getRobotMarkersGivenState(*(it->second.state_), arr, it->second.color_, it->first + "_trajectory",
                                       ros::Duration(0.1), &lnames);

      }
    }



  }




  marker_array_publisher_.publish(arr);
  unlockScene();
}

WorkspaceEditor::~WorkspaceEditor()
{

}
