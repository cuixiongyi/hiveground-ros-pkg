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

std_msgs::ColorRGBA makeRandomColor(float brightness, float alpha)
{
  std_msgs::ColorRGBA toReturn;
  toReturn.a = alpha;

  toReturn.r = ((float)(random()) / (float)RAND_MAX) * (1.0f - brightness) + brightness;
  toReturn.g = ((float)(random()) / (float)RAND_MAX) * (1.0f - brightness) + brightness;
  toReturn.b = ((float)(random()) / (float)RAND_MAX) * (1.0f - brightness) + brightness;

  toReturn.r = min(toReturn.r, 1.0f);
  toReturn.g = min(toReturn.g, 1.0f);
  toReturn.b = min(toReturn.b, 1.0f);

  return toReturn;
}

MotionPlanRequestData::MotionPlanRequestData(const unsigned int& id,
                                             const string& source,
                                             const MotionPlanRequest& request,
                                             const KinematicState* robot_state,
                                             const std::string& end_effector_name)
{
  // Note: these must be registered as StateRegistry entries after this request has been created.
  start_state_ = new KinematicState(*robot_state);
  goal_state_ = new KinematicState(*robot_state);

  id_ = id;
  source_ = source;

  end_effector_link_ = end_effector_name;
  motion_plan_request_ = request;

  start_color_ = makeRandomColor(0.3f, 0.6f);
  goal_color_ = makeRandomColor(0.3f, 0.6f);

  is_start_editable_ = true;
  is_goal_editable_ = true;

  setHasGoodIKSolution(true, StartPosition);
  setHasGoodIKSolution(true, GoalPosition);

  if(request.path_constraints.orientation_constraints.size() > 0) {
    const OrientationConstraint& oc = request.path_constraints.orientation_constraints[0];
    setPathConstraints(true);
    if(oc.absolute_roll_tolerance < 3.0) {
      setConstrainRoll(true);
      setRollTolerance(oc.absolute_roll_tolerance);
    } else {
      setConstrainRoll(false);
      setRollTolerance(0.05);
    }
    if(oc.absolute_pitch_tolerance < 3.0) {
      setConstrainPitch(true);
      setPitchTolerance(oc.absolute_pitch_tolerance);
    } else {
      setConstrainPitch(false);
      setPitchTolerance(0.05);
    }
    if(oc.absolute_yaw_tolerance < 3.0) {
      setConstrainYaw(true);
      setYawTolerance(oc.absolute_yaw_tolerance);
    } else {
      setConstrainYaw(false);
      setYawTolerance(0.05);
    }
  } else {
    setPathConstraints(false);
    setConstrainRoll(false);
    setConstrainPitch(false);
    setConstrainYaw(false);
    setRollTolerance(.05);
    setPitchTolerance(.05);
    setYawTolerance(.05);
  }
  show();
  showCollisions();

  should_refresh_colors_ = false;
  has_refreshed_colors_ = true;
  refresh_timer_ = ros::Duration(0.0);
  are_joint_controls_visible_ = false;

  render_type_ = CollisionMesh;
}


WorkspaceEditor::WorkspaceEditor(hg::WorkspaceEditorParameters* parameters)
{
  ROS_INFO("initializing...");
  parameters_ = parameters;

  string robot_description_name = nh_.resolveName("robot_description", true);
  ROS_INFO_STREAM(robot_description_name);

  cm_.reset(new CollisionModels(robot_description_name));


  robot_state_ = new KinematicState(cm_->getKinematicModel());
  robot_state_->setKinematicStateToDefault();

  joint_state_subscriber_ = nh_.subscribe("joint_states", 25, &WorkspaceEditor::jointStateCallback, this);

  interactive_marker_server_.reset(
      new interactive_markers::InteractiveMarkerServer("personal_robotic_workspace_control", "", false));

  ik_control_feedback_ptr_ = boost::bind(&WorkspaceEditor::IKControllerCallback, this, _1);


  vis_marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(parameters_->visualizer_topic_name_, 128);
  vis_marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(parameters_->visualizer_topic_name_ + "_array", 128);


  while (ros::ok() && !ros::service::waitForService(parameters_->set_planning_scene_diff_name_, ros::Duration(1.0)))
  {
    //ROS_INFO_STREAM("Waiting for planning scene service " << parameters_->set_planning_scene_diff_name_);
  }
  set_planning_scene_diff_service_ = nh_.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(
      parameters_->set_planning_scene_diff_name_);

  while (!ros::service::waitForService(parameters_->planner_service_name_, ros::Duration(1.0)))
  {
    //ROS_INFO_STREAM("Waiting for planner service " << parameters_->planner_service_name_);
  }

  planning_service_ = nh_.serviceClient<arm_navigation_msgs::GetMotionPlan>(parameters_->planner_service_name_, true);

  while (!ros::service::waitForService(parameters_->trajectory_filter_service_name_, ros::Duration(1.0)))
  {
    //ROS_INFO_STREAM("Waiting for trajectory filter service " << parameters_->trajectory_filter_service_name_);
  }

  trajectory_filter_service_ = nh_.serviceClient<
      arm_navigation_msgs::FilterJointTrajectoryWithConstraints>(parameters_->trajectory_filter_service_name_, true);





  for(int i = 0; i < parameters_->number_of_arm_; i++)
  {
    if(parameters_->arm_group_[i] != "none")
    {
      collision_aware_ik_services_[parameters_->arm_group_[i]] = nh_.serviceClient<GetConstraintAwarePositionIK>(
          parameters_->ik_name_[i], true);

      non_collision_aware_ik_services_[parameters_->arm_group_[i]] = nh_.serviceClient<GetPositionIK>(
          parameters_->non_collision_ik_name_[i], true);

      arm_controller_map_[parameters_->arm_group_[i]].reset(new actionlib::SimpleActionClient<
        control_msgs::FollowJointTrajectoryAction>(parameters_->arm_controller_[i], true));
      while(ros::ok() && !arm_controller_map_[parameters_->arm_group_[i]]->waitForServer(ros::Duration(1.0)))
      {
        //ROS_INFO_STREAM("Waiting for the " + parameters_->arm_controller_[i] + " server to come up.");
      }
    }
  }



  //sendPlanningScene();

  ROS_INFO("initialized");
}



void WorkspaceEditor::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state)
{
  if(robot_state_ == NULL) return;

  std::map<std::string, double> joint_state_map;
  std::map<std::string, double> joint_velocity_map;

  //message already been validated in kmsm
  if ( joint_state->velocity.size() == joint_state->position.size() )
  {
    for(unsigned int i = 0; i < joint_state->position.size(); ++i)
    {
      joint_state_map[joint_state->name[i]] = joint_state->position[i];
      joint_velocity_map[joint_state->name[i]] = joint_state->velocity[i];
    }
  }
  else
  {
    for(unsigned int i = 0; i < joint_state->position.size(); ++i)
    {
      joint_state_map[joint_state->name[i]] = joint_state->position[i];
      joint_velocity_map[joint_state->name[i]] = 0.0;
    }
  }

  mutex_.lock();
  std::vector<planning_models::KinematicState::JointState*>& joint_state_vector = robot_state_->getJointStateVector();
  for(std::vector<planning_models::KinematicState::JointState*>::iterator it = joint_state_vector.begin();
      it != joint_state_vector.end();
      it++) {
    bool tfSets = false;
    //see if we need to update any transforms
    std::string parent_frame_id = (*it)->getParentFrameId();
    std::string child_frame_id = (*it)->getChildFrameId();
    if(!parent_frame_id.empty() && !child_frame_id.empty()) {
      std::string err;
      ros::Time tm;
      tf::StampedTransform transf;
      bool ok = false;
      if (transform_listener_.getLatestCommonTime(parent_frame_id, child_frame_id, tm, &err) == tf::NO_ERROR) {
        ok = true;
        try
        {
          transform_listener_.lookupTransform(parent_frame_id, child_frame_id, tm, transf);
        }
        catch(tf::TransformException& ex)
        {
          ROS_ERROR("Unable to lookup transform from %s to %s.  Exception: %s", parent_frame_id.c_str(), child_frame_id.c_str(), ex.what());
          ok = false;
        }
      } else {
        ROS_DEBUG("Unable to lookup transform from %s to %s: no common time.", parent_frame_id.c_str(), child_frame_id.c_str());
        ok = false;
      }
      if(ok) {
        tfSets = (*it)->setJointStateValues(transf);
      }
    }
    (*it)->setJointStateValues(joint_state_map);
  }
  robot_state_->updateKinematicLinks();
  robot_state_->getKinematicStateValues(robot_state_joint_values_);
  mutex_.unlock();
}

bool WorkspaceEditor::sendPlanningScene(PlanningSceneData& data)
{
  SetPlanningSceneDiff::Request planning_scene_req;
  SetPlanningSceneDiff::Response planning_scene_res;

  mutex_.lock();

  planning_scene_req.planning_scene_diff = data.planning_scene_;

  //just will get latest data from the monitor
  arm_navigation_msgs::RobotState emp;
  planning_scene_req.planning_scene_diff.robot_state = emp;


  collision_space::EnvironmentModel::AllowedCollisionMatrix acm;
  if(planning_scene_req.planning_scene_diff.allowed_collision_matrix.link_names.empty()) {
    acm = cm_->getDefaultAllowedCollisionMatrix();
  } else {
    acm = planning_environment::convertFromACMMsgToACM(planning_scene_req.planning_scene_diff.allowed_collision_matrix);
  }
  planning_scene_req.planning_scene_diff.collision_objects = std::vector<CollisionObject>();
  planning_scene_req.planning_scene_diff.attached_collision_objects = std::vector<AttachedCollisionObject>();

  deleteKinematicStates();


  if (robot_state_ != NULL)
  {
    ROS_INFO("Reverting planning scene to default.");
    cm_->revertPlanningScene(robot_state_);
    robot_state_ = NULL;
  }

  if(!set_planning_scene_diff_service_.call(planning_scene_req, planning_scene_res))
  {
    ROS_WARN("Can't get planning scene");
    mutex_.unlock();
    return false;
  }

  robot_state_ = cm_->setPlanningScene(planning_scene_res.planning_scene);

  robot_state_->getKinematicStateValues(robot_state_joint_values_);

  mutex_.unlock();
  return true;
}

void WorkspaceEditor::deleteKinematicStates()
{
  mutex_.lock();
  std::vector<KinematicState*> removals;
  for(map<string, map<string, TrajectoryData> >::iterator it = trajectory_map_.begin(); it != trajectory_map_.end(); it++)
  {
    for(map<string, TrajectoryData>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++) {
      removals.push_back(it2->second.current_state_);
      it2->second.reset();
    }
  }

  for(map<string, MotionPlanRequestData>::iterator it = motion_plan_map_.begin(); it != motion_plan_map_.end(); it++)
  {
    removals.push_back(it->second.start_state_);
    removals.push_back(it->second.goal_state_);
    it->second.reset();
  }

  /*
  for(size_t i = 0; i < states_.size(); i++)
  {
    if(states_[i].state != NULL)
    {
      bool shouldBreak = false;
      for(size_t j = 0; j < removals.size(); j++)
      {
        if(states_[i].state == removals[j])
        {
          shouldBreak = true;
          break;
        }
      }

      if(shouldBreak)
      {
        continue;
      }
      ROS_INFO("Missed a state from %s!", states_[i].source.c_str());
      delete states_[i].state;
      states_[i].state = NULL;
    }
  }
  states_.clear();
  */
  mutex_.unlock();
}

void WorkspaceEditor::sendMarkers()
{
  mutex_.lock();
  visualization_msgs::MarkerArray arr;





  vis_marker_array_publisher_.publish(arr);


  mutex_.unlock();

}

void getTrajectoryMarkers(visualization_msgs::MarkerArray& arr)
{

}

void WorkspaceEditor::getMotionPlanningMarkers(visualization_msgs::MarkerArray& arr)
{
  vector<string> removals;

  // For each motion plan request ...
  for (map<string, MotionPlanRequestData>::iterator it = motion_plan_map_.begin(); it != motion_plan_map_.end(); it++)
  {
    if (it->second.name_ == "")
    {
      ROS_WARN("Someone's making empty stuff");
    }
    MotionPlanRequestData& data = it->second;

    // TODO: Find out why this happens.
    if (motion_plan_map_.find(it->first) == motion_plan_map_.end() || data.getName() == "")
    {
      ROS_WARN("Attempting to publish non-existant motion plan request %s Erasing this request!", it->first.c_str());
      removals.push_back(it->first);
      continue;
    }

    // TODO: Find out why this happens.
    if (data.start_state_ == NULL || data.goal_state_ == NULL)
    {
      return;
    }

    // When a motion plan request has its colors changed,
    // we must wait a few milliseconds before rviz registers the change.
    if (data.shouldRefreshColors())
    {
      data.refresh_timer_ += marker_dt_;

      if (data.refresh_timer_.toSec() > MARKER_REFRESH_TIME + 0.05)
      {
        data.setHasRefreshedColors(true);
        data.refresh_timer_ = ros::Duration(0.0);
      }
    }
    else
    {
      std_msgs::ColorRGBA fail_color;
      fail_color.a = 0.9;
      fail_color.r = 1.0;
      fail_color.g = 0.0;
      fail_color.b = 0.0;

      /////
      /// Get markers for the start
      /////
      if (data.isStartVisible())
      {
        const vector<const KinematicModel::LinkModel*>& updated_links = cm_->getKinematicModel()->getModelGroup(
            data.getMotionPlanRequest().group_name)->getUpdatedLinkModels();

        vector<string> lnames;
        lnames.resize(updated_links.size());
        for (unsigned int i = 0; i < updated_links.size(); i++)
        {
          lnames[i] = updated_links[i]->getName();
        }

        // If we have a good ik solution, publish with the normal color
        // else use bright red.
        std_msgs::ColorRGBA col;
        if (data.hasGoodIKSolution(StartPosition))
        {
          col = data.getStartColor();
        }
        else
        {
          col = fail_color;
        }

        switch (data.getRenderType())
        {
          case VisualMesh:
            cm_->getRobotMarkersGivenState(*(data.getStartState()), arr, col, it->first + "_start",
                                           ros::Duration(MARKER_REFRESH_TIME), &lnames, 1.0, false);
            // Bodies held by robot
            cm_->getAttachedCollisionObjectMarkers(*(data.getStartState()), arr, it->first + "_start", col,
                                                   ros::Duration(MARKER_REFRESH_TIME), false, &lnames);

            break;
          case CollisionMesh:
            cm_->getRobotMarkersGivenState(*(data.getStartState()), arr, col, it->first + "_start",
                                           ros::Duration(MARKER_REFRESH_TIME), &lnames, 1.0, true);
            cm_->getAttachedCollisionObjectMarkers(*(data.getStartState()), arr, it->first + "_start", col,
                                                   ros::Duration(MARKER_REFRESH_TIME), false, &lnames);
            break;
          case PaddingMesh:
            cm_->getRobotPaddedMarkersGivenState(*(data.getStartState()), arr, col, it->first + "_start",
                                                 ros::Duration(MARKER_REFRESH_TIME), (const vector<string>*)&lnames);
            cm_->getAttachedCollisionObjectMarkers(*(data.getStartState()), arr, it->first + "_start", col,
                                                   ros::Duration(MARKER_REFRESH_TIME), true, &lnames);
            break;
        }
      }

      /////
      /// Get markers for the end.
      /////
      if (data.isEndVisible())
      {
        const vector<const KinematicModel::LinkModel*>& updated_links = cm_->getKinematicModel()->getModelGroup(
            data.getMotionPlanRequest().group_name)->getUpdatedLinkModels();

        vector<string> lnames;
        lnames.resize(updated_links.size());
        for (unsigned int i = 0; i < updated_links.size(); i++)
        {
          lnames[i] = updated_links[i]->getName();
        }

        std_msgs::ColorRGBA col;
        if (data.hasGoodIKSolution(GoalPosition))
        {
          col = data.getGoalColor();
        }
        else
        {
          col = fail_color;
        }

        switch (data.getRenderType())
        {
          case VisualMesh:
            cm_->getRobotMarkersGivenState(*(data.getGoalState()), arr, col, it->first + "_Goal",
                                           ros::Duration(MARKER_REFRESH_TIME), &lnames, 1.0, false);

            // Bodies held by robot
            cm_->getAttachedCollisionObjectMarkers(*(data.getGoalState()), arr, it->first + "_Goal", col,
                                                   ros::Duration(MARKER_REFRESH_TIME), false, &lnames);

            break;
          case CollisionMesh:
            cm_->getRobotMarkersGivenState(*(data.getGoalState()), arr, col, it->first + "_Goal",
                                           ros::Duration(MARKER_REFRESH_TIME), &lnames, 1.0, true);
            cm_->getAttachedCollisionObjectMarkers(*(data.getGoalState()), arr, it->first + "_Goal", col,
                                                   ros::Duration(MARKER_REFRESH_TIME), false, &lnames);
            break;
          case PaddingMesh:
            cm_->getRobotPaddedMarkersGivenState(*(data.getGoalState()), arr, col, it->first + "_Goal",
                                                 ros::Duration(MARKER_REFRESH_TIME), (const vector<string>*)&lnames);
            cm_->getAttachedCollisionObjectMarkers(*(data.getGoalState()), arr, it->first + "_Goal", col,
                                                   ros::Duration(MARKER_REFRESH_TIME), true, &lnames);
            break;
        }
      }
    }

    //////
    /// Get collision markers for the start and end state.
    /////
    if (it->second.areCollisionsVisible() && (it->second.isStartVisible() || it->second.isEndVisible()))
    {
      // Update collision markers
      if (it->second.hasStateChanged())
      {
        if (params_.proximity_space_validity_name_ == "none")
        {
          it->second.updateCollisionMarkers(cm_, NULL);
        }
        else
        {
          it->second.updateCollisionMarkers(cm_, &distance_state_validity_service_client_);
        }
        it->second.setStateChanged(false);
      }

      // Add them to the global array.
      for (size_t i = 0; i < it->second.getCollisionMarkers().markers.size(); i++)
      {
        collision_markers_.markers.push_back(it->second.getCollisionMarkers().markers[i]);
      }
    }

  }

  /////
  /// TODO: Figure out why motion plans are occasionally NULL
  ////
  for (size_t i = 0; i < removals.size(); i++)
  {
    motion_plan_map_.erase(removals[i]);
  }
}


void WorkspaceEditor::createIKController(MotionPlanRequestData& data, PositionType type, bool rePose)
{
  KinematicState* state = NULL;
  std::string nametag = "";
  if (type == StartPosition)
  {
    state = data.start_state_;
    nametag = "_start_control";
  }
  else
  {
    state = data.goal_state_;
    nametag = "_end_control";
  }

  tf::Transform transform = state->getLinkState(data.end_effector_link_)->getGlobalLinkTransform();
  InteractiveMarker marker;

  if (interactive_marker_server_->get(data.name_ + nametag, marker) && rePose)
  {
    geometry_msgs::Pose pose = toGeometryPose(transform);
    interactive_marker_server_->setPose(data.name_ + nametag, pose);
    return;
  }

  marker.header.frame_id = "/" + cm_->getWorldFrameId();
  marker.pose.position.x = transform.getOrigin().x();
  marker.pose.position.y = transform.getOrigin().y();
  marker.pose.position.z = transform.getOrigin().z();
  marker.pose.orientation.w = transform.getRotation().w();
  marker.pose.orientation.x = transform.getRotation().x();
  marker.pose.orientation.y = transform.getRotation().y();
  marker.pose.orientation.z = transform.getRotation().z();
  marker.scale = 0.225f;
  marker.name = data.name_ + nametag;
  marker.description = data.name_ + nametag;

  InteractiveMarkerControl control;
  control.always_visible = false;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0;

  marker.controls.push_back(control);

  InteractiveMarkerControl control2;
  control2.always_visible = false;
  control2.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  control2.orientation.w = 1;
  control2.orientation.x = 0;
  control2.orientation.y = 1;
  control2.orientation.z = 0;

  Marker marker2;
  marker2.type = Marker::CUBE;
  marker2.scale.x = .2;
  marker2.scale.y = .15;
  marker2.scale.z = .002;
  marker2.pose.position.x = .1;
  marker2.color.r = 0;
  marker2.color.g = 0;
  marker2.color.b = 0.5;
  marker2.color.a = 1;
  control2.markers.push_back(marker2);
  marker2.scale.x = .1;
  marker2.scale.y = .35;
  marker2.pose.position.x = 0;
  control2.markers.push_back(marker2);

  marker.controls.push_back(control2);

  InteractiveMarkerControl control3;
  control3.always_visible = false;
  control3.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  control3.orientation.w = 1;
  control3.orientation.x = 0;
  control3.orientation.y = 0;
  control3.orientation.z = 1;

  Marker marker3;
  marker3.type = Marker::CUBE;
  marker3.scale.x = .2;
  marker3.scale.y = .002;
  marker3.scale.z = .15;
  marker3.pose.position.x = .1;
  marker3.color.r = 0;
  marker3.color.g = .5;
  marker3.color.b = 0;
  marker3.color.a = 1;
  control3.markers.push_back(marker3);
  marker3.scale.x = .1;
  marker3.scale.z = .35;
  marker3.pose.position.x = 0;
  control3.markers.push_back(marker3);

  marker.controls.push_back(control3);

  interactive_marker_server_->insert(marker, ik_control_feedback_ptr_);
}

void WorkspaceEditor::IKControllerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  ROS_INFO_THROTTLE(1.0, __FUNCTION__);
}

void WorkspaceEditor::createNewPlanningScene(const std::string& name, unsigned int id)
{
  mutex_.lock();
  if(robot_state_ == NULL)
  {
    robot_state_ = new KinematicState(cm_->getKinematicModel());
  }
  else
  {
    if (robot_state_joint_values_.empty())
    {
      robot_state_->setKinematicStateToDefault();
    }
    else
    {
      robot_state_->setKinematicState(robot_state_joint_values_);
    }
  }

  PlanningSceneData data;
  data.name_ = name;
  data.id_ = id;
  data.timestamp_ = ros::Time(ros::WallTime::now().toSec());

  convertKinematicStateToRobotState(*robot_state_, data.timestamp_, cm_->getWorldFrameId(),
                                    data.planning_scene_.robot_state);


  //add all previously added objects in the scene
  std::vector<string> collisionObjects;
  for (map<string, SelectableObject>::iterator it = selectable_objects_->begin(); it != selectable_objects_->end();
      it++)
  {
    collisionObjects.push_back(it->first);
  }

  //this also does attached collision objects
  for(size_t i = 0; i < collisionObjects.size(); i++)
  {
    deleteCollisionObject(collisionObjects[i]);
  }

  selectable_objects_->clear();

  char hostname[256];
  gethostname(hostname, 256);
  data.host_ = string(hostname);

  planning_scene_map_[data.name_] = data;



  mutex_.unlock();
}

void WorkspaceEditor::deleteCollisionObject(std::string& name)
{
  (*selectable_objects_)[name].collision_object_.operation.operation
      = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
  (*selectable_objects_)[name].attached_collision_object_.object.operation.operation
      = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
  interactive_marker_server_->erase((*selectable_objects_)[name].selection_marker_.name);
  interactive_marker_server_->erase((*selectable_objects_)[name].control_marker_.name);
  interactive_marker_server_->applyChanges();
}

void WorkspaceEditor::createIkControllersFromMotionPlanRequest(MotionPlanRequestData& data, bool rePose)
{
  if(data.is_start_editable_)
  {
    createIKController(data, StartPosition, rePose);
  }

  if(data.is_goal_editable_)
  {
    createIKController(data, GoalPosition, rePose);
  }

  interactive_marker_server_->applyChanges();
}


void WorkspaceEditor::createMotionPlanRequest(const planning_models::KinematicState& start_state,
                                              const planning_models::KinematicState& end_state,
                                              const std::string& group_name,
                                              const std::string& end_effector_name,
                                              const unsigned int& planning_scene_id,
                                              const bool from_robot_state,
                                              unsigned int& motion_plan_id_out)
{
  MotionPlanRequest motion_plan_request;
  motion_plan_request.group_name = group_name;
  motion_plan_request.num_planning_attempts = 1;
  motion_plan_request.allowed_planning_time = ros::Duration(1);
  const KinematicState::JointStateGroup* jsg = end_state.getJointStateGroup(group_name);
  motion_plan_request.goal_constraints.joint_constraints.resize(jsg->getJointNames().size());

  // Must convert kinematic state to robot state message.
  vector<double> joint_values;
  jsg->getKinematicStateValues(joint_values);
  for (unsigned int i = 0; i < jsg->getJointNames().size(); i++)
  {
    motion_plan_request.goal_constraints.joint_constraints[i].joint_name = jsg->getJointNames()[i];
    motion_plan_request.goal_constraints.joint_constraints[i].position = joint_values[i];
    motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.001;
    motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.001;
  }

  // Create start state from kinematic state passed in if robot data is being used
  if (!from_robot_state)
  {
    convertKinematicStateToRobotState(start_state, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
                                      motion_plan_request.start_state);
  }
  // Otherwise, use the current robot state.
  else
  {
    convertKinematicStateToRobotState(*robot_state_, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
                                      motion_plan_request.start_state);
  }

  if (planning_scene_map_.find(hg::getPlanningSceneNameFromId(planning_scene_id)) == planning_scene_map_.end())
  {
    ROS_WARN_STREAM("Creating new planning scene for motion plan request - bad!!");
  }

  PlanningSceneData& planningSceneData = planning_scene_map_[getPlanningSceneNameFromId(planning_scene_id)];

  // Turn the motion plan request message into a MotionPlanData
  unsigned int id = planningSceneData.getNextMotionPlanRequestId();
  motion_plan_request.group_name = group_name;
  MotionPlanRequestData data(id, "Planner", motion_plan_request, robot_state_, end_effector_name);
  data.is_goal_editable_ = true;
  if (from_robot_state)
  {
    data.is_start_editable_ = false;
  }

  /*
  // Book keeping for kinematic state storage
  StateRegistry start;
  start.state = data.getStartState();
  start.source = "Motion Plan Request Data Start create request";
  StateRegistry end;
  end.state = data.getGoalState();
  end.source = "Motion Plan Request Data End from create request";
  states_.push_back(start);
  states_.push_back(end);
  */

  motion_plan_map_[getMotionPlanRequestNameFromId(id)] = data;
  data.planning_scene_id_ = planning_scene_id;

  // Add request to the planning scene
  planningSceneData.addMotionPlanRequestId(id);

  motion_plan_id_out = data.id_;
  createIkControllersFromMotionPlanRequest(data, false);
  sendPlanningScene(planningSceneData);
}


WorkspaceEditor::~WorkspaceEditor()
{

}
