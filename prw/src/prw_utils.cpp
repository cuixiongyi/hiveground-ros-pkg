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



  sendPlanningScene();

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

bool WorkspaceEditor::sendPlanningScene()
{
  SetPlanningSceneDiff::Request planning_scene_req;
  SetPlanningSceneDiff::Response planning_scene_res;

  mutex_.lock();
  planning_environment::convertKinematicStateToRobotState(*robot_state_, ros::Time::now(), cm_->getWorldFrameId(),
                                                          planning_scene_req.planning_scene_diff.robot_state);


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

void WorkspaceEditor::sendMarkers()
{
  mutex_.lock();
  visualization_msgs::MarkerArray arr;





  vis_marker_array_publisher_.publish(arr);


  mutex_.unlock();

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

void IKControllerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{

}

void WorkspaceEditor::createNewPlanningScene()
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

WorkspaceEditor::~WorkspaceEditor()
{

}
