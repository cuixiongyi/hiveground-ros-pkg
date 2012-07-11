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

void WorkspaceEditor::makeTopLevelMenu()
{
  InteractiveMarker int_marker;
  int_marker.pose.position.z = 2.25;
  int_marker.name = "top_level";
  int_marker.description = "Personal Robotic Workspace Visualizer";
  int_marker.header.frame_id = "/" + cm_->getWorldFrameId();

  InteractiveMarkerControl control;
  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.always_visible = true;

  Marker labelMarker;
  labelMarker.type = Marker::TEXT_VIEW_FACING;
  labelMarker.text = "Command...";
  labelMarker.color.r = 1.0;
  labelMarker.color.g = 1.0;
  labelMarker.color.b = 1.0;
  labelMarker.color.a = 1.0;
  labelMarker.scale.x = 0.5;
  labelMarker.scale.y = 0.2;
  labelMarker.scale.z = 0.1;
  control.markers.push_back(labelMarker);

  int_marker.controls.push_back(control);

  interactive_marker_server_->insert(int_marker, process_menu_feedback_ptr_);
  menu_handler_map_["Top Level"].apply(*interactive_marker_server_, int_marker.name);
}

void WorkspaceEditor::makeIKControllerMarker(tf::Transform transform, std::string name, std::string description,
                                             bool selectable, float scale, bool publish)
{

  SelectableMarker selectable_marker;
  selectable_marker.type_ = EndEffectorControl;
  selectable_marker.name_ = name + "_selectable";
  selectable_marker.controlName_ = name;
  selectable_marker.controlDescription_ = description;


  InteractiveMarker marker;
  marker.header.frame_id = "/" + cm_->getWorldFrameId();
  marker.pose.position.x = transform.getOrigin().x();
  marker.pose.position.y = transform.getOrigin().y();
  marker.pose.position.z = transform.getOrigin().z();
  marker.pose.orientation.w = transform.getRotation().w();
  marker.pose.orientation.x = transform.getRotation().x();
  marker.pose.orientation.y = transform.getRotation().y();
  marker.pose.orientation.z = transform.getRotation().z();
  marker.scale = 0.225f;
  if(selectable)
    marker.name = name + "_selectable";
  else
    marker.name = name;
  marker.description = description;

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

  interactive_marker_server_->insert(marker, process_ik_controller_feedback_ptr_);

  if(selectable)
    selectable_markers_[marker.name] = selectable_marker;

  if(publish)
  {
    interactive_marker_server_->applyChanges();
  }
}

void WorkspaceEditor::selectMarker(SelectableMarker& marker, tf::Transform transform)
{
  InteractiveMarker dummy;
  if (interactive_marker_server_->get(marker.controlName_, dummy))
  {
    dummy.header.stamp = ros::Time::now();
    interactive_marker_server_->setPose(marker.controlName_, toGeometryPose(transform), dummy.header);
  }
  else
  {
    if (!interactive_marker_server_->erase(marker.name_))
    {
      return;
    }

    switch (marker.type_)
    {
      case EndEffectorControl:
        makeIKControllerMarker(transform, marker.controlName_, marker.controlDescription_, false, 0.5f);
        break;
      case CollisionObject:
        break;
      case JointControl:
        break;
    }
  }
}

void WorkspaceEditor::deselectMarker(SelectableMarker& marker, tf::Transform transform)
{
  if (!interactive_marker_server_->erase(marker.controlName_))
  {
    return;
  }

  float scale = 1.0f;

  switch (marker.type_)
  {
    case EndEffectorControl:
      makeIKControllerMarker(transform, marker.controlName_, marker.controlDescription_, true, 0.5f);
      break;
    case CollisionObject:
      scale = 2.0f;
      break;
    case JointControl:
      scale = 0.225f;
      break;
  }
}

void WorkspaceEditor::processIKControllerFeedback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  //ROS_INFO(__FUNCTION__);
  PlanningGroupData& gc = group_map_[current_group_name_];
  switch (feedback->event_type)
  {
    case InteractiveMarkerFeedback::BUTTON_CLICK: break;
    case InteractiveMarkerFeedback::MENU_SELECT: break;
    case InteractiveMarkerFeedback::MOUSE_UP:
      if(gc.good_ik_solution_)
      {
        planToEndEffectorState(gc, true, true);
        filterPlannerTrajectory(gc, true, true);

        if(gc.trajectory_data_map_["filter"].has_joint_trajectory_)
        {
          FollowJointTrajectoryGoal goal;
          goal.trajectory = gc.trajectory_data_map_["filter"].joint_trajectory_;
          goal.trajectory.header.stamp = ros::Time::now();
          gc.arm_controller_->sendGoal(goal, boost::bind(&WorkspaceEditor::controllerDoneCallback, this, _1, _2));
        }



      }
      break;

    case InteractiveMarkerFeedback::MOUSE_DOWN: break;
    case InteractiveMarkerFeedback::POSE_UPDATE:
      //ROS_INFO_STREAM("group " << feedback->marker_name);
      if(is_ik_control_active_ && isGroupName(feedback->marker_name))
      {

        //if(gc.arm_controller_->)


        //ROS_INFO_STREAM(feedback->pose);
        tf::Transform cur = toBulletTransform(feedback->pose);
        setNewEndEffectorPosition(gc, cur, collision_aware_);
        last_ee_poses_[current_group_name_] = feedback->pose;
      }
      else if (is_joint_control_active_ && feedback->marker_name.rfind("_joint_control") != string::npos)
      {
        //tf::Transform cur = toBulletTransform(feedback->pose);

        //string jointName = feedback->marker_name.substr(0, feedback->marker_name.rfind("_joint_control"));
        //setJointState(gc, jointName, cur);
      }
      break;
    case InteractiveMarkerFeedback::KEEP_ALIVE: break;
  }
  interactive_marker_server_->applyChanges();
}

void WorkspaceEditor::processMenuFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  //ROS_INFO(__FUNCTION__);
}

