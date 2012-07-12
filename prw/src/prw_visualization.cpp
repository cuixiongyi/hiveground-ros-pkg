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

void WorkspaceEditor::makeCollisionObjectContextMenu(tf::Transform transform, const std::string& name,
                                                     const std::string& description, float scale)
{
  makeSelectableMarker(CollisionObject, transform, name, description, scale);
}

void WorkspaceEditor::createCollisionPole(int id, geometry_msgs::Pose pose, double diameter, double length)
{
  ROS_INFO("Creating collision pole %d", id);

  arm_navigation_msgs::CollisionObject cylinder_object;
  cylinder_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  cylinder_object.header.stamp = ros::Time::now();
  cylinder_object.header.frame_id = "/" + cm_->getWorldFrameId();
  arm_navigation_msgs::Shape object;
  object.type = arm_navigation_msgs::Shape::CYLINDER;
  object.dimensions.resize(2);
  object.dimensions[0] = diameter;
  object.dimensions[1] = length;

  cylinder_object.shapes.push_back(object);
  cylinder_object.poses.push_back(pose);
  stringstream name;
  name << "pole_" << id;
  cylinder_object.id = name.str();
  collision_objects_[name.str()] = cylinder_object;

  tf::Transform cur = toBulletTransform(pose);
  makeCollisionObjectContextMenu(cur, name.str(), "", 2.0f);
}

int WorkspaceEditor::getNextCollisionObjectId()
{
  return last_collision_objects_id_++;
}
void WorkspaceEditor::createCollisionObject(geometry_msgs::Pose pose,
                                            CollisionObjectType type,
                                            std_msgs::ColorRGBA color,
                                            double a,
                                            double b,
                                            double c,
                                            bool selectable)
{

  arm_navigation_msgs::CollisionObject collision_object;
  collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  collision_object.header.stamp = ros::Time::now();
  collision_object.header.frame_id = "/" + cm_->getWorldFrameId();

  arm_navigation_msgs::Shape object;
  stringstream id;
  id << "object_" << getNextCollisionObjectId();

  switch(type)
  {
    case Sphere:
      object.type = arm_navigation_msgs::Shape::SPHERE;
      object.dimensions.resize(1);
      object.dimensions[0] = a;
      id << "_sphere";
      break;
    case Pole:
      object.type = arm_navigation_msgs::Shape::CYLINDER;
      object.dimensions.resize(2);
      object.dimensions[0] = a;
      object.dimensions[1] = b;
      id << "_pole";
      break;
    case Box:
      object.type = arm_navigation_msgs::Shape::BOX;
      object.dimensions.resize(3);
      object.dimensions[0] = a;
      object.dimensions[1] = b;
      object.dimensions[2] = c;
      id << "_box";
      break;
  }

  collision_object.shapes.push_back(object);
  collision_object.poses.push_back(pose);
  collision_object.id = id.str();
  collision_objects_[id.str()] = collision_object;

  ROS_INFO_STREAM("Creating collision object: " << collision_object.id);




  if(selectable)
  {
    tf::Transform cur = toBulletTransform(pose);
    makeSelectableCollisionObjectMarker(cur, type, collision_object.id, color, a, b, c);
  }
}

void WorkspaceEditor::makeSelectableCollisionObjectMarker(tf::Transform transform,
                                                          CollisionObjectType type,
                                                          const std::string& name,
                                                          std_msgs::ColorRGBA color,
                                                          double a,
                                                          double b,
                                                          double c,
                                                          bool publish)
{
  SelectableMarker selectable_marker;
  selectable_marker.type_ = CollisionObject;
  selectable_marker.name_ = name + "_selectable";
  selectable_marker.controlName_ = name;
  selectable_marker.controlDescription_ = "";
  selectable_marker.a_ = a;
  selectable_marker.b_ = b;
  selectable_marker.c_ = c;
  selectable_marker.color_ = color;
  selectable_marker.collision_object_type_ = type;
  selectable_marker.pose_ = toGeometryPose(transform);

  InteractiveMarker marker;
  marker.header.frame_id = "/" + cm_->getWorldFrameId();
  marker.header.stamp = ros::Time::now();
  marker.pose.position.x = transform.getOrigin().x();
  marker.pose.position.y = transform.getOrigin().y();
  marker.pose.position.z = transform.getOrigin().z();
  marker.pose.orientation.w = transform.getRotation().w();
  marker.pose.orientation.x = transform.getRotation().x();
  marker.pose.orientation.y = transform.getRotation().y();
  marker.pose.orientation.z = transform.getRotation().z();
  marker.scale = 1.0;
  marker.name = selectable_marker.name_;
  marker.description = "";

  InteractiveMarkerControl control;
  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.always_visible = true;
  control.markers.push_back(createCollisionObjectMarker(type, a + 0.005, b + 0.005, c + 0.005, color));
  marker.controls.push_back(control);
  selectable_markers_[marker.name] = selectable_marker;
  interactive_marker_server_->insert(marker);
  interactive_marker_server_->setCallback(marker.name, process_marker_feedback_ptr_);
  menu_handler_map_["Collision Object Selection"].apply(*interactive_marker_server_, marker.name);

  if(publish)
  {
    interactive_marker_server_->applyChanges();
  }

}


visualization_msgs::Marker WorkspaceEditor::createCollisionObjectMarker(CollisionObjectType type,
                                                                        double a,
                                                                        double b,
                                                                        double c,
                                                                        std_msgs::ColorRGBA color)
{
  Marker marker;
  switch(type)
  {
    case Sphere:
      marker.type = Marker::SPHERE;
      marker.scale.x = 2.0 * a;
      marker.scale.y = 2.0 * a;
      marker.scale.z = 2.0 * a;
      break;
    case Pole:
      marker.type = Marker::CYLINDER;
      marker.scale.x = 2.0 * a;
      marker.scale.y = 2.0 * a;
      marker.scale.z = b;
      break;
    case Box:
      marker.type = Marker::CUBE;
      marker.scale.x = a;
      marker.scale.y = b;
      marker.scale.z = c;
      break;
  }
  marker.color = color;
  return marker;
}

InteractiveMarkerControl& WorkspaceEditor::makeInteractiveCollisionObjectControl(InteractiveMarker &msg)
{

  InteractiveMarkerControl control;
  control.always_visible = true;
  SelectableMarker& sm = selectable_markers_[msg.name+"_selectable"];
  //ROS_INFO_STREAM("aaaa " << sm.name_ << " " << sm.a_  << " " << sm.b_  << " " << sm.c_);
  control.markers.push_back(
      createCollisionObjectMarker(sm.collision_object_type_, sm.a_ + 0.005, sm.b_ + 0.005, sm.c_ + 0.005, sm.color_));
  msg.controls.push_back(control);
  return msg.controls.back();
}


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

void WorkspaceEditor::makeIKControllerMarker(tf::Transform transform, const std::string& name, const std::string& description,
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

void WorkspaceEditor::makeSelectableMarker(InteractiveMarkerType type,
                                           tf::Transform transform,
                                           const std::string& name,
                                           const std::string& description,
                                           float scale, bool publish)
{
  if(type == EndEffectorControl)
  {
    ROS_WARN("handle separately");
    return;
  }



  SelectableMarker selectable_marker;
  selectable_marker.type_ = type;
  selectable_marker.name_ = name + "_selectable";
  selectable_marker.controlName_ = name;
  selectable_marker.controlDescription_ = description;

  InteractiveMarker marker;
  marker.header.frame_id = "/" + cm_->getWorldFrameId();

  marker.header.stamp = ros::Time::now();
  marker.pose.position.x = transform.getOrigin().x();
  marker.pose.position.y = transform.getOrigin().y();
  marker.pose.position.z = transform.getOrigin().z();
  marker.pose.orientation.w = transform.getRotation().w();
  marker.pose.orientation.x = transform.getRotation().x();
  marker.pose.orientation.y = transform.getRotation().y();
  marker.pose.orientation.z = transform.getRotation().z();
  marker.scale = scale;
  marker.name = name + "_selectable";
  marker.description = description;
  InteractiveMarkerControl control;
  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  switch (type)
  {
    case EndEffectorControl:
      break;
    case CollisionObject:
      ROS_INFO("a");
      control.markers.push_back(makeMarkerCylinder(marker, 1.0f));
      marker.controls.push_back(control);
      interactive_marker_server_->insert(marker);
      interactive_marker_server_->setCallback(marker.name, process_marker_feedback_ptr_);
      menu_handler_map_["Collision Object Selection"].apply(*interactive_marker_server_, marker.name);
      break;
    case JointControl:
      control.markers.push_back(makeMarkerBox(marker, 0.5f));
      marker.controls.push_back(control);
      interactive_marker_server_->insert(marker);
      interactive_marker_server_->setCallback(marker.name, process_marker_feedback_ptr_);
      menu_handler_map_["Joint Selection"].apply(*interactive_marker_server_, marker.name);
      break;
  }

  selectable_markers_[marker.name] = selectable_marker;

  if (publish)
  {
    interactive_marker_server_->applyChanges();
  }

}

Marker WorkspaceEditor::makeMarkerCylinder(InteractiveMarker &msg, float alpha)
{
  Marker marker;
  marker.type = Marker::CYLINDER;
  // Scale is arbitrary
  marker.scale.x = msg.scale * 0.11;
  marker.scale.y = msg.scale * 0.11;
  marker.scale.z = msg.scale * 1.1;
  marker.color.r = 0.2;
  marker.color.g = 0.9;
  marker.color.b = 0.2;
  marker.color.a = alpha;

  return marker;
}

Marker WorkspaceEditor::makeMarkerBox(InteractiveMarker &msg, float alpha)
{
  Marker marker;
  marker.type = Marker::CUBE;
  // Scale is arbitrarily 1/4 of the marker's scale.
  marker.scale.x = msg.scale * 0.25;
  marker.scale.y = msg.scale * 0.25;
  marker.scale.z = msg.scale * 0.25;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = alpha;

  return marker;
}

void WorkspaceEditor::makeInteractive6DOFMarker(bool fixed, tf::Transform transform, const std::string& name,
                                                const std::string& description, float scale, bool object)
{
  InteractiveMarker marker;
  marker.header.frame_id = "/" + cm_->getWorldFrameId();
  marker.pose.position.x = transform.getOrigin().x();
  marker.pose.position.y = transform.getOrigin().y();
  marker.pose.position.z = transform.getOrigin().z();
  marker.pose.orientation.w = transform.getRotation().w();
  marker.pose.orientation.x = transform.getRotation().x();
  marker.pose.orientation.y = transform.getRotation().y();
  marker.pose.orientation.z = transform.getRotation().z();
  marker.scale = scale;
  marker.name = name;
  marker.description = description;

  if (!object)
  {
    makeInteractiveBoxControl(marker, 0.5f);
  }
  else
  {
    //ROS_INFO_STREAM("name: " << name);
    makeInteractiveCollisionObjectControl(marker);
  }

  InteractiveMarkerControl control;

  if (fixed)
  {
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.always_visible = false;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  interactive_marker_server_->insert(marker);

  control.interaction_mode = InteractiveMarkerControl::MENU;
  //control.markers.push_back(makeMarkerSphere(marker));
  marker.controls.push_back(control);

  if (!object)
  {
    menu_handler_map_["End Effector"].apply(*interactive_marker_server_, marker.name);
  }
  else
  {
    menu_handler_map_["Collision Object"].apply(*interactive_marker_server_, marker.name);
  }

  interactive_marker_server_->setCallback(marker.name, process_marker_feedback_ptr_);
}

/////
/// @brief Creates a clickable, box shaped marker.
/// @param msg the interactive marker to associate this box with.
/// @param alpha the transparency of the marker.
/// @return the control (which is a clickable box)
/////
InteractiveMarkerControl& WorkspaceEditor::makeInteractiveBoxControl(InteractiveMarker &msg, float alpha)
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeMarkerBox(msg, alpha));
  msg.controls.push_back(control);
  return msg.controls.back();
}

/////
/// @brief Creates a clickable, cylinder shaped marker.
/// @param msg the interactive marker to associate this cylinder with.
/// @param alpha the transparency of the marker.
/// @return the control (which is a clickable cylinder)
/////
InteractiveMarkerControl& WorkspaceEditor::makeInteractiveCylinderControl(InteractiveMarker &msg, float alpha)
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeMarkerCylinder(msg, alpha));
  msg.controls.push_back(control);
  return msg.controls.back();
}

void WorkspaceEditor::selectMarker(SelectableMarker& marker, tf::Transform transform)
{
  InteractiveMarker dummy;
  //ROS_INFO_STREAM(marker.controlName_);
  if (interactive_marker_server_->get(marker.controlName_, dummy))
  {
    //ROS_INFO_STREAM(marker.name_ +" a");
    dummy.header.stamp = ros::Time::now();
    interactive_marker_server_->setPose(marker.controlName_, toGeometryPose(transform), dummy.header);
  }
  else
  {
    //ROS_INFO_STREAM(marker.name_ +" b");
    if (!interactive_marker_server_->erase(marker.name_))
    {
      return;
    }

    //ROS_INFO_STREAM(marker.name_ << " c " << marker.type_);

    switch (marker.type_)
    {
      case EndEffectorControl:
        makeIKControllerMarker(transform, marker.controlName_, marker.controlDescription_, false, 0.5f);
        break;
      case CollisionObject:
        {
          double scale = std::max(marker.a_, std::max(marker.b_, marker.c_));
          makeInteractive6DOFMarker(false, transform, marker.controlName_, marker.controlDescription_, scale*2.0, true);
        }
        break;
      case JointControl:
        makeInteractive6DOFMarker(false, transform, marker.controlName_, marker.controlDescription_, 0.225f, false);
        break;
    }
  }
}

void WorkspaceEditor::deselectMarker(SelectableMarker& marker, tf::Transform transform)
{
  if (!interactive_marker_server_->erase(marker.controlName_))
  {
    ROS_ERROR_STREAM("Cannot erase marker " << marker.controlName_);
    return;
  }

  switch (marker.type_)
  {
    case EndEffectorControl:
      makeIKControllerMarker(transform, marker.controlName_, marker.controlDescription_, true, 0.5f);
      break;
    case CollisionObject:
      {
        makeSelectableCollisionObjectMarker(transform,
                                            marker.collision_object_type_,
                                            marker.controlName_,
                                            marker.color_,
                                            marker.a_, marker.b_, marker.c_);
      }
      break;
    case JointControl:
      makeSelectableMarker(marker.type_, transform, marker.controlName_, marker.controlDescription_, 0.225);
      break;
  }
}

void WorkspaceEditor::processIKControllerFeedback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  //ROS_INFO("%s %d", __FUNCTION__, feedback->event_type);
  PlanningGroupData& gc = group_map_[current_group_name_];
  switch (feedback->event_type)
  {
    case InteractiveMarkerFeedback::BUTTON_CLICK:
      break;
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

    case InteractiveMarkerFeedback::MOUSE_DOWN:
      {
        std::map<std::string, SelectableMarker>::iterator it = selectable_markers_.begin();
        while (it != selectable_markers_.end())
        {
          if ((it->first.rfind("object_") != string::npos) && (it->first != feedback->marker_name))
          {
            deselectMarker(it->second, toBulletTransform(it->second.pose_));
          }
          it++;
        }
      }
      break;
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

void WorkspaceEditor::processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  //ROS_INFO("%s %d", __FUNCTION__, feedback->event_type);
  PlanningGroupData& gc = group_map_[current_group_name_];
  switch (feedback->event_type)
  {

    case InteractiveMarkerFeedback::BUTTON_CLICK:
      {
        ROS_INFO_STREAM(feedback->marker_name);
        if (feedback->marker_name.rfind("_selectable") != string::npos)
        {
          tf::Transform cur = toBulletTransform(feedback->pose);
          if (feedback->marker_name.rfind("object_") != string::npos)
          {
            ROS_INFO_STREAM("select: " << feedback->marker_name << " " << selectable_markers_[feedback->marker_name].type_);
            selectMarker(selectable_markers_[feedback->marker_name], cur);


            //deselect other markers
            std::map<std::string, SelectableMarker>::iterator it = selectable_markers_.begin();
            while(it != selectable_markers_.end())
            {
              if((it->first.rfind("object_") != string::npos) && (it->first != feedback->marker_name))
              {
                deselectMarker(it->second, toBulletTransform(it->second.pose_));
              }
              it++;
            }

          }
        }
      }
      break;
    case InteractiveMarkerFeedback::MENU_SELECT:
      break;
    case InteractiveMarkerFeedback::MOUSE_UP:
      if (feedback->marker_name.rfind("object_") != string::npos
          && feedback->marker_name.rfind("_selectable") == string::npos)
      {
        ROS_INFO_STREAM("mouse up: " << feedback->marker_name);
        collision_objects_[feedback->marker_name].poses[0] = feedback->pose;
        selectable_markers_[feedback->marker_name + "_selectable"].pose_ = feedback->pose;
        refreshEnvironment();
      }
      break;
    case InteractiveMarkerFeedback::MOUSE_DOWN:
      break;
    case InteractiveMarkerFeedback::POSE_UPDATE:
      break;
    case InteractiveMarkerFeedback::KEEP_ALIVE:
      break;
  }
  interactive_marker_server_->applyChanges();
}

