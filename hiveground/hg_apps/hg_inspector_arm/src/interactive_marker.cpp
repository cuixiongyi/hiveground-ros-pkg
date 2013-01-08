/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Imai Laboratory, Keio University.
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

#include <hg_inspector_arm/inspector_arm.h>

using namespace visualization_msgs;
using namespace interactive_markers;

bool InspectorArm::initializeInteractiveMarkerServer()
{
  name_count_ = 0;
  marker_callback_ptr_ = boost::bind(&InspectorArm::processMarkerCallback, this, _1);

  ros::service::waitForService("get_ik_solver_info");
  ros::service::waitForService("get_ik");

  ROS_ASSERT(nh_private_.getParam("world_frame", world_frame_));
  ROS_INFO_STREAM("world_frame: " << world_frame_);

  ROS_ASSERT(nh_private_.getParam("base_link", base_link_));
  ROS_INFO_STREAM("base_link: " << base_link_);

  ik_query_client_ = nh_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("get_ik_solver_info");
  ik_client_ = nh_.serviceClient<kinematics_msgs::GetPositionIK>("get_ik");

  kinematics_msgs::GetKinematicSolverInfo::Request request;

  if (ik_query_client_.call(request, response_))
  {
    for (unsigned int i = 0; i < response_.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_INFO("Joint: %d %s", i, response_.kinematic_solver_info.joint_names[i].c_str());
    }

    for (unsigned int i = 0; i < response_.kinematic_solver_info.link_names.size(); i++)
    {
      ROS_INFO("Link: %d %s", i, response_.kinematic_solver_info.link_names[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("Could not call query service");
    return false;
  }

  makeMenu();
  return true;
}


void InspectorArm::addMarker(const std::string& name, geometry_msgs::Pose pose, double arrow_length)
{
  InteractiveMarker int_marker;
  std::stringstream ss;
  ss << "marker_" << name_count_++ << "_" << name;
  ROS_INFO_STREAM("Added " << ss.str());
  int_marker.name = ss.str();
  int_marker.header.frame_id = world_frame_;
  int_marker.scale = arrow_length * 0.5;
  int_marker.pose = pose;

  makeArrowControl(int_marker, arrow_length);

  // create a non-interactive control which contains the arrow
  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  marker_server_.insert(int_marker);

  control.interaction_mode = InteractiveMarkerControl::MENU;
  int_marker.controls.push_back(control);
  menu_handler_map_["Inspection Point"].apply(marker_server_, int_marker.name);

  markers_[int_marker.name] = new InspectionPointItem(&marker_server_, int_marker.pose);
  markers_[int_marker.name]->setName(int_marker.name.c_str());

  marker_server_.setCallback(int_marker.name, marker_callback_ptr_);
  marker_server_.applyChanges();

  Q_EMIT inspectionPointClickedSignal(markers_[int_marker.name]);
}

void InspectorArm::processMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  switch (feedback->event_type)
  {
    case InteractiveMarkerFeedback::MOUSE_DOWN:
      Q_EMIT inspectionPointClickedSignal(markers_[feedback->marker_name]);
      break;
    case InteractiveMarkerFeedback::POSE_UPDATE:
      if(checkIK(feedback))
      {
        markers_[feedback->marker_name]->setPose(feedback->pose);
        Q_EMIT inspectionPointMovedSignal(markers_[feedback->marker_name]);
      }
      else
      {
        if(markers_[feedback->marker_name])
        {
          marker_server_.setPose(feedback->marker_name, markers_[feedback->marker_name]->pose());
          marker_server_.applyChanges();
        }
      }
      break;
    case InteractiveMarkerFeedback::MENU_SELECT:
      if(feedback->marker_name.rfind("marker_") != std::string::npos)
      {
        if(feedback->menu_entry_id == menu_entry_add_)
        {
          tf::StampedTransform transform;
          listener_.lookupTransform(world_frame_, response_.kinematic_solver_info.link_names[0], ros::Time(0), transform);
          geometry_msgs::Pose pose;
          tf::poseTFToMsg(transform, pose);
          addMarker("default", pose);
        }
        else if(feedback->menu_entry_id == menu_entry_add_here_)
        {
          addMarker("default", feedback->pose);
        }
        else if(feedback->menu_entry_id == menu_entry_reset_position_)
        {
          ROS_DEBUG_STREAM("reset position:" << feedback->header << " " << feedback->pose);
          geometry_msgs::Pose pose = feedback->pose;
          pose.position = geometry_msgs::Point();
          marker_server_.setPose(feedback->marker_name, pose, feedback->header);
          marker_server_.applyChanges();
          markers_[feedback->marker_name]->setPose(pose);
          Q_EMIT inspectionPointMovedSignal(markers_[feedback->marker_name]);
        }
        else if (feedback->menu_entry_id == menu_entry_reset_orientation_)
        {
          ROS_DEBUG("reset orientation");
          geometry_msgs::Pose pose = feedback->pose;
          pose.orientation = geometry_msgs::Quaternion();
          marker_server_.setPose(feedback->marker_name, pose, feedback->header);
          marker_server_.applyChanges();
          markers_[feedback->marker_name]->setPose(pose);
          Q_EMIT inspectionPointMovedSignal(markers_[feedback->marker_name]);
        }
        else if(feedback->menu_entry_id == menu_entry_remove_)
        {
          ROS_DEBUG_STREAM("remove " << feedback->marker_name);
          delete markers_[feedback->marker_name];
          markers_.erase(feedback->marker_name);
          marker_server_.erase(feedback->marker_name);
          marker_server_.applyChanges();
          Q_EMIT inspectionPointClickedSignal(0);
        }
      }
      else if(feedback->marker_name == "top_level")
      {
        if(feedback->menu_entry_id == menu_entry_top_add_)
        {
          ROS_DEBUG("top add");
          tf::StampedTransform transform;
          listener_.lookupTransform(world_frame_, response_.kinematic_solver_info.link_names[0], ros::Time(0), transform);
          geometry_msgs::Pose pose;
          tf::poseTFToMsg(transform, pose);
          addMarker("default", pose);
        }
        else if(feedback->menu_entry_id == menu_entry_top_clear_)
        {
          ROS_DEBUG("clear");
          std::map<std::string, InspectionPointItem*>::iterator it = markers_.begin();
          while(it != markers_.end())
          {
            delete it->second;
            marker_server_.erase(it->first);
            it++;
          }
          markers_.clear();
          marker_server_.applyChanges();
          Q_EMIT inspectionPointClickedSignal(0);
        }
      }
      break;
    default:
      break;
  }
}

bool InspectorArm::checkIK(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  ROS_DEBUG_STREAM("check IK: " << feedback->pose);

  kinematics_msgs::GetPositionIK::Request gpik_req;
  kinematics_msgs::GetPositionIK::Response gpik_res;
  gpik_req.timeout = ros::Duration(5.0);
  gpik_req.ik_request.ik_link_name = response_.kinematic_solver_info.link_names[0];
  gpik_req.ik_request.pose_stamped.header.frame_id = base_link_;

  geometry_msgs::Pose transformed_pose;
  if(world_frame_ != base_link_)
  {
    tf::StampedTransform transform;
    try
    {
      listener_.lookupTransform(world_frame_, base_link_, ros::Time(0), transform);
      tf::Transform tf;
      tf::poseMsgToTF(feedback->pose, tf);
      tf = transform * tf;
      tf::poseTFToMsg(tf, transformed_pose);
    }
    catch (const tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }
  else
  {
    transformed_pose = feedback->pose;
  }

  gpik_req.ik_request.pose_stamped.pose = transformed_pose;

  gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response_.kinematic_solver_info.joint_names.size());
  gpik_req.ik_request.ik_seed_state.joint_state.name = response_.kinematic_solver_info.joint_names;

  for(unsigned int i=0; i< response_.kinematic_solver_info.joint_names.size(); i++)
  {
    gpik_req.ik_request.ik_seed_state.joint_state.position[i] =
        (response_.kinematic_solver_info.limits[i].min_position +
         response_.kinematic_solver_info.limits[i].max_position) / 2.0;
  }

  if(ik_client_.call(gpik_req, gpik_res))
  {
    if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
    {
      for(unsigned int i=0; i < gpik_res.solution.joint_state.name.size(); i ++)
        ROS_DEBUG("Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_state.position[i]);
    }
    else
    {
      ROS_ERROR("Inverse kinematics failed");
      return false;
    }
  }
  else
  {
    ROS_ERROR("Inverse kinematics service call failed");
    return false;
  }
  return true;

}

Marker InspectorArm::makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0.8;

  return marker;
}

Marker InspectorArm::makeArrow( InteractiveMarker &msg, double arrow_length)
{
  Marker marker;

  marker.type = Marker::ARROW;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = arrow_length;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& InspectorArm::makeArrowControl( InteractiveMarker &msg, double arrow_length)
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.independent_marker_orientation = true;
  control.markers.push_back( makeBox(msg));
  control.markers.push_back(makeArrow(msg, arrow_length));
  msg.controls.push_back( control );

  return msg.controls.back();
}

void InspectorArm::makeMenu()
{
  menu_entry_maps_["Top Level"] = MenuEntryHandleMap();
  menu_entry_maps_["Inspection Point"] = MenuEntryHandleMap();


  // Allocate memory to the menu handlers
  menu_handler_map_["Top Level"];
  menu_handler_map_["Inspection Point"];

  menu_entry_add_ = registerMenuEntry(
      menu_handler_map_["Inspection Point"],
      menu_entry_maps_["Inspection Point"],
      "Add");

  menu_entry_add_here_ = registerMenuEntry(
      menu_handler_map_["Inspection Point"],
      menu_entry_maps_["Inspection Point"],
      "Add Here");

  menu_entry_reset_position_ = registerMenuEntry(
      menu_handler_map_["Inspection Point"],
      menu_entry_maps_["Inspection Point"],
      "Reset position");

  menu_entry_reset_orientation_ = registerMenuEntry(
      menu_handler_map_["Inspection Point"],
      menu_entry_maps_["Inspection Point"],
      "Reset orientation");

  menu_entry_remove_ = registerMenuEntry(
      menu_handler_map_["Inspection Point"],
      menu_entry_maps_["Inspection Point"],
      "Remove");

  menu_entry_top_add_ = registerMenuEntry(menu_handler_map_["Top Level"], menu_entry_maps_["Top Level"], "Add");
  menu_entry_top_clear_ = registerMenuEntry(menu_handler_map_["Top Level"], menu_entry_maps_["Top Level"], "Clear");


  InteractiveMarker int_marker;
  int_marker.pose.position.z = 2.0;
  int_marker.name = "top_level";
  int_marker.description = "Inspection Marker Server";
  int_marker.header.frame_id = world_frame_;

  InteractiveMarkerControl control;
  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.always_visible = true;

  Marker labelMarker;
  labelMarker.type = Marker::TEXT_VIEW_FACING;
  labelMarker.text = "Inspection Marker Command...";
  labelMarker.color.r = 1.0;
  labelMarker.color.g = 1.0;
  labelMarker.color.b = 1.0;
  labelMarker.color.a = 1.0;
  labelMarker.scale.x = 0.5;
  labelMarker.scale.y = 0.2;
  labelMarker.scale.z = 0.1;
  control.markers.push_back(labelMarker);

  int_marker.controls.push_back(control);

  marker_server_.insert(int_marker, marker_callback_ptr_);
  menu_handler_map_["Top Level"].apply(marker_server_, int_marker.name);
  marker_server_.applyChanges();
}

MenuHandler::EntryHandle InspectorArm::registerMenuEntry(interactive_markers::MenuHandler& handler, MenuEntryHandleMap& map, std::string name)
{
  MenuHandler::EntryHandle toReturn = handler.insert(name, marker_callback_ptr_);
  map[toReturn] = name;
  return toReturn;
}

