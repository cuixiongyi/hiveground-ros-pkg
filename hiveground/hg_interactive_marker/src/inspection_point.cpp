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

#include <hg_interactive_marker/inspection_point_marker_server.h>
#include <hg_interactive_marker/inspection_point.h>


using namespace hg_interactive_marker;
using namespace visualization_msgs;
using namespace interactive_markers;

InspectionPoint::InspectionPoint(InspectionPointMarkerServer* server,
                                      const std::string& name,
                                      const std::string& frame_id,
                                      const std::string& description,
                                      double arrow_length)
  : server_(server), name_(name), frame_id_(frame_id), description_(description)
{
  marker_callback_ptr_ = boost::bind(&InspectionPoint::processMarkerCallback, this, _1);
  makeMenu();

  InteractiveMarker int_marker;
  int_marker.name = name_;
  int_marker.description = description_;
  int_marker.header.frame_id = frame_id_;
  int_marker.scale = arrow_length * 0.5;

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

  server_->marker_server_.insert(int_marker);


  control.interaction_mode = InteractiveMarkerControl::MENU;
  int_marker.controls.push_back(control);
  menu_handler_map_["Inspection Point"].apply(server_->marker_server_, int_marker.name);


  server_->marker_server_.setCallback(int_marker.name, marker_callback_ptr_);

}

InspectionPoint::~InspectionPoint()
{

}


void InspectionPoint::processMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  ROS_INFO_STREAM(
      feedback->marker_name << " is now at " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z);
  ROS_INFO("event type %d", feedback->event_type);
  switch (feedback->event_type)
  {
    case InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO("%s %d" , feedback->marker_name.c_str(), feedback->menu_entry_id);
      if(feedback->menu_entry_id == menu_entry_check_ik_)
      {
        ROS_INFO("check ik");

        kinematics_msgs::GetPositionIK::Request ik_request;
        kinematics_msgs::GetPositionIK::Response ik_response;

        ik_request.ik_request.ik_link_name = server_->tip_link_map_[server_->group_name_];
        ik_request.ik_request.pose_stamped.header.frame_id = server_->collision_models_interface_->getWorldFrameId();
        ik_request.ik_request.pose_stamped.header.stamp = ros::Time::now();
        ik_request.ik_request.pose_stamped.pose = feedback->pose;

        //ik_request.ik_request.ik_seed_state.joint_state.position = server_->;
        //ik_request.ik_request.ik_seed_state.joint_state.name = robot_state.joint_state.name;
        ik_request.timeout = ros::Duration(5.0);

        ROS_DEBUG(
            "request pose: (%0.3f %0.3f %0.3f) (%0.3f %0.3f %0.3f %0.3f)",
            feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z,
            feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w);

        bool ik_service_call = server_->ik_none_collision_client_map_[server_->group_name_].call(ik_request, ik_response);
        if (!ik_service_call)
        {
          ROS_ERROR("IK service call failed!");
          //return 0;
        }

        if (ik_response.error_code.val == ik_response.error_code.SUCCESS)
        {
          //solution = ik_response.solution.joint_state.position;
          //for(int i = 0; i < (int)solution.size(); i++)
          //{
            //ROS_DEBUG("Joint %s solution angles [%d]: %f", robot_state.joint_state.name[i].c_str(), i, solution[i]);
         // }
          ROS_DEBUG("IK service call succeeded");
          //return 1;
        }
        else
        {
          ROS_DEBUG("IK service call error code: %d", ik_response.error_code.val);
        }
        //return 0;







      }
      else if(feedback->menu_entry_id == menu_entry_reset_position_)
      {
        ROS_INFO_STREAM("reset position:" << feedback->header << " " << feedback->pose);

        geometry_msgs::Pose pose = feedback->pose;
        pose.position = geometry_msgs::Point();
        server_->marker_server_.setPose(feedback->marker_name, pose, feedback->header);
        server_->marker_server_.applyChanges();
      }
      else if(feedback->menu_entry_id == menu_entry_reset_orientation_)
      {
        ROS_INFO("reset orientation");
        geometry_msgs::Pose pose = feedback->pose;
        pose.orientation = geometry_msgs::Quaternion();
        server_->marker_server_.setPose(feedback->marker_name, pose, feedback->header);
        server_->marker_server_.applyChanges();
      }



      break;
    default:
      break;
  }
}

Marker InspectionPoint::makeBox( InteractiveMarker &msg )
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

Marker InspectionPoint::makeArrow( InteractiveMarker &msg, double arrow_length)
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

InteractiveMarkerControl& InspectionPoint::makeArrowControl( InteractiveMarker &msg, double arrow_length)
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

void InspectionPoint::makeMenu()
{
  menu_entry_maps_["Inspection Point"] = MenuEntryHandleMap();

  // Allocate memory to the menu handlers
  menu_handler_map_["Inspection Point"];

  menu_entry_check_ik_ = registerMenuEntry(
      menu_handler_map_["Inspection Point"],
      menu_entry_maps_["Inspection Point"],
      "Check IK");

  menu_entry_reset_position_ = registerMenuEntry(
      menu_handler_map_["Inspection Point"],
      menu_entry_maps_["Inspection Point"],
      "Reset position");

  menu_entry_reset_orientation_ = registerMenuEntry(
      menu_handler_map_["Inspection Point"],
      menu_entry_maps_["Inspection Point"],
      "Reset orientation");
}

MenuHandler::EntryHandle InspectionPoint::registerMenuEntry(interactive_markers::MenuHandler& handler, MenuEntryHandleMap& map, std::string name)
{
  MenuHandler::EntryHandle toReturn = handler.insert(name, marker_callback_ptr_);
  map[toReturn] = name;
  return toReturn;
}
