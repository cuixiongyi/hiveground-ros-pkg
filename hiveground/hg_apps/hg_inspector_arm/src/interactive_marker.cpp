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
#include <hg_inspector_arm/inspection_point.h>
#include <qfile.h>
#include <algorithm>

using namespace visualization_msgs;
using namespace interactive_markers;

bool InspectorArm::initializeInteractiveMarkerServer()
{
  name_count_ = 0;
  marker_callback_ptr_ = boost::bind(&InspectorArm::processMarkerCallback, this, _1);

  //ros::service::waitForService("get_ik_solver_info");
  //ros::service::waitForService("get_ik");

  ROS_ASSERT(nh_private_.getParam("world_frame", world_frame_));
  ROS_INFO_STREAM("world_frame: " << world_frame_);

  ROS_ASSERT(nh_private_.getParam("base_link", base_link_));
  ROS_INFO_STREAM("base_link: " << base_link_);

  //ik_query_client_ = nh_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("get_ik_solver_info");
  //ik_client_ = nh_.serviceClient<kinematics_msgs::GetPositionIK>("get_ik");

  kinematics_msgs::GetKinematicSolverInfo::Request request;

  if (ik_info_client_map_["manipulator"].call(request, ik_solver_info_))
  {
    for (unsigned int i = 0; i < ik_solver_info_.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_INFO("Joint: %d %s", i, ik_solver_info_.kinematic_solver_info.joint_names[i].c_str());
    }

    for (unsigned int i = 0; i < ik_solver_info_.kinematic_solver_info.link_names.size(); i++)
    {
      ROS_INFO("Link: %d %s", i, ik_solver_info_.kinematic_solver_info.link_names[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("Could not call query service");
    return false;
  }

  makeMenu();

  markers_touched_ = false;

  return true;
}

void InspectorArm::processMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  switch (feedback->event_type)
  {
    case InteractiveMarkerFeedback::BUTTON_CLICK:
      {
        selectOnlyOneMarker(feedback->marker_name);
        Q_EMIT followPointSignal();
      }
      break;
    case InteractiveMarkerFeedback::MOUSE_DOWN:
      if(feedback->marker_name.rfind("marker_") != std::string::npos)
      {
        Q_EMIT inspectionPointClickedSignal(markers_[feedback->marker_name]);
      }
      break;
    case InteractiveMarkerFeedback::MOUSE_UP:
      if (feedback->marker_name.rfind("marker_") != std::string::npos)
      {
        markers_[feedback->marker_name]->setPose(markers_[feedback->marker_name]->pose());
        Q_EMIT inspectionPointMovedSignal(markers_[feedback->marker_name]);
        markers_touched_ = true;
      }
      break;
    case InteractiveMarkerFeedback::POSE_UPDATE:
      {
        sensor_msgs::JointState joint_state;
        if(checkIKConstraintAware(feedback->pose, joint_state))
        {
          tf::Transform tf_old, tf_new;
          tf::poseMsgToTF(markers_[feedback->marker_name]->pose(), tf_old);
          tf::poseMsgToTF(feedback->pose, tf_new);
          if(tf_old == tf_new)
            return;
          markers_[feedback->marker_name]->setPose(feedback->pose);
          markers_[feedback->marker_name]->setJointState(joint_state);
          Q_EMIT inspectionPointMovedSignal(markers_[feedback->marker_name]);
          Q_EMIT followPointSignal();
          markers_touched_ = true;
        }
      }
      break;
    case InteractiveMarkerFeedback::MENU_SELECT:

      if(feedback->marker_name.rfind("marker_") != std::string::npos)
      {
        if(feedback->menu_entry_id == menu_entry_add_)
        {
          tf::StampedTransform transform;
          listener_.lookupTransform(world_frame_, ik_solver_info_.kinematic_solver_info.link_names[0], ros::Time(0), transform);
          geometry_msgs::Pose pose;
          tf::poseTFToMsg(transform, pose);
          std::string name = getMarkerName();
          addMarker(name, pose);
          InspectionPointItem* item = new InspectionPointItem(&marker_server_, pose);
          item->setName(name.c_str());
          markers_[item->name().toStdString()] = item;
          Q_EMIT inspectionPointClickedSignal(markers_[name]);
        }
        else if(feedback->menu_entry_id == menu_entry_add_here_)
        {
          std::string name = getMarkerName();
          addMarker(name, feedback->pose);
          InspectionPointItem* item = new InspectionPointItem(&marker_server_, feedback->pose);
          item->setName(name.c_str());
          item->setJointState(markers_[feedback->marker_name]->jointState());
          markers_[item->name().toStdString()] = item;
          selectOnlyOneMarker(name);
          Q_EMIT inspectionPointClickedSignal(markers_[name]);
        }
        else if(feedback->menu_entry_id == menu_entry_point_x_plus_)
        {
          setMarkerOrientation(feedback->marker_name, 0, 0, 0);
        }
        else if(feedback->menu_entry_id == menu_entry_point_x_minus_)
        {
          setMarkerOrientation(feedback->marker_name, 0, -M_PI, 0);
        }
        else if(feedback->menu_entry_id == menu_entry_point_y_plus_)
        {
          setMarkerOrientation(feedback->marker_name, 0, 0, M_PI_2);
        }
        else if(feedback->menu_entry_id == menu_entry_point_y_minus_)
        {
          setMarkerOrientation(feedback->marker_name, 0, 0, -M_PI_2);
        }
        else if(feedback->menu_entry_id == menu_entry_point_z_plus_)
        {
          setMarkerOrientation(feedback->marker_name, 0, -M_PI_2, 0);
        }
        else if(feedback->menu_entry_id == menu_entry_point_z_minus_)
        {
          setMarkerOrientation(feedback->marker_name, 0, M_PI_2, 0);
        }
        else if(feedback->menu_entry_id == menu_entry_reset_position_)
        {
          ROS_DEBUG_STREAM("reset position:" << feedback->header << " " << feedback->pose);

          tf::StampedTransform ee_pose;
          listener_.lookupTransform(world_frame_,
                                      ik_solver_info_.kinematic_solver_info.link_names[0],
                                      ros::Time(0), ee_pose);
          geometry_msgs::Pose pose;
          tf::poseTFToMsg(ee_pose, pose);
          marker_server_.setPose(feedback->marker_name, pose, feedback->header);
          marker_server_.applyChanges();
          markers_[feedback->marker_name]->setPose(pose);
          mutex_joint_state_.lock();
          markers_[feedback->marker_name]->setJointState(latest_joint_state_);
          mutex_joint_state_.unlock();
          Q_EMIT inspectionPointMovedSignal(markers_[feedback->marker_name]);
          markers_touched_ = true;
        }
        else if (feedback->menu_entry_id == menu_entry_reset_orientation_)
        {
          setMarkerOrientation(feedback->marker_name, 0, 0, 0);
        }
        else if(feedback->menu_entry_id == menu_entry_remove_)
        {
          ROS_DEBUG_STREAM("remove " << feedback->marker_name);
          delete markers_[feedback->marker_name];
          markers_.erase(feedback->marker_name);
          marker_server_.erase(feedback->marker_name);
          marker_server_.applyChanges();
          Q_EMIT inspectionPointClickedSignal(0);
          markers_touched_ = true;
        }
      }
      else if(feedback->marker_name == "top_level")
      {
        if(feedback->menu_entry_id == menu_entry_top_add_)
        {
          addMarkerAtEndEffector();
        }
        else if(feedback->menu_entry_id == menu_entry_top_clear_)
        {
          clearMarker();
        }
      }
      break;
    default:
      break;
  }
}

bool InspectorArm::checkIK(const geometry_msgs::Pose& pose, sensor_msgs::JointState& joint_state)
{
  tf::Transform tf;
  tf::poseMsgToTF(pose, tf);
  return checkIK(tf, joint_state);
}

bool InspectorArm::checkIK(const tf::Transform& pose, sensor_msgs::JointState& joint_state)
{
  kinematics_msgs::GetPositionIK::Request gpik_req;
  kinematics_msgs::GetPositionIK::Response gpik_res;
  gpik_req.timeout = ros::Duration(0.2);
  gpik_req.ik_request.ik_link_name = ik_solver_info_.kinematic_solver_info.link_names[0];
  gpik_req.ik_request.pose_stamped.header.frame_id = base_link_;

  if (world_frame_ != base_link_)
  {
    tf::StampedTransform transform;
    try
    {
      listener_.lookupTransform(world_frame_, base_link_, ros::Time(0), transform);
      tf::Transform tf = transform * pose;
      tf::poseTFToMsg(tf, gpik_req.ik_request.pose_stamped.pose);
    }
    catch (const tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }
  else
  {
    tf::poseTFToMsg(pose, gpik_req.ik_request.pose_stamped.pose);
  }

  mutex_joint_state_.lock();
  gpik_req.ik_request.ik_seed_state.joint_state = latest_joint_state_;
  mutex_joint_state_.unlock();
  gpik_req.ik_request.robot_state = gpik_req.ik_request.ik_seed_state;

  if (ik_none_collision_client_map_["manipulator"].call(gpik_req, gpik_res))
  {
    if (gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
    {
      joint_state = gpik_res.solution.joint_state;
    }
    else
    {
      ROS_WARN_THROTTLE(1.0, "Inverse kinematics failed");
      return false;
    }
  }
  else
  {
    ROS_WARN_THROTTLE(1.0, "Inverse kinematics service call failed");
    return false;
  }
  return true;
}

bool InspectorArm::checkIKConstraintAware(const geometry_msgs::Pose& pose, sensor_msgs::JointState& joint_state)
{
  tf::Transform tf;
  tf::poseMsgToTF(pose, tf);
  return checkIKConstraintAware(tf, joint_state);
}

bool InspectorArm::checkIKConstraintAware(const tf::Transform& pose, sensor_msgs::JointState& joint_state)
{
  kinematics_msgs::PositionIKRequest ik_request;
  ik_request.ik_link_name = ik_solver_info_.kinematic_solver_info.link_names[0];
  ik_request.pose_stamped.header.frame_id =   base_link_;
  ik_request.pose_stamped.header.stamp = ros::Time::now();
  if (world_frame_ != base_link_)
    {
      tf::StampedTransform transform;
      try
      {
        listener_.lookupTransform(world_frame_, base_link_, ros::Time(0), transform);
        tf::Transform tf = transform * pose;
        tf::poseTFToMsg(tf, ik_request.pose_stamped.pose);
      }
      catch (const tf::TransformException& ex)
      {
        ROS_ERROR("%s", ex.what());
      }
    }
    else
    {
      tf::poseTFToMsg(pose, ik_request.pose_stamped.pose);
    }

  mutex_joint_state_.lock();
  ik_request.ik_seed_state.joint_state = latest_joint_state_;
  mutex_joint_state_.unlock();
  ik_request.robot_state = ik_request.ik_seed_state;

  kinematics_msgs::GetConstraintAwarePositionIK::Request ik_req;
  kinematics_msgs::GetConstraintAwarePositionIK::Response ik_res;
  ik_req.ik_request = ik_request;
  ik_req.timeout = ros::Duration(0.2);

  if (ik_client_map_["manipulator"].call(ik_req, ik_res))
  {
    if (ik_res.error_code.val == ik_res.error_code.SUCCESS)
    {
      joint_state = ik_res.solution.joint_state;
    }
    else
    {
      ROS_WARN_THROTTLE(1.0, "Inverse kinematics failed");
      return false;
    }
  }
  else
  {
    ROS_WARN_THROTTLE(1.0, "Inverse kinematics service call failed");
    return false;
  }
  return true;
}

std::string InspectorArm::getMarkerName()
{
    std::stringstream ss;
    ss << "marker_" << std::setfill('0') << std::setw(5) << name_count_++;
    return ss.str();
}


void InspectorArm::addMarker(const std::string& name,
                                 geometry_msgs::Pose pose,
                                 bool selectable,
                                 double arrow_length)
{
  InteractiveMarker int_marker;
  int_marker.name = name;
  int_marker.description = name;
  int_marker.header.frame_id = world_frame_;
  int_marker.scale = arrow_length;
  int_marker.pose = pose;

  std::vector<Marker> markers;
  markers.push_back(makeBox(0.5 * arrow_length, 0.5, 0.5, 0.5));
  markers.push_back(makeArrow(arrow_length));

  if(selectable)
  {
    makeSelectableControl(int_marker, markers);
  }
  else
  {
    makeFreeMoveControl(int_marker, markers);
    make6DOFControl(int_marker, markers);
  }



  marker_server_.insert(int_marker);
  InteractiveMarkerControl control;
  control.interaction_mode = InteractiveMarkerControl::MENU;
  int_marker.controls.push_back(control);
  menu_handler_map_["Inspection Point"].apply(marker_server_, int_marker.name);

  marker_server_.setCallback(int_marker.name, marker_callback_ptr_);
  marker_server_.applyChanges();
}

void InspectorArm::addMarkerAtEndEffector()
{
  tf::StampedTransform transform;
  listener_.lookupTransform(world_frame_, ik_solver_info_.kinematic_solver_info.link_names[0], ros::Time(0), transform);
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(transform, pose);
  std::string name = getMarkerName();
  addMarker(name, pose);
  InspectionPointItem* item = new InspectionPointItem(&marker_server_, pose);
  item->setName(name.c_str());
  mutex_joint_state_.lock();
  item->setJointState(latest_joint_state_);
  mutex_joint_state_.unlock();

  markers_[item->name().toStdString()] = item;
  selectOnlyOneMarker(name);
  Q_EMIT inspectionPointClickedSignal(markers_[name]);
  markers_touched_ = true;
}

void InspectorArm::clearMarker()
{
  std::map<std::string, InspectionPointItem*>::iterator it = markers_.begin();
  while (it != markers_.end())
  {
    delete it->second;
    marker_server_.erase(it->first);
    it++;
  }
  markers_.clear();
  marker_server_.applyChanges();
  Q_EMIT inspectionPointClickedSignal(0);
  markers_touched_ = true;
}

Marker InspectorArm::makeBox(double size, double r, double g, double b, double a)
{
  Marker marker;
  marker.type = Marker::CUBE;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  return marker;
}

Marker InspectorArm::makeArrow(double size, double r, double g, double b, double a)
{
  Marker marker;
  marker.type = Marker::ARROW;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = size;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  return marker;
}

Marker InspectorArm::makeSphere(double size, double r, double g, double b, double a)
{
  Marker marker;

  marker.type = Marker::ARROW;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;
  return marker;
}

InteractiveMarkerControl& InspectorArm::makeFreeMoveControl(visualization_msgs::InteractiveMarker &msg,
                                                 std::vector<visualization_msgs::Marker>& markers)
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.independent_marker_orientation = true;
  for(size_t i = 0; i < markers.size(); i++)
  {
    control.markers.push_back(markers[i]);
  }
  msg.controls.push_back(control);
  return msg.controls.back();
}

InteractiveMarkerControl& InspectorArm::make6DOFControl(visualization_msgs::InteractiveMarker &msg,
                                                            std::vector<visualization_msgs::Marker>& markers)
{
  InteractiveMarkerControl control;
  // create a non-interactive control which contains the arrow
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control);
  return msg.controls.back();
}

InteractiveMarkerControl& InspectorArm::makeSelectableControl(visualization_msgs::InteractiveMarker &msg,
                                                                   std::vector<visualization_msgs::Marker>& markers)
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  for(size_t i = 0; i < markers.size(); i++)
  {
    control.markers.push_back(markers[i]);
  }
  msg.controls.push_back(control);
  return msg.controls.back();
}


void InspectorArm::selectMarker(const std::string& name)
{
  if (!marker_server_.erase(name))
  {
    ROS_ERROR_STREAM("Cannot erase " << name);
    return;
  }
  addMarker(name, markers_[name]->pose(), false);
  selected_markers_.push_back(name);
}

void InspectorArm::deselectMarker(const std::string& name)
{
  if (!marker_server_.erase(name))
  {
    ROS_ERROR_STREAM("Cannot erase " << name);
    return;
  }
  addMarker(name, markers_[name]->pose());
  selected_markers_.remove(name);
}

void InspectorArm::selectOnlyOneMarker(const std::string& name)
{
  std::map<std::string, InspectionPointItem*>::iterator it = markers_.begin();
  while (it != markers_.end())
  {
    if (std::find(selected_markers_.begin(), selected_markers_.end(), it->first) != selected_markers_.end())
      deselectMarker(it->first);
    it++;
  }
  selectMarker(name);
}

bool InspectorArm::setMarkerOrientation(const std::string& name, double roll, double pitch, double yaw)
{
  ROS_DEBUG("reset orientation");
  geometry_msgs::Pose pose = markers_[name]->pose();
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(q, pose.orientation);
  sensor_msgs::JointState joint_state;
  if (checkIKConstraintAware(pose, joint_state))
  {
    markers_[name]->setPose(pose);
    markers_[name]->setJointState(joint_state);
    marker_server_.setPose(name, pose);
    marker_server_.applyChanges();
    Q_EMIT inspectionPointMovedSignal(markers_[name]);
    Q_EMIT followPointSignal();
    markers_touched_ = true;
    return true;
  }
  return false;
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

  menu_entry_point_x_plus_ = registerMenuEntry(
      menu_handler_map_["Inspection Point"],
      menu_entry_maps_["Inspection Point"],
      "Point +X");
  menu_entry_point_x_minus_ = registerMenuEntry(
      menu_handler_map_["Inspection Point"],
      menu_entry_maps_["Inspection Point"],
      "Point -X");

  menu_entry_point_y_plus_ = registerMenuEntry(
      menu_handler_map_["Inspection Point"],
      menu_entry_maps_["Inspection Point"],
      "Point +Y");
  menu_entry_point_y_minus_ = registerMenuEntry(
      menu_handler_map_["Inspection Point"],
      menu_entry_maps_["Inspection Point"],
      "Point -Y");

  menu_entry_point_z_plus_ = registerMenuEntry(
      menu_handler_map_["Inspection Point"],
      menu_entry_maps_["Inspection Point"],
      "Point +Z");
  menu_entry_point_z_minus_ = registerMenuEntry(
      menu_handler_map_["Inspection Point"],
      menu_entry_maps_["Inspection Point"],
      "Point -Z");


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

void InspectorArm::saveMarker()
{
  QFile file(markers_save_file_name_);
  file.open(QIODevice::WriteOnly);
  if(!file.isOpen())
  {
    ROS_ERROR("Cannot open %s for writing", markers_save_file_name_.toStdString().c_str());
    return;
  }
  QDataStream out(&file);

  out << (quint32) FILE_MAGIC_MARKER;
  out << (quint32) FILE_VERSION_MARKER;
  out << name_count_;

  std::map<std::string, InspectionPointItem*>::iterator it = markers_.begin();
  while (it != markers_.end())
  {
    it->second->save(out);
    it++;
  }
  file.close();
  markers_touched_  = false;
}

void InspectorArm::loadMarker()
{
  QFile file(markers_save_file_name_);
  file.open(QIODevice::ReadOnly);
  if (!file.isOpen())
  {
    ROS_ERROR("Cannot open %s for reading", markers_save_file_name_.toStdString().c_str());
    return;
  }
  QDataStream in(&file);

  // Read and check the header
  quint32 magic;
  in >> magic;
  if(magic != FILE_MAGIC_MARKER)
  {
    ROS_ERROR("Bad file format");
    return;
  }

  // Read the version
  qint32 version;
  in >> version;
  if (version < 100)
  {
    ROS_ERROR("File version is too old");
    return;
  }
  if (version > 100)
  {
    ROS_ERROR("File version is too new");
    return;
  }

  clearMarker();

  in >> name_count_;

  int rtti;
  while(!in.atEnd())
  {
    in >> rtti;
    ROS_INFO("Marker rtti %d", rtti);
    switch(rtti)
    {
      case InspectionPointItem::Rtti_Item:
        {
          InspectionPointItem* item = new InspectionPointItem(&marker_server_);
          item->load(in);
          markers_[item->name().toStdString()] = item;
          addMarker(item->name().toStdString(), item->pose());
          Q_EMIT inspectionPointClickedSignal(item);
        }
        break;
      case InspectionPointItem::Rtti_LookAt:
        {
        }
        break;
      default:
        ROS_ERROR("Unknown InspectionPointItem::Rtti %d", rtti);
        break;
    }
  }
}

