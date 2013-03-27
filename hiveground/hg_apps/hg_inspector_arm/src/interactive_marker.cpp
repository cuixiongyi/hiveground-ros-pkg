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

  ROS_ASSERT(nh_private_.getParam("tool_frame", tool_frame_));
  ROS_INFO_STREAM("tool_frame: " << tool_frame_);

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
        //ROS_INFO_STREAM("click : " << feedback->marker_name);
        selectOnlyOneMarker(feedback->marker_name);
        Q_EMIT followPointSignal();
      }
      break;
    case InteractiveMarkerFeedback::MOUSE_DOWN:
      //ROS_INFO_STREAM("mouse down : " << feedback->marker_name);
      if(feedback->marker_name.rfind("marker_") != std::string::npos)
      {
        Q_EMIT inspectionPointClickedSignal(markers_[feedback->marker_name]);
      }
      break;
    case InteractiveMarkerFeedback::MOUSE_UP:
      break;
    case InteractiveMarkerFeedback::POSE_UPDATE:
      {
        //ROS_INFO_STREAM("pose: " << feedback->marker_name);
        if(feedback->marker_name.rfind("marker_ee") != std::string::npos)
        {
          //ROS_INFO("a");
          tf::Transform pose_new;
          tf::poseMsgToTF(feedback->pose, pose_new);
          if(pose_new == last_feedback_pose_)
            return;
          last_feedback_pose_ = pose_new;

          if(markers_[feedback->marker_name]->setPose(feedback->pose, true))
            markers_touched_ = true;
        }
        else if(feedback->marker_name.rfind("marker_tool") != std::string::npos)
        {
          //ROS_INFO("b");
          tf::Transform pose_tool;
          tf::poseMsgToTF(feedback->pose, pose_tool);
          if(pose_tool == last_feedback_pose_)
            return;
          last_feedback_pose_ = pose_tool;

          //ROS_INFO_STREAM("haha: " << feedback->pose);

          tf::StampedTransform tfs;
          listener_.lookupTransform(tool_frame_,
                                    ik_solver_info_.kinematic_solver_info.link_names[0],
                                    ros::Time(0), tfs);
          tf::Transform pose_ee = pose_tool * tfs;
          sensor_msgs::JointState joint_state = markers_[feedback->marker_name]->jointState();
          if(checkIKConstraintAware(pose_ee, joint_state))
          {
            markers_[feedback->marker_name]->setJointState(joint_state);
            markers_[feedback->marker_name]->setPose(feedback->pose, false);
            markers_touched_ = true;
          }
        }
      }
      break;
    case InteractiveMarkerFeedback::MENU_SELECT:

      if(feedback->marker_name.rfind("marker_") != std::string::npos)
      {
#if 0
        if(feedback->menu_entry_id == menu_entry_add_)
        {
          tf::StampedTransform transform;
          listener_.lookupTransform(world_frame_, ik_solver_info_.kinematic_solver_info.link_names[0], ros::Time(0), transform);
          geometry_msgs::Pose pose;
          tf::poseTFToMsg(transform, pose);
          std::string name = getMarkerName();
          addMarker(name, pose);
          InspectionPointItem* item = new InspectionPointItem(this, &marker_server_, pose);
          item->setJointState(markers_[item->name().toStdString()]->jointState());
          item->setName(name.c_str());
          markers_[item->name().toStdString()] = item;
          Q_EMIT inspectionPointClickedSignal(markers_[name]);
        }
        else
 #endif
        if(feedback->menu_entry_id == menu_entry_add_here_)
        {
          std::string name;
          if(feedback->marker_name.rfind("marker_ee") != std::string::npos)
          {
            name = getMarkerName("marker_ee");
          }
          else if(feedback->marker_name.rfind("marker_tool") != std::string::npos)
          {
            name = getMarkerName("marker_tool");
          }

          addMarker(name, feedback->pose, true, 0.05, ui.doubleSpinBoxDefaultMarkerScale->value());
          addInspectionPointFrom(name, markers_[feedback->marker_name]);

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
          if(feedback->marker_name.rfind("marker_ee") != std::string::npos)
          {

            ROS_DEBUG_STREAM("reset position:" << feedback->header << " " << feedback->pose);

            tf::StampedTransform ee_pose;
            listener_.lookupTransform(world_frame_,
                                        ik_solver_info_.kinematic_solver_info.link_names[0],
                                        ros::Time(0), ee_pose);
            geometry_msgs::Pose pose;
            tf::poseTFToMsg(ee_pose, pose);
            if(markers_[feedback->marker_name]->setPose(feedback->pose, true))
              markers_touched_ = true;
          }


/*
          marker_server_.setPose(feedback->marker_name, pose, feedback->header);
          marker_server_.applyChanges();
          markers_[feedback->marker_name]->setPose(pose, false);
          mutex_joint_state_.lock();
          markers_[feedback->marker_name]->setJointState(latest_joint_state_);
          mutex_joint_state_.unlock();
          Q_EMIT inspectionPointMovedSignal(markers_[feedback->marker_name]);
          markers_touched_ = true;
          */
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
          QList<QListWidgetItem*> items = ui.listWidgetMarker->findItems(feedback->marker_name.c_str(), Qt::MatchExactly);
          Q_FOREACH(QListWidgetItem* item, items)
          {
            ROS_INFO_STREAM(item->text().toStdString());
            ui.listWidgetMarker->takeItem(ui.listWidgetMarker->row(item));
            delete item;
          }
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
  if(joint_state.position.size() != 0)
  {
    gpik_req.ik_request.ik_seed_state.joint_state = latest_joint_state_;
    gpik_req.ik_request.ik_seed_state.joint_state.position = joint_state.position;
  }
  else
  {
    gpik_req.ik_request.ik_seed_state.joint_state = latest_joint_state_;
  }
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
  if(joint_state.position.size() != 0)
  {
    ik_request.ik_seed_state.joint_state = latest_joint_state_;
    ik_request.ik_seed_state.joint_state.position = joint_state.position;
  }
  else
  {
    ik_request.ik_seed_state.joint_state = latest_joint_state_;
  }
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

std::string InspectorArm::getMarkerName(const std::string& type)
{
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(5) << name_count_++ << "_" << type;
    return ss.str();
}


void InspectorArm::addMarker(const std::string& name,
                             geometry_msgs::Pose pose,
                             bool selectable,
                             double arrow_length,
                             double scale)
{
  InteractiveMarker int_marker;
  int_marker.name = name;
  int_marker.description = name;
  int_marker.header.frame_id = world_frame_;
  int_marker.scale = arrow_length * scale;
  int_marker.pose = pose;

  std::vector<Marker> markers;
  markers.push_back(makeBox(scale * 0.5 * arrow_length, 0.5, 0.5, 0.5, 0.8));
  markers.push_back(makeArrow(scale * arrow_length, 1.0, 0.0, 0.0, 0.8));

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
  std::string name = getMarkerName("marker_ee");
  addMarker(name, pose, true, 0.05, ui.doubleSpinBoxDefaultMarkerScale->value());

  addInspectionPoint(name, pose, latest_joint_state_);

  selectOnlyOneMarker(name);

  Q_EMIT inspectionPointClickedSignal(markers_[name]);
  markers_touched_ = true;
}

void InspectorArm::addMarkerAtTool()
{
  tf::StampedTransform transform;
  listener_.lookupTransform(world_frame_, tool_frame_, ros::Time(0), transform);
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(transform, pose);
  std::string name = getMarkerName("marker_tool");
  addMarker(name, pose, true, 0.05, ui.doubleSpinBoxDefaultMarkerScale->value());

  addInspectionPoint(name, pose, latest_joint_state_);

  selectOnlyOneMarker(name);

  Q_EMIT inspectionPointClickedSignal(markers_[name]);
  markers_touched_ = true;

}

void InspectorArm::addInspectionPoint(const std::string& name, const geometry_msgs::Pose& pose, const sensor_msgs::JointState& joint_state)
{
  InspectionPointItem* item = new InspectionPointItem(this, &marker_server_, pose);
  item->setName(name.c_str());
  item->setMarkerScale(ui.doubleSpinBoxDefaultMarkerScale->value());
  item->setJointState(joint_state);
  markers_[item->name().toStdString()] = item;

  ui.listWidgetMarker->addItem(item->name());
}

void InspectorArm::addInspectionPointFrom(const std::string& name, const InspectionPointItem* from)
{
  addInspectionPoint(name, from->pose(), from->jointState());
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

  ui.listWidgetMarker->clear();
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
  addMarker(name, markers_[name]->pose(), false, 0.05, markers_[name]->getMarkerScale());
  selected_markers_.push_back(name);
}

void InspectorArm::deselectMarker(const std::string& name)
{
  if (!marker_server_.erase(name))
  {
    ROS_ERROR_STREAM("Cannot erase " << name);
    return;
  }
  addMarker(name, markers_[name]->pose(), true, 0.05, markers_[name]->getMarkerScale());
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
  geometry_msgs::Pose pose = markers_[name]->pose();
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(q, pose.orientation);
  sensor_msgs::JointState joint_state;


  tf::Transform marker_pose;
  tf::poseMsgToTF(pose, marker_pose);
  tf::Transform pose_ee = marker_pose;
  if(name.rfind("marker_tool") != std::string::npos)
  {
    tf::StampedTransform tfs;
    listener_.lookupTransform(tool_frame_,
                              ik_solver_info_.kinematic_solver_info.link_names[0],
                              ros::Time(0), tfs);
    pose_ee = marker_pose * tfs;
  }


  if (checkIKConstraintAware(pose_ee, joint_state))
  {
    markers_[name]->setPose(pose, false);
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

tf::Transform  InspectorArm::moveSelectedMarker(const std::string& name, const tf::Vector3& rotation, const tf::Vector3& translation)
{
  tf::Vector3 linear = translation;
  tf::Vector3 angular = rotation;
  tf::Transform pose;
  tf::poseMsgToTF(markers_[name]->pose(), pose);


  double l1 = linear.length2();
  double l2 = angular.length2();

  if((l1 == 0.0) && (l2 == 0.0))
    return pose;

  bool translate = false;
  if(l1 > l2)
    translate = true;


  if(!ui.checkBoxAllTranslation->isChecked())
  {
    if(!ui.checkBoxEnableTranX->isChecked()) linear.setX(0);
    if(!ui.checkBoxEnableTranY->isChecked()) linear.setY(0);
    if(!ui.checkBoxEnableTranZ->isChecked()) linear.setZ(0);
  }

  if(!ui.checkBoxAllRotation->isChecked())
  {
    if(!ui.checkBoxEnableRotX->isChecked()) angular.setX(0);
    if(!ui.checkBoxEnableRotY->isChecked()) angular.setY(0);
    if(!ui.checkBoxEnableRotZ->isChecked()) angular.setZ(0);
  }

  tf::Quaternion q;
  if (ui.checkBoxSwapRxRz->isChecked())
    q.setRPY(angular.z(), angular.y(), angular.x());
  else
    q.setRPY(angular.x(), angular.y(), angular.z());


  if (ui.checkBoxEnableTranslation->isChecked())
  {
    if(!ui.checkBoxApproach->isChecked())
    {
      if (ui.checkBoxUseWorldCoordinate->isChecked())
      {
        pose.setOrigin(pose.getOrigin() + linear);
      }
      else
      {
        tf::Transform offset(tf::Quaternion(0, 0, 0, 1), linear);
        offset = pose * offset;
        pose = offset;
      }
    }
    else
    {
      tf::Transform offset(tf::Quaternion(0, 0, 0, 1), tf::Vector3(linear.getX(), 0, 0));
      offset = pose * offset;
      pose = offset;
    }
  }

  if (ui.checkBoxEnableRotation->isChecked() && !translate)
  {
    pose.setRotation(pose.getRotation() * q);
  }

  return pose;
}

void InspectorArm::makeMenu()
{
  menu_entry_maps_["Top Level"] = MenuEntryHandleMap();
  menu_entry_maps_["Inspection Point"] = MenuEntryHandleMap();


  // Allocate memory to the menu handlers
  menu_handler_map_["Top Level"];
  menu_handler_map_["Inspection Point"];

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

  int count = ui.listWidgetMarker->count();
  for (int i = 0; i < count; i++)
  {
    markers_[ui.listWidgetMarker->item(i)->text().toStdString()]->save(out);
  }

  /*
  std::map<std::string, InspectionPointItem*>::iterator it = markers_.begin();
  while (it != markers_.end())
  {
    it->second->save(out);
    it++;
  }
  */

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
  if (version < FILE_VERSION_MARKER)
  {
    ROS_ERROR("File version is too old");
    return;
  }
  if (version > FILE_VERSION_MARKER)
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
          InspectionPointItem* item = new InspectionPointItem(this, &marker_server_);
          item->load(in);
          markers_[item->name().toStdString()] = item;
          addMarker(item->name().toStdString(), item->pose(), true, 0.05, item->getMarkerScale());
          ui.listWidgetMarker->addItem(item->name());
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

