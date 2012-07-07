/*
 * prw_marker.cpp
 *
 *  Created on: Jul 7, 2012
 *      Author: mahisorn
 */

geometry_msgs::Pose PRW::toGeometryPose(tf::Transform transform)
{
  geometry_msgs::Pose toReturn;
  toReturn.position.x = transform.getOrigin().x();
  toReturn.position.y = transform.getOrigin().y();
  toReturn.position.z = transform.getOrigin().z();
  toReturn.orientation.x = transform.getRotation().x();
  toReturn.orientation.y = transform.getRotation().y();
  toReturn.orientation.z = transform.getRotation().z();
  toReturn.orientation.w = transform.getRotation().w();
  return toReturn;
}

tf::Transform PRW::toBulletTransform(geometry_msgs::Pose pose)
{
  tf::Quaternion quat = tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf::Vector3 vec = tf::Vector3(pose.position.x, pose.position.y, pose.position.z);
  return tf::Transform(quat, vec);
}

////
/// @brief Returns true if a selectable marker of the given name exists in the marker map.
/// @param name the unique identifier of the marker.
/// @returns true if the marker exists, or false otherwise.
////
bool PRW::selectableMarkerExists(std::string name)
{
  return selectable_markers_.find(name) != selectable_markers_.end();
}

/////
/// @brief Removes the menu marker given and replaces it with a 6DOF marker
/// @param marker a reference to the selectablemarker struct.
/// @param transform location to select the marker.
/////
void PRW::selectMarker(SelectableMarker& marker, tf::Transform transform)
{
  visualization_msgs::InteractiveMarker dummy;
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
      case PRW::EndEffectorControl:
        makeInteractive6DOFMarker(false, transform, marker.controlName_, marker.controlDescription_, 0.225f, false);
        break;
      case PRW::CollisionObject:
        makeInteractive6DOFMarker(false, transform, marker.controlName_, marker.controlDescription_, 2.0f, true);
        break;
      case PRW::JointControl:
        makeInteractive6DOFMarker(false, transform, marker.controlName_, marker.controlDescription_, 0.225f, false);
        break;
    }
  }
}

/////
/// @brief Removes the 6DOF control of the given marker and replaces it with a menu.
/// @param marker a reference to the selectablemarker struct.
/// @param transform location of the marker when it is de-selected.
/////
void PRW::deselectMarker(SelectableMarker& marker, tf::Transform transform)
{
  if (!interactive_marker_server_->erase(marker.controlName_))
  {
    return;
  }

  float scale = 1.0f;

  switch (marker.type_)
  {
    case PRW::EndEffectorControl:
      scale = 0.5f;
      break;
    case PRW::CollisionObject:
      scale = 2.0f;
      break;
    case PRW::JointControl:
      scale = 0.225f;
      break;
  }

  makeSelectableMarker(marker.type_, transform, marker.controlName_, marker.controlDescription_, scale);
}

void PRW::moveEndEffectorMarkers(double vx, double vy, double vz, double vr, double vp, double vw, bool coll_aware)
{
  lock_.lock();
  GroupCollection& gc = group_map_[current_group_name_];
  tf::Transform cur = gc.getState(ik_control_type_)->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform();
  double mult = CONTROL_SPEED / 100.0;

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

  //setNewEndEffectorPosition(gc, cur, coll_aware);

  lock_.unlock();
}

/////
/// @brief Creates an interactive 6DOF marker control.
/// @param fixed if true, the axes remain fixed to the frame of the marker. Otherwise they rotate with input.
/// @param transform the location and rotation of the 6DOF marker.
/// @param name used internally to represent the marker. Must be unique.
/// @param description drawn above the marker as a label.
/// @param scale uniformly sets the size in meters of the marker.
/// @param pole if true, the marker is a large green cylinder. Otherwise it is a small white cube.
/////
void PRW::makeInteractive6DOFMarker(bool fixed, tf::Transform transform, std::string name, std::string description, float scale,
                               bool pole)
{
  visualization_msgs::InteractiveMarker marker;
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

  if (!pole)
  {
    makeInteractiveBoxControl(marker, 0.5f);
  }
  else
  {
    makeInteractiveCylinderControl(marker, 1.0f);
  }

  visualization_msgs::InteractiveMarkerControl control;

  if (fixed)
  {
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  }

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.always_visible = false;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  interactive_marker_server_->insert(marker);

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  //control.markers.push_back(makeMarkerSphere(marker));
  marker.controls.push_back(control);

  if (!pole)
  {
    menu_handler_map_["End Effector"].apply(*interactive_marker_server_, marker.name);
  }
  else
  {
    menu_handler_map_["Collision Object"].apply(*interactive_marker_server_, marker.name);
  }

  interactive_marker_server_->setCallback(marker.name, process_function_ptr_);
}

/////
/// @brief Creates a marker that is initially a clickable menu. Upon selection it turns into a 6DOF control.
/// @param type the type (collision object, joint, IK control, etc.) of the selectable marker.
/// @param transform location and orientation of the marker.
/// @param name internal, unique representation of the marker (this is for the 6DOF control)
/// @param description displayed above the menu marker and the 6DOF marker.
/// @param scale uniformly sizes the marker and its controls
/// @param publish if true, the marker server will publish the marker. Otherwise, it will not.
/////
void PRW::makeSelectableMarker(InteractiveMarkerType type, tf::Transform transform, std::string name,
                               std::string description, float scale, bool publish)
{
  SelectableMarker selectable_marker;
  selectable_marker.type_ = type;
  selectable_marker.name_ = name + "_selectable";
  selectable_marker.controlName_ = name;
  selectable_marker.controlDescription_ = description;

  visualization_msgs::InteractiveMarker marker;
  marker.header.frame_id = "/" + cm_->getWorldFrameId();
  ;
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
  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  switch (type)
  {
    case PRW::EndEffectorControl:
      control.markers.push_back(makeMarkerBox(marker, 0.5f));
      marker.controls.push_back(control);
      interactive_marker_server_->insert(marker);
      interactive_marker_server_->setCallback(marker.name, process_function_ptr_);
      menu_handler_map_["End Effector Selection"].apply(*interactive_marker_server_, marker.name);
      break;
    case PRW::CollisionObject:
      control.markers.push_back(makeMarkerCylinder(marker, 1.0f));
      marker.controls.push_back(control);
      interactive_marker_server_->insert(marker);
      interactive_marker_server_->setCallback(marker.name, process_function_ptr_);
      menu_handler_map_["Collision Object Selection"].apply(*interactive_marker_server_, marker.name);
      break;
    case PRW::JointControl:
      control.markers.push_back(makeMarkerBox(marker, 0.5f));
      marker.controls.push_back(control);
      interactive_marker_server_->insert(marker);
      interactive_marker_server_->setCallback(marker.name, process_function_ptr_);
      menu_handler_map_["Joint Selection"].apply(*interactive_marker_server_, marker.name);
      break;
  }

  selectable_markers_[marker.name] = selectable_marker;

  if (publish)
  {
    interactive_marker_server_->applyChanges();
  }
}


visualization_msgs::Marker PRW::makeMarkerBox(visualization_msgs::InteractiveMarker &msg, float alpha)
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
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

/////
/// @brief Creates a cylinder shaped marker.
/// @param msg the interactive marker to associate this cylinder with.
/// @param alpha the transparency of the marker.
/// @return the marker (which is a cylinder).
/////
visualization_msgs::Marker PRW::makeMarkerCylinder(visualization_msgs::InteractiveMarker &msg, float alpha)
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CYLINDER;
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

/////
/// @brief Creates a sphere-shaped marker.
/// @param msg the interactive marker to associate this sphere with.
/// @return the marker (which is a sphere)
/////
visualization_msgs::Marker PRW::makeMarkerSphere(visualization_msgs::InteractiveMarker &msg)
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::SPHERE;
  // Scale is arbitrary.
  marker.scale.x = msg.scale * 0.75;
  marker.scale.y = msg.scale * 0.75;
  marker.scale.z = msg.scale * 0.75;
  marker.color.r = 0.8;
  marker.color.g = 0.8;
  marker.color.b = 1.0;
  marker.color.a = 0.1;

  return marker;
}

/////
/// @brief Creates a clickable, box shaped marker.
/// @param msg the interactive marker to associate this box with.
/// @param alpha the transparency of the marker.
/// @return the control (which is a clickable box)
/////
visualization_msgs::InteractiveMarkerControl& PRW::makeInteractiveBoxControl(visualization_msgs::InteractiveMarker &msg, float alpha)
{
  visualization_msgs::InteractiveMarkerControl control;
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
visualization_msgs::InteractiveMarkerControl& PRW::makeInteractiveCylinderControl(visualization_msgs::InteractiveMarker &msg, float alpha)
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeMarkerCylinder(msg, alpha));
  msg.controls.push_back(control);
  return msg.controls.back();
}

void PRW::makeTopLevelMenu()
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.pose.position.z = 2.25;
  int_marker.name = "top_level";
  int_marker.description = "Personal Robotic Workspace";
  int_marker.header.frame_id = "/" + cm_->getWorldFrameId();


  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  control.always_visible = true;

  visualization_msgs::Marker labelMarker;
  labelMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
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

  interactive_marker_server_->insert(int_marker);
  interactive_marker_server_->setCallback(int_marker.name, process_function_ptr_);
  menu_handler_map_["Top Level"].apply(*interactive_marker_server_, int_marker.name);
}

void PRW::deleteJointMarkers(GroupCollection& gc)
{
  for (size_t i = 0; i < gc.joint_names_.size(); i++)
  {
    interactive_marker_server_->erase(gc.joint_names_[i] + "_joint_control");
  }
}


void PRW::createSelectableJointMarkers(GroupCollection& gc)
{
  if (!is_joint_control_active_)
  {
    return;
  }

  /*
  // For each joint model, find the location of its axis and make a control there.
  for (size_t i = 0; i < gc.joint_names_.size(); i++)
  {
    const string& jointName = gc.joint_names_[i];
    KinematicModel::JointModel* model =
        (KinematicModel::JointModel*)(gc.getState(ik_control_type_)->getKinematicModel()->getJointModel(jointName));
    KinematicModel::RevoluteJointModel* revoluteJoint = dynamic_cast<KinematicModel::RevoluteJointModel*>(model);
    KinematicModel::PrismaticJointModel* prismaticJoint = dynamic_cast<KinematicModel::PrismaticJointModel*>(model);

    joint_clicked_map_[jointName + "_joint_control"] = false;

    if (model->getParentLinkModel() != NULL)
    {
      string parentLinkName = model->getParentLinkModel()->getName();
      string childLinkName = model->getChildLinkModel()->getName();
      tf::Transform transform = gc.getState(ik_control_type_)->getLinkState(parentLinkName)->getGlobalLinkTransform()
          * (gc.getState(ik_control_type_)->getKinematicModel()->getLinkModel(childLinkName)->getJointOriginTransform()
              * (gc.getState(ik_control_type_)->getJointState(jointName)->getVariableTransform()));

      joint_prev_transform_map_[jointName + "joint_control"] = transform;

      const shapes::Shape* linkShape = model->getChildLinkModel()->getLinkShape();
      const shapes::Mesh* meshShape = dynamic_cast<const shapes::Mesh*>(linkShape);

      double maxDimension = 0.0f;
      if (meshShape != NULL)
      {
        for (unsigned int i = 0; i < meshShape->vertexCount; i++)
        {
          double x = meshShape->vertices[3 * i];
          double y = meshShape->vertices[3 * i];
          double z = meshShape->vertices[3 * i];

          if (abs(maxDimension) < abs(sqrt(x * x + y * y + z * z)))
          {
            maxDimension = abs(x);
          }

        }

        maxDimension *= 3.0;

        maxDimension = max(0.15, maxDimension);
        maxDimension = min(0.5, maxDimension);
      }
      else
      {
        maxDimension = 0.15;
      }

      if (revoluteJoint != NULL)
      {
        makeInteractive1DOFRotationMarker(
            transform, revoluteJoint->axis_, model->getName() + "_joint_control", "", (float)maxDimension,
            gc.getState(ik_control_type_)->getJointState(jointName)->getJointStateValues()[0]);
      }
      else if (prismaticJoint != NULL)
      {
        maxDimension *= 3.0;
        makeInteractive1DOFTranslationMarker(
            transform, prismaticJoint->axis_, model->getName() + "_joint_control", "", (float)maxDimension,
            gc.getState(ik_control_type_)->getJointState(jointName)->getJointStateValues()[0]);
      }

    }
  }

  interactive_marker_server_->applyChanges();
  */
}
