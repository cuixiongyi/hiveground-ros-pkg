#include <termios.h>
#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <math.h>

#include <ros/ros.h>
#include <prw/prw.h>

#include <QtGui/QApplication>

static const std::string VIS_TOPIC_NAME = "prw_visualizer";
static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
static const std::string PLANNER_SERVICE_NAME = "/ompl_planning/plan_kinematic_path";
static const std::string TRAJECTORY_FILTER_SERVICE_NAME = "/trajectory_filter_server/filter_trajectory_with_constraints";

//in 100 hz ticks
static const unsigned int CONTROL_SPEED = 10;


#define DEG2RAD(x) (((x)*M_PI)/180.0)

PRW::PRW(QWidget *parent, Qt::WFlags flags)
  : QMainWindow(parent, flags),
    node_handle_("~"),
    quit_threads_(false)

{
  ui.setupUi(this);

}


PRW::~PRW()
{
}

void PRW::initialize()
{
  ik_control_type_ = EndPosition;
  collision_aware_ = true;
  constrain_rp_ = false;
  is_ik_control_active_ = true;
  is_joint_control_active_ = false;

  vis_marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker> (VIS_TOPIC_NAME, 128);
  vis_marker_array_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray> (VIS_TOPIC_NAME + "_array", 128);
  process_function_ptr_ = boost::bind(&PRW::processInteractiveFeedback, this, _1);

  cm_ = new planning_environment::CollisionModels("robot_description");

  /*
  ros::service::waitForService("/arm_kinematics/get_ik");
  ros::service::waitForService("/arm_kinematics/get_ik_solver_info");
  ik_client_ = node_handle_.serviceClient<kinematics_msgs::GetPositionIK>("/arm_kinematics/get_ik", true);
  ik_info_client_ = node_handle_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("/arm_kinematics/get_ik_solver_info");
  */
  subscriber_joint_state_ = node_handle_.subscribe("/joint_states", 1, &PRW::callbackJointState, this);

  publisher_joints_[0] = node_handle_.advertise<std_msgs::Float64>("/arm1/J1/command/position", 1);
  publisher_joints_[1] = node_handle_.advertise<std_msgs::Float64>("/arm1/J2/command/position", 1);
  publisher_joints_[2] = node_handle_.advertise<std_msgs::Float64>("/arm1/J3/command/position", 1);
  publisher_joints_[3] = node_handle_.advertise<std_msgs::Float64>("/arm1/J4/command/position", 1);
  publisher_joints_[4] = node_handle_.advertise<std_msgs::Float64>("/arm1/J5/command/position", 1);
  publisher_joints_[5] = node_handle_.advertise<std_msgs::Float64>("/arm1/J6/command/position", 1);


  while(!ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME, ros::Duration(1.0)))
  {
    ROS_INFO_STREAM("Waiting for planning scene service " << SET_PLANNING_SCENE_DIFF_NAME);
  }

  set_planning_scene_diff_client_
      = node_handle_.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff> (SET_PLANNING_SCENE_DIFF_NAME);

  while(!ros::service::waitForService(PLANNER_SERVICE_NAME, ros::Duration(1.0)))
  {
    ROS_INFO_STREAM("Waiting for planner service " << PLANNER_SERVICE_NAME);
  }

  planner_service_client_ = node_handle_.serviceClient<arm_navigation_msgs::GetMotionPlan> (PLANNER_SERVICE_NAME, true);

  while(!ros::service::waitForService(TRAJECTORY_FILTER_SERVICE_NAME, ros::Duration(1.0)))
  {
    ROS_INFO_STREAM("Waiting for trajectory filter service " << TRAJECTORY_FILTER_SERVICE_NAME);
  }

  trajectory_filter_service_client_
      = node_handle_.serviceClient<arm_navigation_msgs::FilterJointTrajectoryWithConstraints> (TRAJECTORY_FILTER_SERVICE_NAME, true);


  const KinematicModelGroupConfigMap& group_config_map = cm_->getKinematicModel()->getJointModelGroupConfigMap();

  for(KinematicModelGroupConfigMap::const_iterator it = group_config_map.begin(); it
      != group_config_map.end(); it++)
  {
    ROS_INFO_STREAM("Kinematic model group: " << it->first);
    if(!it->second.base_link_.empty())
    {
      group_map_[it->first].name_ = it->first;
      group_map_[it->first].ik_link_name_ = it->second.tip_link_;
      std::string ik_service_name = cm_->getKinematicModel()->getRobotName() + "_" + it->first + "_kinematics/";
      std::string coll_aware_name = ik_service_name + "get_constraint_aware_ik";
      std::string non_coll_aware_name = ik_service_name + "get_ik";

      ROS_INFO_STREAM("Waiting for service " << coll_aware_name);
      while(!ros::service::waitForService(coll_aware_name, ros::Duration(1.0)))
      {
        ROS_INFO_STREAM("Waiting for service " << coll_aware_name);
      }

      ROS_INFO_STREAM("Waiting for service " << non_coll_aware_name);
      while(!ros::service::waitForService(non_coll_aware_name, ros::Duration(1.0)))
      {
        ROS_INFO_STREAM("Waiting for service " << non_coll_aware_name);
      }

      group_map_[it->first].coll_aware_ik_service_ = node_handle_.serviceClient<
          kinematics_msgs::GetConstraintAwarePositionIK> (coll_aware_name, true);

      group_map_[it->first].non_coll_aware_ik_service_
          = node_handle_.serviceClient<kinematics_msgs::GetPositionIK> (non_coll_aware_name, true);
    }
  }
  robot_state_ = new planning_models::KinematicState(cm_->getKinematicModel());
  robot_state_->setKinematicStateToDefault();


  sendPlanningScene();


  // Create a new interactive marker server.
  interactive_marker_server_.reset(new interactive_markers::InteractiveMarkerServer("prw_visualizer_controls", "", false));

  // "Select Planning Chain" sub menu.
  unsigned int cmd = 0;
  for(GroupMap::iterator it = group_map_.begin(); it != group_map_.end(); it++)
  {
    //registerSubMenuEntry(menu_handler_map_["Top Level"], menu_entry_maps_["Top Level"], it->first, sub_menu_handle);

    // These positions will be reset by main()
    ROS_INFO_STREAM(it->first);
    makeSelectableMarker(PRW::EndEffectorControl,
                         tf::Transform(tf::Quaternion(0.0f, 0.0f, 0.0f, 1.0f), tf::Vector3(0.0f, 0.0f, 0.0f)), it->first,
                         it->first, 0.5f);
    cmd++;
  }


  makeToLevelMenu();

  interactive_marker_server_->applyChanges();

  ROS_INFO_STREAM("Initialized");


  ROS_WARN_STREAM("!!TESTING AHEAD!!");

  selectPlanningGroup(0);
  solveIKForEndEffectorPose(*(getPlanningGroup(0)), false);
  updateJointStates(*(getPlanningGroup(0)));

  ROS_WARN_STREAM("!!END TESTING!!");

}

void PRW::on_ik_move_go_clicked()
{
  /*
  ROS_INFO("Go clicked! xyz(%4.2f, %4.2f %4.2f) ryp(%4.2f, %4.2f, %4.2f)",
           ui.ik_move_x->value(),
           ui.ik_move_y->value(),
           ui.ik_move_z->value(),
           ui.ik_move_roll->value(),
           ui.ik_move_pitch->value(),
           ui.ik_move_yaw->value());
  */

  geometry_msgs::PoseStamped ps, pose;
  ps.header.frame_id = "/world";
  ps.pose.position.x = ui.ik_move_x->value();
  ps.pose.position.y = ui.ik_move_y->value();
  ps.pose.position.z = ui.ik_move_z->value();

  tf::Quaternion q(DEG2RAD(ui.ik_move_yaw->value()), DEG2RAD(ui.ik_move_pitch->value()), DEG2RAD(ui.ik_move_roll->value()));
  ps.pose.orientation.x = q.x();
  ps.pose.orientation.y = q.y();
  ps.pose.orientation.z = q.z();
  ps.pose.orientation.w = q.w();

  //tf_listener_.transformPose("/base_link", ps, pose);
  //ROS_INFO_STREAM(ps.pose);
  //ROS_INFO_STREAM(pose.pose);


  kinematics_msgs::GetPositionIKRequest request;
  request.timeout = ros::Duration(5.0);
  request.ik_request.pose_stamped.header.frame_id = "/base_link";
  request.ik_request.ik_link_name = "link5";
  request.ik_request.pose_stamped.pose = ps.pose; //not transformation
  //request.ik_request.pose_stamped.pose.position.y = pose.pose.position.y
  //request.ik_request.pose_stamped.pose.position.z = pose.pose.position.z

  //ik_info_client_.
  kinematics_msgs::GetKinematicSolverInfo srv;
  if(ik_info_client_.call(srv))
  {
    ROS_INFO_STREAM(srv.response.kinematic_solver_info.joint_names.size());
    for(size_t i = 0; i < srv.response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_INFO_STREAM(srv.response.kinematic_solver_info.joint_names[i]);
    }
  }

  request.ik_request.ik_seed_state.joint_state.name = srv.response.kinematic_solver_info.joint_names;
  request.ik_request.ik_seed_state.joint_state.position = joint_positions_;

  kinematics_msgs::GetPositionIKResponse respond;
  if(ik_client_.call(request, respond))
  {
    if(respond.error_code.val == ::arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS)
    {
      for(size_t i = 0; i < respond.solution.joint_state.position.size(); i++)
      {
        ROS_INFO_STREAM(respond.solution.joint_state.position[i]);
        std_msgs::Float64 msg;
        msg.data = respond.solution.joint_state.position[i];
        publisher_joints_[i].publish(msg);
      }
    }
    else
    {
      ROS_INFO("request not success");
    }
  }
  else
  {
    ROS_INFO("request error");
  }

}

void PRW::on_ik_move_reset_clicked()
{
  ui.ik_move_x->setValue(0.0);
  ui.ik_move_y->setValue(0.0);
  ui.ik_move_z->setValue(0.0);
  ui.ik_move_roll->setValue(0.0);
  ui.ik_move_pitch->setValue(0.0);
  ui.ik_move_yaw->setValue(0.0);
}

void PRW::closeEvent(QCloseEvent *event)
{
  ROS_INFO("Close windows");
  quit_threads_ = true;
  event->accept();
}


void PRW::callbackJointState(const sensor_msgs::JointState& message)
{
  //ROS_INFO_STREAM(message);
  joint_positions_ = message.position;
  //robot_state_->set

  lock_.lock();

  //robot_state_->getJointStateGroupMap();

  /*
  std::map<std::string, double> joint_values;
  for(unsigned int i = 0; i < robot_state_->j.joint_trajectory_.joint_names.size(); i++)
  {
    joint_values[disp.joint_trajectory_.joint_names[i]]
        = disp.joint_trajectory_.points[disp.current_trajectory_point_].positions[i];
  }
  disp.state_->setKinematicState(joint_values);
  */

  lock_.unlock();

}

void PRW::selectPlanningGroup(unsigned int entry)
{
  ROS_INFO("Selecting planning group %u", entry);

  lock_.lock();
  std::vector<std::string> names;
  for(GroupMap::iterator it = group_map_.begin(); it != group_map_.end(); it++)
  {
    names.push_back(it->first);
  }
  std::string old_group_name = current_group_name_;

  current_group_name_ = names[entry];

  //check if end state exist
  if(group_map_[current_group_name_].end_state_ != NULL)
  {
    ROS_WARN_STREAM("Selecting with non NULL");
  }
  else
  {
    group_map_[current_group_name_].setState(EndPosition, new planning_models::KinematicState(*robot_state_));
  }

  //check if start state exist
  if(group_map_[current_group_name_].start_state_ != NULL)
  {
    ROS_WARN_STREAM("Selecting with non NULL");
  }
  else
  {
    group_map_[current_group_name_].setState(StartPosition, new planning_models::KinematicState(*robot_state_));
  }


  lock_.unlock();
  ROS_INFO_STREAM("Planning group [" << current_group_name_ << "] selected.");
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

  if(roll > 2 * M_PI)
  {
    roll -= 2 * M_PI;
  }
  else if(roll < -2 * M_PI)
  {
    roll += 2 * M_PI;
  }

  if(pitch > 2 * M_PI)
  {
    pitch -= 2 * M_PI;
  }
  else if(pitch < -2 * M_PI)
  {
    pitch += 2 * M_PI;
  }

  cur.getBasis().setRPY(roll, pitch, yaw);

  setNewEndEffectorPosition(gc, cur, coll_aware);

  lock_.unlock();
}

void PRW::setNewEndEffectorPosition(PRW::GroupCollection& gc, tf::Transform& cur, bool coll_aware)
{
  if (!gc.getState(ik_control_type_)->updateKinematicStateWithLinkAt(gc.ik_link_name_, cur))
  {
    ROS_INFO("Problem");
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


bool PRW::solveIKForEndEffectorPose(PRW::GroupCollection& gc, bool coll_aware, bool constrain_pitch_and_roll,
                                    double change_redundancy)
{
  kinematics_msgs::PositionIKRequest ik_request;

  ROS_INFO_STREAM(gc.ik_link_name_ << ":" << cm_->getWorldFrameId() );


  ik_request.ik_link_name = gc.ik_link_name_;
  ik_request.pose_stamped.header.frame_id = cm_->getWorldFrameId();
  ik_request.pose_stamped.header.stamp = ros::Time::now();
  tf::poseTFToMsg(gc.getState(ik_control_type_)->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform(),
                  ik_request.pose_stamped.pose);
  planning_environment::convertKinematicStateToRobotState(*gc.getState(ik_control_type_), ros::Time::now(),
                                                          cm_->getWorldFrameId(), ik_request.robot_state);
  ik_request.ik_seed_state = ik_request.robot_state;

  //ROS_INFO_STREAM(ik_request.ik_seed_state.joint_state);


  std::map<std::string, double> joint_values;
  std::vector <std::string> joint_names;

  if (coll_aware)
  {
    kinematics_msgs::GetConstraintAwarePositionIK::Request ik_req;
    kinematics_msgs::GetConstraintAwarePositionIK::Response ik_res;
    if (constrain_pitch_and_roll)
    {
      IKControlType other_state;
      if (ik_control_type_ == EndPosition)
      {
        other_state = StartPosition;
      }
      else
      {
        other_state = EndPosition;
      }
      arm_navigation_msgs::Constraints goal_constraints;
      goal_constraints.orientation_constraints.resize(1);
      arm_navigation_msgs::Constraints path_constraints;
      path_constraints.orientation_constraints.resize(1);
      determinePitchRollConstraintsGivenState(gc, *gc.getState(other_state),
                                              goal_constraints.orientation_constraints[0],
                                              path_constraints.orientation_constraints[0]);
      arm_navigation_msgs::ArmNavigationErrorCodes err;
      if (!cm_->isKinematicStateValid(*gc.getState(ik_control_type_), std::vector<std::string>(), err, goal_constraints,
                                      path_constraints))
      {
        ROS_INFO_STREAM("Violates rp constraints");
        return false;
      }
      ik_req.constraints = goal_constraints;
    }


    ik_req.ik_request = ik_request;
    ik_req.timeout = ros::Duration(0.2);
    if (!gc.coll_aware_ik_service_.call(ik_req, ik_res))
    {
      ROS_INFO("Problem with ik service call (collision check)");
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
    if (!gc.non_coll_aware_ik_service_.call(ik_req, ik_res))
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

  lock_.lock();
  gc.getState(ik_control_type_)->setKinematicState(joint_values);
  lock_.unlock();


  createSelectableJointMarkers(gc);
  if (coll_aware)
  {
    arm_navigation_msgs::Constraints emp_con;
    arm_navigation_msgs::ArmNavigationErrorCodes error_code;

    if (!cm_->isKinematicStateValid(*gc.getState(ik_control_type_), joint_names, error_code, emp_con, emp_con, true))
    {
      ROS_INFO_STREAM("Problem with response");
    }
  }

  updateJointStates(gc);


  return true;
}

void PRW::determinePitchRollConstraintsGivenState(const PRW::GroupCollection& gc,
                                                  const planning_models::KinematicState& state,
                                                  arm_navigation_msgs::OrientationConstraint& goal_constraint,
                                                  arm_navigation_msgs::OrientationConstraint& path_constraint)
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

void PRW::deleteKinematicStates()
{
  for (GroupMap::iterator it = group_map_.begin(); it != group_map_.end(); it++)
  {
    it->second.reset();
  }
}

////
/// @brief Given the group collection, creates a set of selectable markers for each joint in the group.
/// @param gc the group collection to create joint markers for.
////
void PRW::createSelectableJointMarkers(PRW::GroupCollection& gc)
{
  if (!is_joint_control_active_)
  {
    return;
  }

  ROS_WARN_STREAM(__FUNCTION__ << "!!!COMMENTED OUT!!!");

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

/////
/// @brief Sends the joint states of the given group collection to the robot state publisher.
/// @param gc the group collection to publish the states for.
/////
void PRW::updateJointStates(PRW::GroupCollection& gc)
{
  sensor_msgs::JointState msg;
  msg.header.frame_id = cm_->getWorldFrameId();
  msg.header.stamp = ros::Time::now();

  std::vector<planning_models::KinematicState::JointState*> jointStates =
      gc.getState(ik_control_type_)->getJointStateVector();

  std::map<std::string, double> stateMap;
  gc.getState(ik_control_type_)->getKinematicStateValues(stateMap);
  robot_state_->setKinematicState(stateMap);

  for (size_t i = 0; i < jointStates.size(); i++)
  {
    planning_models::KinematicState::JointState* state = jointStates[i];
    msg.name.push_back(state->getName());

    // Assume that joints only have one value.
    if (state->getJointStateValues().size() > 0)
    {
      msg.position.push_back(state->getJointStateValues()[0]);
    }
    else
    {
      msg.position.push_back(0.0f);
    }
  }
  joint_state_lock_.lock();
  last_joint_state_msg_ = msg;
  joint_state_lock_.unlock();

}

void PRW::publishJointStates()
{
  /*
  joint_state_lock_.lock();
  last_joint_state_msg_.header.frame_id = cm_->getWorldFrameId();
  last_joint_state_msg_.header.stamp = ros::Time::now();
  joint_state_publisher_.publish(last_joint_state_msg_);
  joint_state_lock_.unlock();

  if (cm_->getWorldFrameId() != cm_->getRobotFrameId())
  {
    TransformStamped trans;
    trans.header.frame_id = cm_->getWorldFrameId();
    trans.header.stamp = ros::Time::now();
    trans.child_frame_id = cm_->getRobotFrameId();
    trans.transform.rotation.w = 1.0;
    transform_broadcaster_.sendTransform(trans);
  }
  */
}


void PRW::makeToLevelMenu()
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.pose.position.z = 2.25;
  int_marker.name = "top_level";
  int_marker.description = "Personal Robotics Workspace";
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

}
void PRW::processInteractiveFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  ROS_INFO_STREAM("interactive!");
}



void PRW::sendPlanningScene()
{
  ROS_INFO("Sending Planning Scene....");

  lock_.lock();
  arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
  arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;


  /*
  std::vector <std::string> removals;
  // Handle additions and removals of planning scene objects.
  for (std::map<std::string, arm_navigation_msgs::CollisionObject>::const_iterator it = collision_poles_.begin();
      it != collision_poles_.end(); it++)
  {
    string name = it->first;
    arm_navigation_msgs::CollisionObject object = it->second;

    // Add or remove objects.
    if (object.operation.operation != arm_navigation_msgs::CollisionObjectOperation::REMOVE)
    {
      ROS_INFO("Adding Collision Pole %s", object.id.c_str());
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
    collision_poles_.erase(removals[i]);
  }
  */

  planning_environment::convertKinematicStateToRobotState(*robot_state_, ros::Time::now(), cm_->getWorldFrameId(),
                                                          planning_scene_req.planning_scene_diff.robot_state);

  planning_models::KinematicState* startState = NULL;
  planning_models::KinematicState* endState = NULL;
  std::map<std::string, double> startStateValues;
  std::map<std::string, double> endStateValues;

  if (current_group_name_ != "")
  {
    startState = group_map_[current_group_name_].getState(StartPosition);
    endState = group_map_[current_group_name_].getState(EndPosition);

    if (startState != NULL)
    {
      startState->getKinematicStateValues(startStateValues);
    }
    if (endState != NULL)
    {
      endState->getKinematicStateValues(endStateValues);
    }
  }

  deleteKinematicStates();

  if (robot_state_ != NULL)
  {
    ROS_INFO("Reverting planning scene to default.");
    cm_->revertPlanningScene(robot_state_);
    robot_state_ = NULL;
  }

  if (!set_planning_scene_diff_client_.call(planning_scene_req, planning_scene_res))
  {
    ROS_WARN("Can't get planning scene");
    lock_.unlock();
    return;
  }

  robot_state_ = cm_->setPlanningScene(planning_scene_res.planning_scene);

  if (robot_state_ == NULL)
  {
    ROS_ERROR("Something wrong with planning scene");
    lock_.unlock();
    return;
  }

  if (current_group_name_ != "")
  {
    ROS_INFO("Resetting state...");
    group_map_[current_group_name_].setState(StartPosition, new planning_models::KinematicState(robot_state_->getKinematicModel()));
    group_map_[current_group_name_].setState(EndPosition, new planning_models::KinematicState(robot_state_->getKinematicModel()));
    startState = group_map_[current_group_name_].getState(StartPosition);
    endState = group_map_[current_group_name_].getState(EndPosition);

    if (startState != NULL)
    {
      ROS_INFO("Resetting start state.");
      startState->setKinematicState(startStateValues);
    }

    if (endState != NULL)
    {
      ROS_INFO("Resetting end state.");
      endState->setKinematicState(endStateValues);
    }
  }
  lock_.unlock();
  ROS_INFO("Planning scene sent.");
}

void PRW::sendMarkers()
{
  lock_.lock();
  visualization_msgs::MarkerArray arr;

  std_msgs::ColorRGBA stat_color_;
  stat_color_.a = 1.0;
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
    std_msgs::ColorRGBA group_color;
    group_color.a = 0.3;
    group_color.r = 0.5;
    group_color.g = 0.9;
    group_color.b = 0.5;

    std_msgs::ColorRGBA updated_color;
    updated_color.a = 0.3;
    updated_color.r = 1.0;
    updated_color.g = 0.5;
    updated_color.b = 1.0;

    std_msgs::ColorRGBA bad_color;
    bad_color.a = 0.6;
    bad_color.r = 1.0;
    bad_color.g = 0.0;
    bad_color.b = 0.0;

    GroupCollection& gc = group_map_[current_group_name_];
    const planning_models::KinematicModel* kinematic_model = cm_->getKinematicModel();

    IKControlType otherState;
    if (ik_control_type_ == EndPosition)
    {
      otherState = StartPosition;
    }
    else
    {
      otherState = EndPosition;
    }

    if (is_ik_control_active_)
    {
      if (gc.getState(otherState) != NULL)
      {
        cm_->getGroupAndUpdatedJointMarkersGivenState(*gc.getState(otherState), arr, current_group_name_, group_color,
                                                      updated_color, ros::Duration(0.1));
      }
      else
      {
        ROS_ERROR("Other state invalid!");
      }
    }

    if (!gc.good_ik_solution_ && gc.getState(ik_control_type_) != NULL)
    {
      std::vector <std::string> lnames = kinematic_model->getChildLinkModelNames(
          kinematic_model->getLinkModel(gc.ik_link_name_));

      cm_->getRobotMarkersGivenState(*gc.getState(ik_control_type_), arr, bad_color, current_group_name_,
                                     ros::Duration(0.1), &lnames);
      cm_->getAttachedCollisionObjectMarkers(*gc.getState(ik_control_type_), arr, current_group_name_, bad_color,
                                             ros::Duration(.2));

    }
    for (std::map<std::string, StateTrajectoryDisplay>::iterator it = gc.state_trajectory_display_map_.begin();
        it != gc.state_trajectory_display_map_.end(); it++)
    {

      /*
      if (it->second.play_joint_trajectory_)
      {
        moveThroughTrajectory(gc, it->first, 5);
      }

      if (it->second.show_joint_trajectory_)
      {
        const vector<const KinematicModel::LinkModel*>& updated_links =
            kinematic_model->getModelGroup(gc.name_)->getUpdatedLinkModels();
        vector < string > lnames;
        lnames.resize(updated_links.size());
        for (unsigned int i = 0; i < updated_links.size(); i++)
        {
          lnames[i] = updated_links[i]->getName();
        }
        cm_->getRobotMarkersGivenState(*(it->second.state_), arr, it->second.color_, it->first + "_trajectory",
                                       ros::Duration(0.1), &lnames);

        cm_->getAttachedCollisionObjectMarkers(*(it->second.state_), arr, it->first + "_trajectory", it->second.color_,
                                               ros::Duration(0.1));
      }
      */
    }
  }
  vis_marker_array_publisher_.publish(arr);
  lock_.unlock();
}

PRW* prw_ = NULL;
bool initialized_ = false;
void spin_function()
{
  ros::WallRate r(100.0);
  while(ros::ok() && !initialized_)
  {
    r.sleep();
    ros::spinOnce();
  }
  while(ros::ok() && !prw_->quit_threads_)
  {
    r.sleep();
    ros::spinOnce();
  }
}

void update_function()
{
  unsigned int counter = 0;
  while(ros::ok())
  {
    if(initialized_)
    {
      //pcv->sendTransforms();
      if(counter % CONTROL_SPEED == 0)
      {
        //ROS_INFO("send");
        counter = 1;
        prw_->sendMarkers();
      }
      else
      {
        //pcv->publishJointStates();
        counter++;
      }
    }

    usleep(5000);
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "personal_robotic_workspace", ros::init_options::NoSigintHandler);

  boost::thread spin_thread(boost::bind(&spin_function));
  boost::thread update_thread(boost::bind(&update_function));

  QApplication a(argc, argv);

  PRW w;
  w.initialize();
  prw_ = &w;
  w.show();

  initialized_ = true;

  int ret = a.exec();

  spin_thread.join();

  return ret;
}
