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

#include <unistd.h>

#include <ros/ros.h>
#include <prw/prw.h>
#include <QDebug>



using namespace std;
using namespace planning_environment;
using namespace planning_models;
using namespace visualization_msgs;
using namespace arm_navigation_msgs;
using namespace kinematics_msgs;
using namespace interactive_markers;
using namespace control_msgs;

static const string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
static const string PLANNER_SERVICE_NAME = "/ompl_planning/plan_kinematic_path";
static const string TRAJECTORY_FILTER_SERVICE_NAME = "/trajectory_filter_server/filter_trajectory_with_constraints";

#define DEG2RAD(x) (((x)*M_PI)/180.0)
#define RAD2DEG(x) (((x)*180.0)/M_PI)

PRW::PRW(QWidget *parent, Qt::WFlags flags) :
    QMainWindow(parent, flags),
    quit_threads_(false),
    nh_private_("~")
{
  ui.setupUi(this);


  connect(ui.hs_move_x, SIGNAL(valueChanged(int)), this, SLOT(endEffectorSlideUpdate()));
  connect(ui.hs_move_y, SIGNAL(valueChanged(int)), this, SLOT(endEffectorSlideUpdate()));
  connect(ui.hs_move_z, SIGNAL(valueChanged(int)), this, SLOT(endEffectorSlideUpdate()));
  connect(ui.hs_roll, SIGNAL(valueChanged(int)), this, SLOT(endEffectorSlideUpdate()));
  connect(ui.hs_pitch, SIGNAL(valueChanged(int)), this, SLOT(endEffectorSlideUpdate()));
  connect(ui.hs_yaw, SIGNAL(valueChanged(int)), this, SLOT(endEffectorSlideUpdate()));

  connect(ui.sb_move_x, SIGNAL(valueChanged(double)), this, SLOT(endEffectorValueUpdate()));
  connect(ui.sb_move_y, SIGNAL(valueChanged(double)), this, SLOT(endEffectorValueUpdate()));
  connect(ui.sb_move_z, SIGNAL(valueChanged(double)), this, SLOT(endEffectorValueUpdate()));
  connect(ui.sb_roll, SIGNAL(valueChanged(double)), this, SLOT(endEffectorValueUpdate()));
  connect(ui.sb_pitch, SIGNAL(valueChanged(double)), this, SLOT(endEffectorValueUpdate()));
  connect(ui.sb_yaw, SIGNAL(valueChanged(double)), this, SLOT(endEffectorValueUpdate()));


}

PRW::~PRW()
{

}

bool PRW::initialize()
{
  ROS_INFO_STREAM("Initializing...");
  on_bt_reset_clicked();
  on_bt_reset_clicked();

    //get robot description
  string robot_description_name = nh_.resolveName("robot_description", true);

  //create robot collision model
  cm_ = new CollisionModels("robot_description");

  robot_state_= NULL;
  collision_aware_ = true;
  constrain_rp_ = false;
  is_joint_control_active_ = false;
  is_ik_control_active_ = true;
  arm_is_moving_ = false;

  //subscriber
  joint_state_subscriber_ = nh_.subscribe("joint_states", 1, &PRW::jointStateCallback, this);
  gesture_subscriber_ = nh_.subscribe("kinect_gesture", 1, &PRW::gestureCallback, this);

  //publisher
  marker_publisher_ = nh_.advertise<Marker>("prw_workspace_editor", 128);
  marker_array_publisher_ = nh_.advertise<MarkerArray>("prw_workspace_editor_array", 128);

  //interactive marker feedback
  process_ik_controller_feedback_ptr_ = boost::bind(&PRW::processIKControllerCallback, this, _1);
  process_menu_feedback_ptr_ = boost::bind(&PRW::processMenuCallback, this, _1);
  process_marker_feedback_ptr_ = boost::bind(&PRW::processMarkerCallback, this, _1);

  //connect to services
  while (!ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME, ros::Duration(1.0))) { }
  get_planning_scene_client_ = nh_.serviceClient<GetPlanningScene>(SET_PLANNING_SCENE_DIFF_NAME);

  while (!ros::service::waitForService(PLANNER_SERVICE_NAME, ros::Duration(1.0))) { }
  planner_client_ = nh_.serviceClient<GetMotionPlan>(PLANNER_SERVICE_NAME, true);

  while (!ros::service::waitForService(TRAJECTORY_FILTER_SERVICE_NAME, ros::Duration(1.0))) { }
  trajectory_filter_client_ = nh_.serviceClient<FilterJointTrajectoryWithConstraints>(TRAJECTORY_FILTER_SERVICE_NAME, true);

  const map<string, KinematicModel::GroupConfig>& group_config_map = cm_->getKinematicModel()->getJointModelGroupConfigMap();
  int i = 0;
  for (map<string, KinematicModel::GroupConfig>::const_iterator it = group_config_map.begin(); it != group_config_map.end(); it++)
  {
    ROS_INFO("Group %d name: %s with tip link: %s", i, it->first.c_str(), it->second.tip_link_.c_str());
    group_data_map_[it->first].name_ = it->first;
    group_data_map_[it->first].ik_link_name_ = it->second.tip_link_;

    string ik_service_name = cm_->getKinematicModel()->getRobotName() + "_" + it->first + "_kinematics/";
    string coll_aware_name = ik_service_name + "get_constraint_aware_ik";
    string non_coll_aware_name = ik_service_name + "get_ik";
    string arm_controller_name = cm_->getKinematicModel()->getRobotName() + "/follow_joint_trajectory";

    ROS_INFO_STREAM(ik_service_name);
    ROS_INFO_STREAM(coll_aware_name);
    ROS_INFO_STREAM(non_coll_aware_name);
    ROS_INFO_STREAM(arm_controller_name);

    while (!ros::service::waitForService(coll_aware_name, ros::Duration(1.0))) { }
    while (!ros::service::waitForService(non_coll_aware_name, ros::Duration(1.0))) { }

    group_data_map_[it->first].ik_collision_aware_client_ = nh_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>(
        coll_aware_name, true);

    group_data_map_[it->first].ik_non_collision_aware_client_ = nh_.serviceClient<kinematics_msgs::GetPositionIK>(
        non_coll_aware_name, true);

    group_data_map_[it->first].arm_controller_.reset(
        new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(arm_controller_name, true));
    while(ros::ok() && !group_data_map_[it->first].arm_controller_->waitForServer(ros::Duration(1.0))) { }

    i++;
  }

  //initialize robot state
  robot_state_ = new KinematicState(cm_->getKinematicModel());
  robot_state_->setKinematicStateToDefault();


  sendPlanningScene();

  //interactive Marker
  interactive_marker_server_.reset(new InteractiveMarkerServer("prw_visualizer_controls", "", false));

  unsigned int cmd = 0;
  for (PlanningGroupDataMap::iterator it = group_data_map_.begin(); it != group_data_map_.end(); it++)
  {

    makeIKControllerMarker(tf::Transform(tf::Quaternion(0.0f, 0.0f, 0.0f, 1.0f), tf::Vector3(0.0f, 0.0f, 0.0f)),
                           it->first, it->first, true, 0.5f);
    cmd++;
  }

  interactive_marker_server_->applyChanges();

  //waiting for joint information
  while (robot_state_joint_values_.empty() && ros::ok())
  {
    ROS_INFO("waiting for joint information");
    ros::Duration(1.0).sleep();
  }

  //initialize menu
  makeMenu();

  selectPlanningGroup(0);
  solveIKForEndEffectorPose(*getPlanningGroup(0));

  ROS_INFO_STREAM("Initialized");
  return true;

}

void PRW::on_bt_go_clicked()
{
  //qDebug() << __FUNCTION__;
  if(arm_is_moving_)
  {
    ROS_WARN("Arm is moving!");
    return;
  }

  geometry_msgs::Pose pose;
  tf::Quaternion quat;
  {
    boost::recursive_mutex::scoped_lock lock(ui_mutex_);
    quat.setRPY( DEG2RAD(ui.sb_roll->value()), DEG2RAD(ui.sb_pitch->value()), DEG2RAD(ui.sb_yaw->value()));
    pose.position.x = ui.sb_move_x->value();
    pose.position.y = ui.sb_move_y->value();
    pose.position.z = ui.sb_move_z->value();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
  }

  PlanningGroupData& gc = group_data_map_[current_group_name_];
  tf::Transform cur = toBulletTransform(pose);
  setNewEndEffectorPosition(gc, cur, collision_aware_);




  if(gc.good_ik_solution_)
  {
    interactive_marker_server_->setPose(current_group_name_, pose);
    interactive_marker_server_->applyChanges();

    //ros::Time startTime = ros::Time(ros::WallTime::now().toSec());
    planToEndEffectorState(gc, false, false);

    if (gc.trajectory_data_map_["planner"].has_joint_trajectory_)
    {
      FollowJointTrajectoryGoal goal;
      goal.trajectory = gc.trajectory_data_map_["planner"].joint_trajectory_;
      trajectory_msgs::JointTrajectoryPoint last = goal.trajectory.points.back();
      goal.trajectory.points.clear();
      goal.trajectory.points.push_back(last);
      goal.trajectory.header.stamp = ros::Time::now(); // + dt;
      gc.arm_controller_->sendGoal(goal, boost::bind(&PRW::controllerDoneCallback, this, _1, _2));
      arm_is_moving_ = true;
      //ROS_INFO_STREAM("Total time " << dt);
    }





  }
  else
  {
    double ryp[3];
    tf::Transform good_state = gc.last_good_state_;
    tf::Matrix3x3(good_state.getRotation()).getRPY(ryp[0], ryp[1], ryp[2]);
    bool checked = ui.cb_real_time_update->isChecked();
    if(checked)
    {
      ui.cb_real_time_update->setChecked(false);
    }

    ui.sb_move_x->setValue(good_state.getOrigin().x());
    ui.sb_move_y->setValue(good_state.getOrigin().y());
    ui.sb_move_z->setValue(good_state.getOrigin().z());
    ui.sb_roll->setValue(RAD2DEG(ryp[0]));
    ui.sb_pitch->setValue(RAD2DEG(ryp[1]));
    ui.sb_yaw->setValue(RAD2DEG(ryp[2]));



    ui.cb_real_time_update->setChecked(checked);
    resetToLastGoodState(gc);
    interactive_marker_server_->applyChanges();
  }

}

void PRW::on_bt_reset_clicked()
{
  //qDebug() << __FUNCTION__;
  ui.sb_move_x->setValue(0.28);
  ui.sb_move_y->setValue(0.0);
  ui.sb_move_z->setValue(0.565);
  ui.sb_roll->setValue(0.0);
  ui.sb_pitch->setValue(0.0);
  ui.sb_yaw->setValue(0.0);
  if(ui.cb_real_time_update->isChecked())
    on_bt_go_clicked();
}

void PRW::endEffectorSlideUpdate()
{
  boost::recursive_mutex::scoped_lock lock(ui_mutex_);
  ui.sb_move_x->setValue(ui.hs_move_x->value() * 0.001 * ui.sb_move_x->maximum());
  ui.sb_move_y->setValue(ui.hs_move_y->value() * 0.001 * ui.sb_move_y->maximum());
  ui.sb_move_z->setValue(ui.hs_move_z->value() * 0.001 * ui.sb_move_z->maximum());
  ui.sb_roll->setValue(ui.hs_roll->value() * 0.001 * ui.sb_roll->maximum());
  ui.sb_pitch->setValue(ui.hs_pitch->value() * 0.001 * ui.sb_pitch->maximum());
  ui.sb_yaw->setValue(ui.hs_yaw->value() * 0.001 * ui.sb_yaw->maximum());
  if(ui.cb_real_time_update->isChecked())
    on_bt_go_clicked();
}

void PRW::endEffectorValueUpdate()
{
  boost::recursive_mutex::scoped_lock lock(ui_mutex_);
  ui.hs_move_x->setValue(ui.sb_move_x->value() / ui.sb_move_x->maximum() * 1000);
  ui.hs_move_y->setValue(ui.sb_move_y->value() / ui.sb_move_y->maximum() * 1000);
  ui.hs_move_z->setValue(ui.sb_move_z->value() / ui.sb_move_z->maximum() * 1000);
  ui.hs_roll->setValue(ui.sb_roll->value() / ui.sb_roll->maximum() * 1000);
  ui.hs_pitch->setValue(ui.sb_pitch->value() / ui.sb_pitch->maximum() * 1000);
  ui.hs_yaw->setValue(ui.sb_yaw->value() / ui.sb_yaw->maximum() * 1000);
  //if(ui.cb_real_time_update->isChecked())
    //on_bt_go_clicked();
}


void PRW::on_cb_enable_teleop_clicked()
{
  if(ui.cb_enable_teleop->isChecked())
  {
    ui.cb_teleop_arm->setEnabled(true);
    ui.cb_teleop_end_effector->setEnabled(true);
    ui.cb_teleop_tool->setEnabled(true);
  }
  else
  {
    ui.cb_teleop_arm->setChecked(false);
    ui.cb_teleop_end_effector->setChecked(false);
    ui.cb_teleop_tool->setChecked(false);
    ui.cb_teleop_arm->setEnabled(false);
    ui.cb_teleop_end_effector->setEnabled(false);
    ui.cb_teleop_tool->setEnabled(false);
  }
}


void PRW::on_new_scene_clicked()
{
  qDebug() << __FUNCTION__;
  //createNewPlanningScene("test", 0);
}

void PRW::on_new_mpr_clicked()
{
  qDebug() << __FUNCTION__;
  //unsigned int plan_id = 0;
  //unsigned int id;
  //createMotionPlanRequest(*robot_state_, *robot_state_,
    //                      "arm", "link5", plan_id, true, id);

}

void PRW::on_plan_clicked()
{
  qDebug() << __FUNCTION__;

}


void PRW::closeEvent(QCloseEvent *event)
{
  ROS_INFO("Close windows");
  quit_threads_ = true;
  event->accept();
}

void PRW::jointStateCallback(const sensor_msgs::JointStateConstPtr& message)
{
  if (robot_state_ == NULL)
    return;

  std::map<std::string, double> joint_state_map;
  std::map<std::string, double> joint_velocity_map;

  //message already been validated in kmsm
  if (message->velocity.size() == message->position.size())
  {
    for (unsigned int i = 0; i < message->position.size(); ++i)
    {
      joint_state_map[message->name[i]] = message->position[i];
      joint_velocity_map[message->name[i]] = message->velocity[i];
    }
  }
  else
  {
    for (unsigned int i = 0; i < message->position.size(); ++i)
    {
      joint_state_map[message->name[i]] = message->position[i];
      joint_velocity_map[message->name[i]] = 0.0;
    }
  }

  //lockScene();
  std::vector<KinematicState::JointState*>& joint_state_vector = robot_state_->getJointStateVector();
  std::vector<KinematicState::JointState*>::iterator it;
  for (it = joint_state_vector.begin(); it != joint_state_vector.end(); it++)
  {
    //see if we need to update any transforms
    std::string parent_frame_id = (*it)->getParentFrameId();
    std::string child_frame_id = (*it)->getChildFrameId();
    ROS_DEBUG_STREAM_THROTTLE(1.0, parent_frame_id);
    ROS_DEBUG_STREAM_THROTTLE(1.0, child_frame_id);

    if (!parent_frame_id.empty() && !child_frame_id.empty())
    {
      std::string err;
      ros::Time tm;
      tf::StampedTransform transf;
      bool ok = false;
      if (transform_listener_.getLatestCommonTime(parent_frame_id, child_frame_id, tm, &err) == tf::NO_ERROR)
      {
        ok = true;
        try
        {
          transform_listener_.lookupTransform(parent_frame_id, child_frame_id, tm, transf);
        }
        catch (tf::TransformException& ex)
        {
          ROS_ERROR("Unable to lookup transform from %s to %s.  Exception: %s", parent_frame_id.c_str(), child_frame_id.c_str(), ex.what());
          ok = false;
        }
      }
      else
      {
        ROS_DEBUG("Unable to lookup transform from %s to %s: no common time.", parent_frame_id.c_str(), child_frame_id.c_str());
        ok = false;
      }
      if (ok)
      {
        (*it)->setJointStateValues(transf);
      }
    }
    (*it)->setJointStateValues(joint_state_map);
  }
  robot_state_->updateKinematicLinks();
  robot_state_->getKinematicStateValues(robot_state_joint_values_);
}

void PRW::gestureCallback(const std_msgs::StringConstPtr& message)
{
  if(ui.cb_enable_gesture->isChecked())
  {
    QString gesture_string = message->data.c_str();
    QStringList gesture_list = gesture_string.split("_");
    Q_FOREACH(QString gesture, gesture_list)
    {
      //ROS_INFO_STREAM(gesture.toStdString());
      if(gesture.contains("MoveThreeAxis"))
      {
        QStringList move_list = gesture.split(":");
        //qDebug() << move_list;
        if(move_list.size() == 3)
        {
          boost::recursive_mutex::scoped_lock lock(ui_mutex_);

          /*
          //Right hand
          if(move_list[1].indexOf("X") != -1)
          {

          }
          if(move_list[1].indexOf("Y") != -1)
          {

          }
          if(move_list[1].indexOf("Z") != -1)
          {

          }
          */
          /*
          if(!move_list[1].isEmpty())
          {
            ROS_WARN_THROTTLE(1.0, "put right hand in center position");
            return;
          }
          */


          if(move_list[1] != move_list[2])
          {
            ROS_WARN_THROTTLE(1.0, "both hands need to be synchronized");
            return;
          }


          //Left hand
          int index;
          double value;
          if((index = move_list[2].indexOf("X")) != -1)
          {
            value = ui.sb_move_y->value();
            value += (move_list[2].at(index-1) == '+') ? 0.001 : -0.001;
            ui.sb_move_y->setValue(value);
          }
          if((index = move_list[2].indexOf("Y")) != -1)
          {
            value = ui.sb_move_z->value();
            value += (move_list[2].at(index - 1) == '+') ? 0.001 : -0.001;
            ui.sb_move_z->setValue(value);
          }
          if((index = move_list[2].indexOf("Z")) != -1)
          {
            value = ui.sb_move_x->value();
            value += (move_list[2].at(index - 1) == '+') ? 0.001 : -0.001;
            ui.sb_move_x->setValue(value);
          }

          /*
          //Left hand
          if(move_list[2].indexOf("X") != -1)
          {

          }
          if(move_list[2].indexOf("Y") != -1)
          {

          }
          if(move_list[2].indexOf("Z") != -1)
          {

          }
          */

        }
        //ROS_INFO_STREAM("Move axis: " << move_list);




      }

    }



    //ROS_INFO_STREAM(1.0, gesture->data);
  }
}

void PRW::processIKControllerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  //ROS_INFO("%s %d", __FUNCTION__, feedback->event_type);
  PlanningGroupData& gc = group_data_map_[current_group_name_];
  switch (feedback->event_type)
  {
    case InteractiveMarkerFeedback::BUTTON_CLICK: break;
    case InteractiveMarkerFeedback::MENU_SELECT: break;
    case InteractiveMarkerFeedback::MOUSE_UP:
    {
      /*
      if(arm_is_moving_)
      {
        ROS_WARN("Arm is moving!");
        break;
      }

      tf::Transform cur = toBulletTransform(feedback->pose);
      setNewEndEffectorPosition(gc, cur, collision_aware_);
      if(gc.good_ik_solution_)
      {
        //ros::Time startTime = ros::Time(ros::WallTime::now().toSec());
        planToEndEffectorState(gc, false, false);
        filterPlannerTrajectory(gc, false, false);
        //ros::Duration dt = ros::Time(ros::WallTime::now().toSec()) - startTime;


        if(gc.trajectory_data_map_["filter"].has_joint_trajectory_)
        {
          FollowJointTrajectoryGoal goal;
          goal.trajectory = gc.trajectory_data_map_["filter"].joint_trajectory_;


          goal.trajectory.header.stamp = ros::Time::now();// + dt;
          gc.arm_controller_->sendGoal(goal, boost::bind(&PRW::controllerDoneCallback, this, _1, _2));
          arm_is_moving_ = true;
          //ROS_INFO_STREAM("Total time " << dt);
        }
      }
      */
      break;
    }
    case InteractiveMarkerFeedback::MOUSE_DOWN:
      {
        std::map<std::string, SelectableMarker>::iterator it = selectable_markers_.begin();
        while (it != selectable_markers_.end())
        {
          if((it->second.type_ == CollisionObjectMarker) && (it->first != feedback->marker_name))
          {
            deselectMarker(it->second, toBulletTransform(it->second.pose_));
          }
          it++;
        }
      }
      break;
    case InteractiveMarkerFeedback::POSE_UPDATE:

      if(is_ik_control_active_ && isGroupName(feedback->marker_name))
      {
        tf::Transform cur = toBulletTransform(feedback->pose);
        static tf::Transform last_tf = cur;

        if(!(last_tf == cur))
        {
          if(arm_is_moving_)
          {
            interactive_marker_server_->setPose(feedback->marker_name, toGeometryPose(last_tf));
            break;
          }

          setNewEndEffectorPosition(gc, cur, collision_aware_);

          if (gc.good_ik_solution_)
          {
            planToEndEffectorState(gc, false, false);
            if (gc.trajectory_data_map_["planner"].has_joint_trajectory_)
            {
              FollowJointTrajectoryGoal goal;
              goal.trajectory = gc.trajectory_data_map_["planner"].joint_trajectory_;

              trajectory_msgs::JointTrajectoryPoint last = goal.trajectory.points.back();
              goal.trajectory.points.clear();
              goal.trajectory.points.push_back(last);


              goal.trajectory.header.stamp = ros::Time::now(); // + dt;
              gc.arm_controller_->sendGoal(goal, boost::bind(&PRW::controllerDoneCallback, this, _1, _2));
            }
          }


          last_tf = cur;
        }
      }
      else if (is_joint_control_active_ && feedback->marker_name.rfind("_joint_control") != string::npos)
      {
      }
      break;
    case InteractiveMarkerFeedback::KEEP_ALIVE: break;
  }
  interactive_marker_server_->applyChanges();
}

void PRW::processMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  ROS_INFO("%s %d", __FUNCTION__, feedback->event_type);
  PlanningGroupData& gc = group_data_map_[current_group_name_];
  switch (feedback->event_type)
  {
    case InteractiveMarkerFeedback::BUTTON_CLICK: break;
    case InteractiveMarkerFeedback::MENU_SELECT:
    {
      MenuHandler::EntryHandle handle;
      ROS_INFO_STREAM("Selected " << feedback->marker_name);
      if(is_ik_control_active_ && isGroupName(feedback->marker_name))
      {
        handle = feedback->menu_entry_id;
        if(handle == menu_ee_last_good_state_)
        {
          ROS_INFO_STREAM("Set group " << current_group_name_ << " to last good state");
          resetToLastGoodState(gc);
        }
        else if(handle == menu_ee_start_state_)
        {

        }
      }






      break;
    }
    case InteractiveMarkerFeedback::MOUSE_UP: break;
    case InteractiveMarkerFeedback::MOUSE_DOWN: break;
    case InteractiveMarkerFeedback::POSE_UPDATE: break;
  }
  interactive_marker_server_->applyChanges();
}

void PRW::processMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{

}

void PRW::sendPlanningScene()
{
  ROS_INFO("Sending Planning Scene....");
  lockScene();

  arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;
  arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;

  //get current robot state
  convertKinematicStateToRobotState(*robot_state_, ros::Time::now(), cm_->getWorldFrameId(),
                                    planning_scene_req.planning_scene_diff.robot_state);

  KinematicState* startState = NULL;
  KinematicState* endState = NULL;
  map<string, double> startStateValues;
  map<string, double> endStateValues;

  if (current_group_name_ != "")
  {
   startState = group_data_map_[current_group_name_].start_state_;
   endState = group_data_map_[current_group_name_].end_state_;

   if (startState != NULL)
   {
     startState->getKinematicStateValues(startStateValues);
   }
   if (endState != NULL)
   {
     endState->getKinematicStateValues(endStateValues);
   }
  }

  //delete stored state
  PlanningGroupDataMap::iterator it;
  for (it = group_data_map_.begin(); it != group_data_map_.end(); it++)
  {
    it->second.reset();
  }

  if (robot_state_ != NULL)
  {
    ROS_INFO("Reverting planning scene to default.");
    cm_->revertPlanningScene(robot_state_);
    robot_state_ = NULL;
  }

  //request planning scene
  if (!get_planning_scene_client_.call(planning_scene_req, planning_scene_res))
  {
    ROS_WARN("Can't get planning scene");
    unlockScene();
    return;
  }

  //get current robot state
  robot_state_ = cm_->setPlanningScene(planning_scene_res.planning_scene);
  if (robot_state_ == NULL)
  {
    ROS_ERROR("Something wrong with planning scene");
    unlockScene();
    return;
  }

  //update current group information
  if (current_group_name_ != "")
  {
    ROS_INFO("Resetting state...");
    group_data_map_[current_group_name_].setState(StartPosition, new KinematicState(robot_state_->getKinematicModel()));
    group_data_map_[current_group_name_].setState(EndPosition, new KinematicState(robot_state_->getKinematicModel()));

    startState = group_data_map_[current_group_name_].start_state_;
    endState = group_data_map_[current_group_name_].end_state_;

    if (startState != NULL)
    {
      //update state with latest joint value
      startState->setKinematicState(robot_state_joint_values_);
    }

    if (endState != NULL)
    {
      //use previously saved joint value
      endState->setKinematicState(endStateValues);
    }
  }

  unlockScene();
  ROS_INFO("Planning scene sent.");
}

void PRW::selectPlanningGroup(int group)
{
  ROS_INFO("Selecting planning group %u", group);
  lockScene();

  vector<string> names;
  for (PlanningGroupDataMap::iterator it = group_data_map_.begin(); it != group_data_map_.end(); it++)
  {
    names.push_back(it->first);
  }

  string old_group_name = current_group_name_;

  current_group_name_ = names[group];

  //use old state is exist
  if (group_data_map_[current_group_name_].end_state_ != NULL)
  {
    group_data_map_[current_group_name_].setState(EndPosition, new KinematicState(*group_data_map_[current_group_name_].end_state_));
  }
  else
  {
    group_data_map_[current_group_name_].setState(EndPosition, new KinematicState(*robot_state_));
  }

  group_data_map_[current_group_name_].setState(StartPosition, new KinematicState(*robot_state_));

  moveEndEffectorMarkers(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);


  if (is_ik_control_active_ && old_group_name != "" && selectableMarkerExists(old_group_name + "_selectable"))
  {
    PlanningGroupData& gc = group_data_map_[old_group_name];
    tf::Transform cur = robot_state_->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform();
    deselectMarker(selectable_markers_[old_group_name + "_selectable"], cur);

    if (is_joint_control_active_)
    {
      //deleteJointMarkers(group_map_[old_group_name]);
    }
  }
  else if (is_joint_control_active_)
  {
    ROS_WARN("NO IMPLEMENTATION %s", __FUNCTION__);
  }

  // Select the new planning group's marker.
  if (is_ik_control_active_ && selectableMarkerExists(current_group_name_ + "_selectable"))
  {
    PlanningGroupData& gc = group_data_map_[current_group_name_];
    tf::Transform cur = gc.end_state_->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform();
    selectMarker(selectable_markers_[current_group_name_ + "_selectable"], cur);
  }
  else if (is_joint_control_active_)
  {
    //GroupCollection& gc = group_map_[current_group_name_];
    //createSelectableJointMarkers(gc);
  }

  interactive_marker_server_->applyChanges();

  unlockScene();
  ROS_INFO("Planning group selected.");
}

PlanningGroupData* PRW::getPlanningGroup(unsigned int i)
{
  unsigned int cmd = 0;
  for (PlanningGroupDataMap::iterator it = group_data_map_.begin(); it != group_data_map_.end(); it++)
  {
    if (cmd == i)
    {
      return &(it->second);
    }
    cmd++;
  }
  return NULL;
}

void PRW::makeIKControllerMarker(tf::Transform transform, const std::string& name, const std::string& description,
                                 bool selectable, float scale, bool publish)
{

  SelectableMarker selectable_marker;
  selectable_marker.type_ = EndEffectorControlMarker;
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
  menu_handler_map_["End Effector"].apply(*interactive_marker_server_, marker.name);
  if(selectable)
  {
    selectable_markers_[marker.name] = selectable_marker;
  }

  if(publish)
  {
    interactive_marker_server_->applyChanges();
  }
}

void PRW::moveEndEffectorMarkers(double vx, double vy, double vz, double vr, double vp, double vw, bool coll_aware)
{
  lockScene();
  PlanningGroupData& gc = group_data_map_[current_group_name_];
  tf::Transform cur = gc.end_state_->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform();
  double mult = 0.1;

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

  setNewEndEffectorPosition(gc, cur, coll_aware);

  unlockScene();
}

void PRW::setNewEndEffectorPosition(PlanningGroupData& gc, tf::Transform& cur, bool coll_aware)
{
  //ROS_INFO_STREAM(gc.ik_link_name_);
  if (!gc.end_state_->updateKinematicStateWithLinkAt(gc.ik_link_name_, cur))
  {
    ROS_INFO_STREAM(__FUNCTION__ << " Problem");
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

bool PRW::solveIKForEndEffectorPose(PlanningGroupData& gc, bool coll_aware, bool constrain_pitch_and_roll,
                                    double change_redundancy)
{
  kinematics_msgs::PositionIKRequest ik_request;

  ik_request.ik_link_name = gc.ik_link_name_;
  ik_request.pose_stamped.header.frame_id = cm_->getWorldFrameId();
  ik_request.pose_stamped.header.stamp = ros::Time::now();
  tf::poseTFToMsg(gc.end_state_->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform(),
                  ik_request.pose_stamped.pose);
  convertKinematicStateToRobotState(*gc.end_state_, ros::Time::now(), cm_->getWorldFrameId(), ik_request.robot_state);
  ik_request.ik_seed_state = ik_request.robot_state;

  map<string, double> joint_values;
  vector<string> joint_names;

  if (coll_aware)
  {
    kinematics_msgs::GetConstraintAwarePositionIK::Request ik_req;
    kinematics_msgs::GetConstraintAwarePositionIK::Response ik_res;
    if (constrain_pitch_and_roll)
    {
      ROS_WARN("NO IMPLEMENTATION %s", __FUNCTION__);
      /*
      arm_navigation_msgs::Constraints goal_constraints;
      goal_constraints.orientation_constraints.resize(1);
      arm_navigation_msgs::Constraints path_constraints;
      path_constraints.orientation_constraints.resize(1);
      determinePitchRollConstraintsGivenState(gc, *gc.start_state_, goal_constraints.orientation_constraints[0],
                                              path_constraints.orientation_constraints[0]);
      arm_navigation_msgs::ArmNavigationErrorCodes err;
      if (!cm_->isKinematicStateValid(*gc.end_state_, std::vector<std::string>(), err, goal_constraints,
                                      path_constraints))
      {
        ROS_INFO_STREAM("Violates rp constraints");
        return false;
      }
      ik_req.constraints = goal_constraints;
      */
    } //constrain_pitch_and_roll

    ik_req.ik_request = ik_request;
    ik_req.timeout = ros::Duration(0.2);
    if (!gc.ik_collision_aware_client_.call(ik_req, ik_res))
    {
      ROS_INFO("Problem with ik service call");
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
    if (!gc.ik_non_collision_aware_client_.call(ik_req, ik_res))
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

  lockScene();
  gc.end_state_->setKinematicState(joint_values);
  unlockScene();

  //createSelectableJointMarkers(gc);

  if (coll_aware)
  {
    Constraints emp_con;
    ArmNavigationErrorCodes error_code;

    if (!cm_->isKinematicStateValid(*gc.end_state_, joint_names, error_code, emp_con, emp_con, true))
    {
      ROS_INFO_STREAM("Problem with response");
    }
  }

  return true;
}

void PRW::deselectMarker(SelectableMarker& marker, tf::Transform transform)
{
  if (!interactive_marker_server_->erase(marker.controlName_))
  {
    ROS_ERROR_STREAM("Cannot erase marker " << marker.controlName_);
    return;
  }

  switch (marker.type_)
  {
    case EndEffectorControlMarker:
      makeIKControllerMarker(transform, marker.controlName_, marker.controlDescription_, true, 0.5f);
      break;
    case CollisionObjectMarker:
      /*
      {
        makeSelectableCollisionObjectMarker(transform,
                                            marker.collision_object_type_,
                                            marker.controlName_,
                                            marker.color_,
                                            marker.a_, marker.b_, marker.c_);
      }
      */
      break;
    case JointControlMarker:
      //makeSelectableMarker(marker.type_, transform, marker.controlName_, marker.controlDescription_, 0.225);
      break;
  }
}

void PRW::selectMarker(SelectableMarker& marker, tf::Transform transform)
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
      case EndEffectorControlMarker:
        makeIKControllerMarker(transform, marker.controlName_, marker.controlDescription_, false, 0.5f);
        break;
      case CollisionObjectMarker:
        /*
        {
          double scale = std::max(marker.a_, std::max(marker.b_, marker.c_));
          makeInteractive6DOFMarker(false, transform, marker.controlName_, marker.controlDescription_, scale*2.0, true);
        }
        */
        break;
      case JointControlMarker:
        //makeInteractive6DOFMarker(false, transform, marker.controlName_, marker.controlDescription_, 0.225f, false);
        break;
    }
  }
}

void PRW::sendMarkers()
{
  //This function cause base_link , /base_link warning in rviz

  lockScene();
  MarkerArray arr;

  std_msgs::ColorRGBA stat_color_;
  stat_color_.a = 0.6;
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
    std_msgs::ColorRGBA bad_color;
    bad_color.a = 0.5;
    bad_color.r = 0.9;
    bad_color.g = 0.0;
    bad_color.b = 0.0;

    std_msgs::ColorRGBA collision_color;
    bad_color.a = 1.0;
    bad_color.r = 1.0;
    bad_color.g = 0.5;
    bad_color.b = 0.0;

    PlanningGroupData& gc = group_data_map_[current_group_name_];
    const KinematicModel* kinematic_model = cm_->getKinematicModel();

    if (is_ik_control_active_)
    {
      if (gc.end_state_ != NULL)
      {
        cm_->getGroupAndUpdatedJointMarkersGivenState(*gc.end_state_, arr, current_group_name_, gc.end_color_,
                                                      gc.start_color_, ros::Duration(0.1));
      }
      else
      {
        ROS_ERROR("End state invalid!");
      }
    }

    if (!gc.good_ik_solution_ && gc.end_state_ != NULL)
    {
      vector<string> lnames = kinematic_model->getChildLinkModelNames(kinematic_model->getLinkModel(gc.ik_link_name_));
      cm_->getRobotMarkersGivenState(*gc.end_state_, arr, bad_color, current_group_name_, ros::Duration(0.2), &lnames);
      cm_->getAllCollisionPointMarkers(*robot_state_, arr, bad_color, ros::Duration(.2));
    }

    for (TrajectoryDataMap::iterator it = gc.trajectory_data_map_.begin(); it != gc.trajectory_data_map_.end(); it++)
    {

      if (it->second.play_joint_trajectory_)
      {
        moveThroughTrajectory(gc, it->first, 5);
      }

      if (it->second.show_joint_trajectory_)
      {
        const vector<const KinematicModel::LinkModel*>& updated_links =
            kinematic_model->getModelGroup(gc.name_)->getUpdatedLinkModels();
        vector<string> lnames;
        lnames.resize(updated_links.size());
        for (unsigned int i = 0; i < updated_links.size(); i++)
        {
          lnames[i] = updated_links[i]->getName();
        }

        cm_->getAttachedCollisionObjectMarkers(*(it->second.state_), arr, it->first + "_trajectory", it->second.color_,
                                               ros::Duration(0.1));

        cm_->getRobotMarkersGivenState(*(it->second.state_), arr, it->second.color_, it->first + "_trajectory",
                                       ros::Duration(0.1), &lnames);

      }
    }
  }

  marker_array_publisher_.publish(arr);
  unlockScene();
}

void PRW::moveThroughTrajectory(PlanningGroupData& gc, const string& source_name, int step)
{
  lockScene();
  TrajectoryData& disp = gc.trajectory_data_map_[source_name];
  unsigned int tsize = disp.joint_trajectory_.points.size();
  if(tsize == 0 || disp.state_ == NULL)
  {
    unlockScene();
    return;
  }
  if((int)disp.current_trajectory_point_ + step < 0)
  {
    disp.current_trajectory_point_ = 0;
  }
  else
  {
    disp.current_trajectory_point_ = ((int)disp.current_trajectory_point_) + step;
  }
  if(disp.current_trajectory_point_ >= tsize - 1)
  {
    disp.current_trajectory_point_ = tsize - 1;
    disp.play_joint_trajectory_ = false;
    disp.show_joint_trajectory_ = false;
  }
  map<string, double> joint_values;
  for(unsigned int i = 0; i < disp.joint_trajectory_.joint_names.size(); i++)
  {
    joint_values[disp.joint_trajectory_.joint_names[i]]
        = disp.joint_trajectory_.points[disp.current_trajectory_point_].positions[i];
  }
  disp.state_->setKinematicState(joint_values);
  unlockScene();
}

bool PRW::planToEndEffectorState(PlanningGroupData& gc, bool show, bool play)
{
  MotionPlanRequest motion_plan_request;
  motion_plan_request.group_name = gc.name_;
  motion_plan_request.num_planning_attempts = 1;
  motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  if (!constrain_rp_)
  {
    const KinematicState::JointStateGroup* jsg = gc.end_state_->getJointStateGroup(gc.name_);
    motion_plan_request.goal_constraints.joint_constraints.resize(jsg->getJointNames().size());
    vector<double> joint_values;
    jsg->getKinematicStateValues(joint_values);
    for (unsigned int i = 0; i < jsg->getJointNames().size(); i++)
    {
      motion_plan_request.goal_constraints.joint_constraints[i].joint_name = jsg->getJointNames()[i];
      motion_plan_request.goal_constraints.joint_constraints[i].position = joint_values[i];
      motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.01;
      motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.01;
    }
  }
  else
  {
    ROS_WARN("NO IMPLEMENTATION %s", __FUNCTION__);
    /*
     motion_plan_request.group_name += "_cartesian";
     motion_plan_request.goal_constraints.position_constraints.resize(1);
     motion_plan_request.goal_constraints.orientation_const    raints.resize(1);
     geometry_msgs::PoseStamped end_effector_wrist_pose;
     tf::poseTFToMsg(gc.end_state_->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform(),
     end_effector_wrist_pose.pose);
     end_effector_wrist_pose.header.frame_id = cm_->getWorldFrameId();
     arm_navigation_msgs::poseStampedToPositionOrientationConstraints(
     end_effector_wrist_pose, gc.ik_link_name_, motion_plan_request.goal_constraints.position_constraints[0],
     motion_plan_request.goal_constraints.orientation_constraints[0]);
     motion_plan_request.path_constraints.orientation_constraints.resize(1);
     determinePitchRollConstraintsGivenState(gc, *robot_state_,
     motion_plan_request.goal_constraints.orientation_constraints[0],
     motion_plan_request.path_constraints.orientation_constraints[0]);
     */
  }

  convertKinematicStateToRobotState(*robot_state_, ros::Time::now(), cm_->getWorldFrameId(),
                                    motion_plan_request.start_state);

  GetMotionPlan::Request plan_req;
  plan_req.motion_plan_request = motion_plan_request;
  GetMotionPlan::Response plan_res;
  if (!planner_client_.call(plan_req, plan_res))
  {
    ROS_INFO("Something wrong with planner client");
    return false;
  }

  if (gc.trajectory_data_map_.find("planner") != gc.trajectory_data_map_.end())
  {
    lockScene(); //lock to from sendMarker()

    TrajectoryData& disp = gc.trajectory_data_map_["planner"];
    if (plan_res.error_code.val != plan_res.error_code.SUCCESS)
    {
      disp.trajectory_error_code_ = plan_res.error_code;
      ROS_INFO_STREAM("Bad planning error code " << plan_res.error_code.val);
      gc.trajectory_data_map_["planner"].reset();
      return false;
    }

    disp.reset();
    disp.joint_trajectory_ = plan_res.trajectory.joint_trajectory;
    disp.has_joint_trajectory_ = true;
    disp.show_joint_trajectory_ = show;
    disp.play_joint_trajectory_ = play;
    disp.state_ = new KinematicState(*robot_state_);

    vector<ArmNavigationErrorCodes> trajectory_error_codes;

    cm_->isJointTrajectoryValid(*disp.state_, disp.joint_trajectory_, last_motion_plan_request_.goal_constraints,
                                last_motion_plan_request_.path_constraints, disp.trajectory_error_code_,
                                trajectory_error_codes, false);

    if (disp.trajectory_error_code_.val != disp.trajectory_error_code_.SUCCESS)
    {
      disp.trajectory_bad_point_ = trajectory_error_codes.size() - 1;
    }
    else
    {
      disp.trajectory_bad_point_ = -1;
    }

    last_motion_plan_request_ = motion_plan_request;

    unlockScene();
    return true;
  }
  else
  {
    return false;
  }
}

bool PRW::filterPlannerTrajectory(PlanningGroupData& gc, bool show, bool play)
{
  FilterJointTrajectoryWithConstraints::Request filter_req;
  FilterJointTrajectoryWithConstraints::Response filter_res;

  convertKinematicStateToRobotState(*robot_state_, ros::Time::now(), cm_->getWorldFrameId(), filter_req.start_state);
  TrajectoryData& planner_disp = gc.trajectory_data_map_["planner"];
  filter_req.trajectory = planner_disp.joint_trajectory_;
  filter_req.group_name = gc.name_;

  filter_req.goal_constraints = last_motion_plan_request_.goal_constraints;
  filter_req.path_constraints = last_motion_plan_request_.path_constraints;

  if(filter_req.trajectory.points.size() < 10)
  {
    ROS_INFO_THROTTLE(1.0, "Hacked for short path");
    filter_req.allowed_time = ros::Duration(0.01);
  }
  else
    filter_req.allowed_time = ros::Duration(0.1);


  ros::Time startTime = ros::Time(ros::WallTime::now().toSec());
  if (!trajectory_filter_client_.call(filter_req, filter_res))
  {
    ROS_INFO("Problem with trajectory filter");
    gc.trajectory_data_map_["filter"].reset();
    return false;
  }
  ROS_INFO_STREAM(ros::Time(ros::WallTime::now().toSec()) - startTime);

  lockScene();

  TrajectoryData& filter_disp = gc.trajectory_data_map_["filter"];
  if (filter_res.error_code.val != filter_res.error_code.SUCCESS)
  {
    filter_disp.trajectory_error_code_ = filter_res.error_code;
    ROS_INFO_STREAM("Bad trajectory_filter error code " << filter_res.error_code.val);
    gc.trajectory_data_map_["filter"].reset();
    return false;
  }

  filter_disp.reset();
  filter_disp.joint_trajectory_ = filter_res.trajectory;
  filter_disp.has_joint_trajectory_ = true;
  filter_disp.show_joint_trajectory_ = show;
  filter_disp.play_joint_trajectory_ = play;
  filter_disp.state_ = new KinematicState(*robot_state_);

  vector<ArmNavigationErrorCodes> trajectory_error_codes;

  cm_->isJointTrajectoryValid(*filter_disp.state_, filter_disp.joint_trajectory_,
                              last_motion_plan_request_.goal_constraints, last_motion_plan_request_.path_constraints,
                              filter_disp.trajectory_error_code_, trajectory_error_codes, false);

  if (filter_disp.trajectory_error_code_.val != filter_disp.trajectory_error_code_.SUCCESS)
  {
    filter_disp.trajectory_bad_point_ = trajectory_error_codes.size() - 1;
  }
  else
  {
    filter_disp.trajectory_bad_point_ = -1;
  }

  unlockScene();

  return true;
}

void PRW::controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                                 const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  ROS_INFO("trajectory done");
  arm_is_moving_ = false;
}

void PRW::makeMenu()
{
  // Allocate memory to each of the menu entry maps.
  menu_entry_maps_["End Effector"] = MenuEntryMap();
  menu_entry_maps_["End Effector Selection"] = MenuEntryMap();
  menu_entry_maps_["Top Level"] = MenuEntryMap();
  menu_entry_maps_["Collision Object"] = MenuEntryMap();
  menu_entry_maps_["Collision Object Selection"] = MenuEntryMap();

  // Allocate memory to the menu handlers
  menu_handler_map_["End Effector"];
  menu_handler_map_["End Effector Selection"];
  menu_handler_map_["Top Level"];
  menu_handler_map_["Collision Object"];
  menu_handler_map_["Collision Object Selection"];


  //end effector
  menu_ee_last_good_state_ = registerMenuEntry(menu_handler_map_["End Effector"], menu_entry_maps_["End Effector"],
                                               "Go to last good state");



  //always register as the last entry for end effector menu
  menu_ee_start_state_ = registerMenuEntry(menu_handler_map_["End Effector"], menu_entry_maps_["End Effector"],
                                               "Go to start state");



  //Top level
  registerMenuEntry(menu_handler_map_["Top Level"], menu_entry_maps_["Top Level"], "Create Pole");






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

MenuHandler::EntryHandle PRW::registerMenuEntry(interactive_markers::MenuHandler& handler, MenuEntryMap& map, std::string name)
{
  MenuHandler::EntryHandle toReturn = handler.insert(name, process_menu_feedback_ptr_);
  map[toReturn] = name;
  return toReturn;
}

void PRW::resetToLastGoodState(PlanningGroupData& gc)
{
  setNewEndEffectorPosition(gc, gc.last_good_state_, collision_aware_);
  if (is_ik_control_active_)
  {
    selectMarker(selectable_markers_[current_group_name_ + "_selectable"],
                 gc.end_state_->getLinkState(gc.ik_link_name_)->getGlobalLinkTransform());
  }
}
