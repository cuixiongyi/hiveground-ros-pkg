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
#include <QtGui/QApplication>
#include <qevent.h>
#include <qfiledialog.h>
#include <qmessagebox.h>

#include <ros/ros.h>


#include <boost/thread.hpp>


#include <hg_inspector_arm/inspector_arm.h>


InspectorArm::InspectorArm(QWidget *parent, Qt::WFlags flags)
  : QMainWindow(parent, flags),
    quit_thread_(false),
    nh_(), nh_private_("~"),
    marker_server_("inspection_point_marker")
{
  ui.setupUi(this);
}

InspectorArm::~InspectorArm()
{

}

bool InspectorArm::run()
{
  return PlanningBase::run();
}


bool InspectorArm::initialize(const std::string& param_server_prefix)
{
  if(!PlanningBase::initialize(param_server_prefix))
     return false;

  hg_cartesian_trajectory::KinematicModelGroupConfigMap::const_iterator it;
  const hg_cartesian_trajectory::KinematicModelGroupConfigMap &group_config_map =
      collision_models_interface_->getKinematicModel()->getJointModelGroupConfigMap();
  for (it = group_config_map.begin(); it != group_config_map.end(); it++)
  {
    std::string arm_controller_name = collision_models_interface_->getKinematicModel()->getRobotName()
        + "/follow_joint_trajectory";
    action_client_map_[it->first] = FollowJointTrajectoryClientPtr(
        new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(arm_controller_name, true));
    while (ros::ok() && !action_client_map_[it->first]->waitForServer(ros::Duration(1.0))) { }
  }

  if(!initializePropertyEditor()) return false;
  if(!initializeInteractiveMarkerServer()) return false;
  if(!initializeServiceClient()) return false;

  connect(this, SIGNAL(followPointSignal()), this, SLOT(followPointSlot()));

  return true;
}

bool InspectorArm::initializeServiceClient()
{
  robot_state_ = NULL;
  joint_state_subscriber_ = nh_.subscribe("joint_states", 1, &InspectorArm::jointStateCallback, this);


  ros::service::waitForService("plan_cartesian_path");
  hg_cartesian_trajectory_client_ =
      nh_.serviceClient<hg_cartesian_trajectory::HgCartesianTrajectory>("plan_cartesian_path");
  return true;
}

void InspectorArm::jointStateCallback(const sensor_msgs::JointStateConstPtr& message)
{
  joint_state_mutex_.lock();
  latest_joint_state_ = *message;
  joint_state_mutex_.unlock();
}

void InspectorArm::controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                                               const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  ROS_INFO("trajectory done");
}

void InspectorArm::on_pushButtonAddInspectionPoint_clicked()
{

}

void InspectorArm::on_pushButtonPlan_clicked()
{
  if(markers_.size() == 0)
    return;

  //on_pushButtonPlan_clicked();
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.header.stamp = ros::Time::now();
  goal.trajectory.joint_names = markers_.begin()->second->jointState().name;

  std::map<std::string, InspectionPointItem*>::iterator it = markers_.begin();
  trajectory_msgs::JointTrajectoryPoint point;
  while (it != markers_.end())
  {
    point.positions = it->second->jointState().position;
    goal.trajectory.points.push_back(point);
    it++;
  }

  action_client_map_["manipulator"]->sendGoal(goal, boost::bind(&InspectorArm::controllerDoneCallback, this, _1, _2));

#if 0
  hg_cartesian_trajectory::HgCartesianTrajectoryRequest request;
  hg_cartesian_trajectory::HgCartesianTrajectoryResponse respond;
  request.header.frame_id = "/base_link";
  request.header.stamp = ros::Time::now();
  request.motion_plan_request.group_name = "manipulator";
  joint_state_mutex_.lock();
  request.motion_plan_request.start_state.joint_state = latest_joint_state_;
  joint_state_mutex_.unlock();
  request.poses.clear();
  std::map<std::string, InspectionPointItem*>::iterator it = markers_.begin();
  while(it != markers_.end())
  {
    request.poses.push_back(it->second->pose());
    it++;
  }

  if(!hg_cartesian_trajectory_client_.call(request, respond))
  {
    ROS_ERROR("error when calling calling plan_cartesian_path: error code %d", respond.error_code.val);
  }
  else
  {
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = respond.trajectory.joint_trajectory;
    action_client_map_[request.motion_plan_request.group_name]->sendGoal(goal,
                                                                         boost::bind(&InspectorArm::controllerDoneCallback, this, _1, _2));
  }
#endif
}

void InspectorArm::on_actionAddMarker_triggered()
{
  //ROS_INFO(__FUNCTION__);
  addMarkerAtEndEffector();
}

void InspectorArm::on_actionClearMarker_triggered()
{
  //ROS_INFO(__FUNCTION__);
  clearMarker();
}

void InspectorArm::on_actionLoadMarker_triggered()
{
  if(markers_touched_)
  {
    int ret = QMessageBox::warning(this, tr("Inspector Arm"),
                                    tr("The marker has been modified.\n"
                                       "Do you want to save your changes?"),
                                    QMessageBox::Save | QMessageBox::Discard
                                    | QMessageBox::Cancel,
                                    QMessageBox::Save);
    if(ret == QMessageBox::Save)
      on_actionSaveMarker_triggered();
  }
  markers_save_file_name_ = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                           "",
                                                           tr("Marker (*.mrk)"));
  loadMarker();
}

void InspectorArm::on_actionSaveMarker_triggered()
{
  if(markers_save_file_name_.isEmpty())
    on_actionSaveMarkerAs_triggered();
  else
    saveMarker();
}

void InspectorArm::on_actionSaveMarkerAs_triggered()
{
   QString file_name = QFileDialog::getSaveFileName(this, tr("Save File"),
                                                         markers_save_file_name_,
                                                         tr("Marker (*.mrk)"));
   markers_save_file_name_ = file_name;
   if(!markers_save_file_name_.isEmpty())
     saveMarker();
}


void InspectorArm::followPointSlot()
{
  //ROS_INFO(__FUNCTION__);
  if(ui.checkBoxFollowPoint->isChecked())
  {
    if(markers_.size() == 1)
    {
      //on_pushButtonPlan_clicked();
      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory.header.stamp = ros::Time::now();
      goal.trajectory.joint_names = markers_.begin()->second->jointState().name;
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions = markers_.begin()->second->jointState().position;
      goal.trajectory.points.push_back(point);
      action_client_map_["manipulator"]->sendGoal(goal, boost::bind(&InspectorArm::controllerDoneCallback, this, _1, _2));
    }
  }
}

void InspectorArm::closeEvent(QCloseEvent *event)
{
  quit_thread_ = true;
  event->accept();
}

InspectorArm* g_inspector_arm = NULL;
bool g_initialized = false;

void spin_function()
{
  ros::WallRate r(100.0);
  while (ros::ok() && !g_initialized)
  {
    r.sleep();
    ros::spinOnce();
  }
  while (ros::ok() && !g_inspector_arm->quit_thread_)
  {
    r.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hg_inspection_arm", ros::init_options::NoSigintHandler);
  boost::thread spin_thread(boost::bind(&spin_function));

  QApplication a(argc, argv);

  InspectorArm arm;
  g_inspector_arm = &arm;
  arm.show();

  g_initialized = arm.run();

  if(!g_initialized)
    exit(-1);

  int ret = a.exec();

  spin_thread.join();

  return ret;
}

