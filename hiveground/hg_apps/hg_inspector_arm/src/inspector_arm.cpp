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

#include <pcl/common/pca.h>
#include <Eigen/StdVector>

#include <planning_models/kinematic_state.h>
#include <hg_inspector_arm/inspector_arm.h>
#include <spline_smoother/cubic_trajectory.h>

using namespace visualization_msgs;
using namespace hg_user_interaction;


InspectorArm::InspectorArm(QWidget *parent, Qt::WFlags flags)
  : QMainWindow(parent, flags),
    PlanningBase(),
    quit_thread_(false),
    marker_server_("inspection_point_marker"),
    arm_is_active_(false)
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

  bool b; double d; int i;
  nh_private_.getParam("look_at_distance", d);
  ui.doubleSpinBoxLookAtDistance->setValue(d);

  nh_private_.getParam("mouse3d_translation_scale", d);
  ui.doubleSpinBoxLinearScale->setValue(d);

  nh_private_.getParam("mouse3d_ratation_scale", d);
  ui.doubleSpinBoxAngularScale->setValue(d);

  nh_private_.getParam("gesture_translation_scale", d);
  ui.doubleSpinBoxGestureTranslationScale->setValue(d);

  nh_private_.getParam("gesture_rotation_scale", d);
  ui.doubleSpinBoxGestureRotationScale->setValue(d);



  KinematicModelGroupConfigMap::const_iterator it;
  const KinematicModelGroupConfigMap &group_config_map =
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
  robot_state_ = new planning_models::KinematicState(collision_models_interface_->getKinematicModel());
  target_robot_state_ = new planning_models::KinematicState(collision_models_interface_->getKinematicModel());
  start_color_.a = 0.5;
  start_color_.r = 1.0;
  start_color_.g = 0.5;
  start_color_.b = 1.0;
  end_color_.a = 0.3;
  end_color_.r = 0.5;
  end_color_.g = 0.9;
  end_color_.b = 0.5;
  stat_color_.a = 0.6;
  stat_color_.r = 0.1;
  stat_color_.g = 0.8;
  stat_color_.b = 0.3;
  attached_color_.a = 1.0;
  attached_color_.r = 0.6;
  attached_color_.g = 0.4;
  attached_color_.b = 0.3;
  bad_color_.a = 0.5;
  bad_color_.r = 0.9;
  bad_color_.g = 0.0;
  bad_color_.b = 0.0;
  collision_color_.a = 1.0;
  collision_color_.r = 1.0;
  collision_color_.g = 0.5;
  collision_color_.b = 0.0;

  joint_state_subscriber_ = nh_.subscribe("joint_states", 1, &InspectorArm::jointStateCallback, this);
  hands_subscriber_ = nh_.subscribe("hands_message", 1, &InspectorArm::handsCallBack, this);
  hand_gestures_subscriber_ = nh_.subscribe("hand_gestures_message", 1, &InspectorArm::handGestureCallBack, this);
  space_navigator_subscriber_ = nh_.subscribe("space_navigator_message", 1, &InspectorArm::spaceNavigatorCallBack, this);

  marker_array_publisher_ = nh_private_.advertise<MarkerArray>("marker_array", 128);

//  ros::service::waitForService("plan_cartesian_path");
//  hg_cartesian_trajectory_client_ =
//      nh_.serviceClient<hg_cartesian_trajectory::HgCartesianTrajectory>("plan_cartesian_path");

  marker_array_publisher_timer_ = new QTimer();
  marker_array_publisher_timer_->start(100);
  connect(marker_array_publisher_timer_, SIGNAL(timeout()), this, SLOT(onMarkerArrayPublisherTimer()));

  return true;
}

void InspectorArm::jointStateCallback(const sensor_msgs::JointStateConstPtr& message)
{
  QMutexLocker lock(&mutex_joint_state_);
  latest_joint_state_ = *message;
}

void InspectorArm::controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                                               const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  //ROS_INFO("trajectory done");
  arm_is_active_ = false;
}


void InspectorArm::handsCallBack(const hg_object_tracking::HandsConstPtr message)
{
  if(message->hands.empty()) return;

  if(ui.checkBoxFollowHand->isChecked())
  {
    tf::Transform hand_left;
    tf::transformMsgToTF(message->hands[0].hand_centroid, hand_left);
    tf::StampedTransform from;
    listener_.lookupTransform(world_frame_, "link2", ros::Time(0), from);
    tf::Transform origin(tf::Quaternion(0, 0, 0, 1), from.getOrigin());
    tf::Transform new_ee_pose;
    lookAt(hand_left.getOrigin(), origin, ui.doubleSpinBoxLookAtDistance->value() , new_ee_pose);
    sensor_msgs::JointState joint_state;
    if (checkIKConstraintAware(new_ee_pose, joint_state))
    {
      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory.header.stamp = ros::Time::now();
      goal.trajectory.joint_names = ik_solver_info_.kinematic_solver_info.joint_names;
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions = joint_state.position;
      goal.trajectory.points.push_back(point);
      action_client_map_["manipulator"]->sendGoal(goal, boost::bind(&InspectorArm::controllerDoneCallback, this, _1, _2));
      arm_is_active_ = true;
    }
  }
}


void InspectorArm::handGestureCallBack(const hg_user_interaction::GesturesConstPtr message)
{
  ROS_INFO_THROTTLE(1,0, __FUNCTION__);
  if(ui.checkBoxEnableGestureControl->isChecked())
  {
    if(ui.checkBoxEnablePushPullGesture->isChecked())
    {
      if(markers_.find(selected_markers_.back()) != markers_.end())
      {
        for(size_t i = 0; i < message->gestures.size(); i++)
        {
          if(message->gestures[i].type == Gesture::GESTURE_HAND_PUSH_PULL)
          {
            tf::Transform pose;
            tf::poseMsgToTF(markers_[selected_markers_.back()]->pose(), pose);
            double rx, ry, rz;
            rx = ry = rz = 0;

            //message->gestures[i].hand_count
            double hand_distance = 0.0;
            for(int j = 0; j < message->gestures[i].hand_count; j++)
            {
              hand_distance += message->gestures[i].vars[j];
            }
            hand_distance /= message->gestures[i].hand_count;
            ROS_INFO("haha %f", hand_distance);

            double translation_scale = ui.doubleSpinBoxGestureTranslationScale->value() * hand_distance;
            double rotation_scale = ui.doubleSpinBoxGestureRotationScale->value() * hand_distance;
            switch(message->gestures[i].direction)
            {
              case Gesture::DIR_X_POS:
                pose.getOrigin().m_floats[0] += translation_scale;
                break;
              case Gesture::DIR_X_NEG:
                pose.getOrigin().m_floats[0] -= translation_scale;
                break;
              case Gesture::DIR_Y_POS:
                pose.getOrigin().m_floats[1] += translation_scale;
                break;
              case Gesture::DIR_Y_NEG:
                pose.getOrigin().m_floats[1] -= translation_scale;
                break;
              case Gesture::DIR_Z_POS:
                pose.getOrigin().m_floats[2] += translation_scale;
                break;
              case Gesture::DIR_Z_NEG:
                pose.getOrigin().m_floats[2] -= translation_scale;
                break;
              case Gesture::ROT_X_POS: rx = rotation_scale; break;
              case Gesture::ROT_X_NEG: rx = -rotation_scale; break;
              case Gesture::ROT_Y_POS: ry = rotation_scale; break;
              case Gesture::ROT_Y_NEG: ry = -rotation_scale; break;
              case Gesture::ROT_Z_POS: rz = rotation_scale; break;
              case Gesture::ROT_Z_NEG: rz = -rotation_scale; break;
                break;
              default: break;
            }

            tf::Quaternion q;
            q.setRPY(rx, ry, rz);
            pose.setRotation(pose.getRotation() * q);
            markers_[selected_markers_.back()]->setPose(pose, true);
          }
        }
      }
    }
  }
}




void InspectorArm::spaceNavigatorCallBack(const geometry_msgs::TwistConstPtr message)
{
  if((ros::Time::now() - space_navigator_last_update_).toSec() < 0.05)
  {
    return;
  }
  else
  {
    space_navigator_last_update_ = ros::Time::now();
  }

  //ROS_INFO_STREAM_THROTTLE(1.0, *message);
  if(!ui.checkBox3DMouse->isChecked()) return;


  if(markers_.find(selected_markers_.back()) != markers_.end())
  {
    tf::Vector3 linear;
    tf::vector3MsgToTF(message->linear, linear);
    tf::Vector3 angular;
    tf::vector3MsgToTF(message->angular, angular);

    double l1 = linear.length2();
    double l2 = angular.length2();

    if((l1 == 0.0) & (l2 == 0.0))
      return;

    bool translate = false;
    if(l1 > l2)
      translate = true;

    linear.setX(-linear.x());
    linear.setY(-linear.y());

    //find max axis

    linear = linear * (1/350.0) * ui.doubleSpinBoxLinearScale->value();
    angular = angular * (1/350.0) * ui.doubleSpinBoxAngularScale->value();

    tf::Transform pose;
    tf::poseMsgToTF(markers_[selected_markers_.back()]->pose(), pose);


    if(translate)
    {
      if(ui.checkBox3DMouseTranslation->isChecked())
        pose.setOrigin(pose.getOrigin() + linear);
    }
    else
    {
      if(ui.checkBox3DMouseRotation->isChecked())
      {
        //update angular
        tf::Transform pose_angular;
        pose_angular.setOrigin(tf::Vector3(0, 0, 0));
        pose_angular.setRotation(pose.getRotation());

        tf::Transform tf;
        tf.setOrigin(tf::Vector3(0, 0, 0));
        tf::Quaternion q;
        if(ui.checkBoxSwapRxRz->isChecked())
          q.setRPY(angular.z(), angular.y(), angular.x());
        else
          q.setRPY(angular.x(), angular.y(), angular.z());
        tf.setRotation(q);
        pose_angular = tf * pose_angular;
        pose.setRotation(pose.getRotation() * q);
      }
    }


    if(ui.checkBox3DMouseTranslation->isChecked() || ui.checkBox3DMouseRotation->isChecked())
      markers_[selected_markers_.back()]->setPose(pose, true);
  }
}

void InspectorArm::lookAt(const tf::Vector3& at, const tf::Transform& from, double distance, tf::Transform& result)
{
  tf::Vector3 from_to_at = at - from.getOrigin();
  Eigen::Vector3f v(from_to_at.x(), from_to_at.y(), from_to_at.z());
  Eigen::Quaternionf q;
  q.setFromTwoVectors(Eigen::Vector3f(1, 0, 0), v.normalized());
  result.setOrigin((from_to_at.normalize() * -distance) + at);
  result.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  //ROS_INFO("(%6.3f, %6.3f, %6.3f) (%6.3f, %6.3f, %6.3f, %6.3f)",
           //result.getOrigin().x(), result.getOrigin().y(),result. getOrigin().z(),
           //result.getRotation().x(), result.getRotation().y(), result.getRotation().z(), result.getRotation().w());
}

void InspectorArm::on_pushButtonPlan_clicked()
{
  if(markers_.size() == 0)
    return;

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.header.stamp = ros::Time::now();
  goal.trajectory.joint_names = ik_solver_info_.kinematic_solver_info.joint_names;

  std::map<std::string, InspectionPointItem*>::iterator it = markers_.begin();
  trajectory_msgs::JointTrajectoryPoint point;
  //point.velocities.resize(ik_solver_info_.kinematic_solver_info.joint_names.size(), 0);
  while (it != markers_.end())
  {
    point.positions = it->second->jointState().position;
    goal.trajectory.points.push_back(point);
    it++;
  }

  action_client_map_["manipulator"]->sendGoal(goal, boost::bind(&InspectorArm::controllerDoneCallback, this, _1, _2));
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
    int ret = QMessageBox::warning(this,
                                    tr("Inspector Arm"),
                                    tr("The marker has been modified.\n"
                                        "Do you want to save your changes?"),
                                        QMessageBox::Save | QMessageBox::Cancel, QMessageBox::Save);

    if(ret == QMessageBox::Save)
      on_actionSaveMarker_triggered();
  }
  markers_save_file_name_ = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                           "",
                                                           tr("Marker (*.mrk)"));
  if(markers_save_file_name_.length()  == 0) return;
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
    if(markers_.find(selected_markers_.back()) != markers_.end())
    {
      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory.header.stamp = ros::Time::now();
      goal.trajectory.joint_names = ik_solver_info_.kinematic_solver_info.joint_names;
      trajectory_msgs::JointTrajectoryPoint point;
      if(ui.checkBoxLookAt->isEnabled() && ui.checkBoxLookAt->isChecked())
      {
        tf::StampedTransform from;
        listener_.lookupTransform(world_frame_, "link2", ros::Time(0), from);
        tf::Transform origin(tf::Quaternion(0, 0, 0, 1), from.getOrigin());
        tf::Transform new_ee_pose;
        lookAt(last_feedback_pose_.getOrigin(), origin, ui.doubleSpinBoxLookAtDistance->value(), new_ee_pose);
        sensor_msgs::JointState joint_state;

        if (checkIKConstraintAware(new_ee_pose, joint_state))
        {
          point.positions = joint_state.position;
        }
        else
        {
          return;
        }
      }
      else
      {
        point.positions = markers_[selected_markers_.back()]->jointState().position;
      }
      goal.trajectory.points.push_back(point);
      action_client_map_["manipulator"]->sendGoal(goal, boost::bind(&InspectorArm::controllerDoneCallback, this, _1, _2));
    }
  }
}

void InspectorArm::onMarkerArrayPublisherTimer()
{
  QMutexLocker lock(&mutex_marker_array_);
  if (marker_array_publisher_.getNumSubscribers() != 0)
  {

    if (ui.checkBoxShowJointMarkers->isChecked() && (markers_.find(selected_markers_.back()) != markers_.end()))
    {
      sensor_msgs::JointState joint_state = markers_[selected_markers_.back()]->jointState();
      std::map<std::string, double> joint_state_map;
      for (unsigned int i = 0; i < joint_state.name.size(); ++i)
      {
        joint_state_map[joint_state.name[i]] = joint_state.position[i];
      }

      //target_robot_state_->setKinematicState()
      target_robot_state_->setKinematicState(joint_state_map);
      //collision_models_interface_->getAllCollisionPointMarkers(*target_robot_state_, marker_array_, bad_color_, ros::Duration(0.1));
      collision_models_interface_->getGroupAndUpdatedJointMarkersGivenState(*target_robot_state_,
                                                                                  marker_array_,
                                                                                  "manipulator", start_color_, end_color_,
                                                                                  ros::Duration(0.1));


    }



    marker_array_publisher_.publish(marker_array_);
    marker_array_.markers.clear();
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

