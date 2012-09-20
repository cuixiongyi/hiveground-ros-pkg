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

#ifndef PRW_H
#define PRW_H


#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/model_utils.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <arm_navigation_msgs/GetPlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <arm_navigation_msgs/GetStateValidity.h>
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <arm_navigation_msgs/convert_messages.h>

#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetPositionIK.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>

#include <std_msgs/String.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>

#include <prw_message/Object.h>
#include <prw_message/ObjectArray.h>

#include <qevent.h>
#include <qdialog.h>
#include <qmutex.h>

#include "ui_prw.h"

static inline geometry_msgs::Pose toGeometryPose(tf::Transform transform)
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

static inline tf::Transform toBulletTransform(geometry_msgs::Pose pose)
{
  tf::Quaternion quat = tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf::Vector3 vec = tf::Vector3(pose.position.x, pose.position.y, pose.position.z);
  return tf::Transform(quat, vec);
}

/// May be for IK control, joint control, or collision objects.
enum InteractiveMarkerType
{
  EndEffectorControlMarker,
  JointControlMarker,
  CollisionObjectMarker
};

enum CollisionObjectType
{
  Pole, Box, Sphere
};

/// Contains data for selectable markers. Stored in a large map.
struct SelectableMarker
{
  /// IK control, joint control, or collision object.
  InteractiveMarkerType type_;

  /// Name of the menu marker.
  std::string name_;

  /// Name of the 6DOF marker
  std::string controlName_;

  /// Text above the control.
  std::string controlDescription_;

  CollisionObjectType collision_object_type_;
  double a_, b_, c_;
  std_msgs::ColorRGBA color_;
  geometry_msgs::Pose pose_;
};

enum IKControlType
{
  StartPosition, EndPosition
};

struct TrajectoryData
{

  TrajectoryData()
  {
    state_ = NULL;
    reset();
  }

  void reset()
  {
    if (state_ != NULL)
    {
      delete state_;
      state_ = NULL;
    }
    has_joint_trajectory_ = false;
    play_joint_trajectory_ = false;
    show_joint_trajectory_ = false;
    current_trajectory_point_ = 0;
    trajectory_bad_point_ = 0;
    trajectory_error_code_.val = 0;
  }

  planning_models::KinematicState* state_;
  trajectory_msgs::JointTrajectory joint_trajectory_;
  unsigned int current_trajectory_point_;
  std_msgs::ColorRGBA color_;
  bool has_joint_trajectory_;
  bool play_joint_trajectory_;
  bool show_joint_trajectory_;
  arm_navigation_msgs::ArmNavigationErrorCodes trajectory_error_code_;
  unsigned int trajectory_bad_point_;
};

typedef std::map<std::string, TrajectoryData> TrajectoryDataMap;

class PlanningGroupData
{
public:
  std::string name_;
  std::string ik_link_name_;
  ros::ServiceClient ik_collision_aware_client_;
  ros::ServiceClient ik_non_collision_aware_client_;
  boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> > arm_controller_;
  planning_models::KinematicState* start_state_;
  planning_models::KinematicState* end_state_;
  std_msgs::ColorRGBA start_color_;
  std_msgs::ColorRGBA end_color_;
  TrajectoryDataMap trajectory_data_map_;
  bool good_ik_solution_;
  tf::Transform last_good_state_;
  std::vector<std::string> joint_names_;


  PlanningGroupData()
  {
    start_state_ = NULL;
    end_state_ = NULL;

    start_color_.a = 0.3;
    start_color_.r = 1.0;
    start_color_.g = 0.5;
    start_color_.b = 1.0;

    end_color_.a = 0.3;
    end_color_.r = 0.5;
    end_color_.g = 0.9;
    end_color_.b = 0.5;

    trajectory_data_map_["planner"].color_.a = .6;
    trajectory_data_map_["planner"].color_.r = 1.0;
    trajectory_data_map_["planner"].color_.g = 1.0;
    trajectory_data_map_["planner"].color_.b = 0.5;

    trajectory_data_map_["filter"].color_.a = .6;
    trajectory_data_map_["filter"].color_.r = 0.5;
    trajectory_data_map_["filter"].color_.g = 1.0;
    trajectory_data_map_["filter"].color_.b = 1.0;
  }

  ~PlanningGroupData()
  {
    reset();
  }

  void setState(IKControlType type, planning_models::KinematicState* state)
  {
    switch (type)
    {
      case StartPosition:
        if (start_state_ != NULL)
        {
          delete start_state_;
          start_state_ = NULL;
        }
        start_state_ = state;
        break;
      case EndPosition:
        if (end_state_ != NULL)
        {
          delete end_state_;
          end_state_ = NULL;
        }
        end_state_ = state;
        break;
    }
  }

  void reset()
  {
    for (TrajectoryDataMap::iterator it = trajectory_data_map_.begin(); it != trajectory_data_map_.end(); it++)
    {
      it->second.reset();
    }

    if (start_state_ != NULL)
    {
      delete start_state_;
      start_state_ = NULL;
    }
    if (end_state_ != NULL)
    {
      delete end_state_;
      end_state_ = NULL;
    }
  }


};

typedef std::map<std::string, PlanningGroupData> PlanningGroupDataMap;

typedef std::map<interactive_markers::MenuHandler::EntryHandle, std::string> MenuEntryMap;
typedef std::map<std::string, MenuEntryMap> MenuMap;
typedef std::map<std::string, interactive_markers::MenuHandler> MenuHandlerMap;
typedef std::map<std::string, arm_navigation_msgs::CollisionObject> CollisionObjectMap;
typedef QMap<int, QPair<QString, geometry_msgs::Pose> > EndEffectorStateMap;

class PRW : public QMainWindow
{
  Q_OBJECT
public:

  PRW(QWidget *parent = 0, Qt::WFlags flags = 0);
  ~PRW();
  bool initialize();

private Q_SLOTS:
  //QtJointStateConstPtr
  void on_bt_go_clicked();
  void on_bt_reset_clicked();
  void on_cb_enable_teleop_clicked();

  void on_new_scene_clicked();
  void on_new_mpr_clicked();
  void on_plan_clicked();

  //prw
  void endEffectorSlideUpdate();
  void endEffectorValueUpdate();
  void endEffectorMoved();

  void createNewCollisionObject(const QString& type);

  void savedStateSelection();
  void addState(const geometry_msgs::Pose& pose);
  void on_bt_move_to_saved_state_clicked();
  void on_bt_play_saved_state_clicked();
  void on_bt_stop_play_saved_stated_clicked();
  void on_bt_delete_saved_state_clicked();

Q_SIGNALS:
  void signalCreateNewCollisionObject(const QString& type);
  void signalUpdateEndEffectorPosition();
  void signalAddState(const geometry_msgs::Pose& pose);

protected:
  void closeEvent(QCloseEvent *event);



  //prw
  //callback
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& message);
  void gestureCallback(const std_msgs::StringConstPtr& message);
  void objectTrackingCallback(const prw_message::ObjectArrayConstPtr& message);

  void processIKControllerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void processMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void processMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void lockScene() { scene_mutex_.lock(); }
  void unlockScene() { scene_mutex_.unlock(); }
  void sendPlanningScene();
  void selectPlanningGroup(int group);
  PlanningGroupData* getPlanningGroup(unsigned int i);

  //collision object
  int getNextCollisionObjectId() { return last_collision_objects_id_++; }
  void createCollisionObject(geometry_msgs::Pose pose,
                             CollisionObjectType type,
                             std_msgs::ColorRGBA color,
                             double a = 0.1,
                             double b = 0.0,
                             double c = 0.0,
                             bool selectable = true);

  //interactive marker
  void makeIKControllerMarker(tf::Transform transform, const std::string& name, const std::string& description,
                              bool selectable, float scale = 1.0, bool publish = true);
  void makeSelectableMarker(InteractiveMarkerType type, tf::Transform transform, const std::string& name,
                              const std::string& description, float scale = 1.0f, bool publish = true);
  void makeSelectableCollisionObjectMarker(tf::Transform transform, CollisionObjectType type, const std::string& name,
                                           std_msgs::ColorRGBA color, double a, double b, double c, bool publish=true);
  visualization_msgs::Marker makeMarkerCylinder(visualization_msgs::InteractiveMarker &msg, float alpha = 1.0f);
  visualization_msgs::Marker makeMarkerBox(visualization_msgs::InteractiveMarker &msg, float alpha = 1.0f);
  void makeInteractive6DOFMarker(bool fixed, tf::Transform transform, const std::string& name, const std::string& description, float scale = 1.0f, bool object = false);
  visualization_msgs::InteractiveMarkerControl& makeInteractiveBoxControl(visualization_msgs::InteractiveMarker &msg, float alpha = 1.0f);
  visualization_msgs::InteractiveMarkerControl& makeInteractiveCylinderControl(visualization_msgs::InteractiveMarker &msg, float alpha = 1.0f);
  visualization_msgs::InteractiveMarkerControl& makeInteractiveCollisionObjectControl(visualization_msgs::InteractiveMarker &msg);
  visualization_msgs::Marker createCollisionObjectMarker(CollisionObjectType type, double a, double b, double c, std_msgs::ColorRGBA color);


  void deselectMarker(SelectableMarker& marker, tf::Transform transform);
  void selectMarker(SelectableMarker& marker, tf::Transform transform);
  void moveThroughTrajectory(PlanningGroupData& gc, const std::string& source_name, int step);
  void removeCollisionObjectByName(std::string id);

  void makeMenu();
    interactive_markers::MenuHandler::EntryHandle registerMenuEntry(interactive_markers::MenuHandler& handler, MenuEntryMap& map, std::string name);



  //end effector
  void moveEndEffectorMarkers(double vx, double vy, double vz, double vr, double vp, double vw, bool coll_aware = true);
  void setNewEndEffectorPosition(PlanningGroupData& gc, tf::Transform& cur, bool coll_aware);
  bool solveIKForEndEffectorPose(PlanningGroupData& gc, bool coll_aware = true, bool constrain_pitch_and_roll = false,
                                 double change_redundancy = 0.0);



  bool moveEndEffector(PlanningGroupData& gc,
                       const geometry_msgs::Pose& pose,
                       bool filter=true,
                       int retry = 20,
                       bool block = false,
                       bool update_marker = false);
  bool planToEndEffectorState(PlanningGroupData& gc, bool show, bool play);
  bool filterPlannerTrajectory(PlanningGroupData& gc, bool show, bool play);
  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const control_msgs::FollowJointTrajectoryResultConstPtr& result);
  void moveThroughSavedState();
  void followObject(PlanningGroupData& gc, const geometry_msgs::Pose& pose, double offset);


  void resetToLastGoodState(PlanningGroupData& gc);
  void refreshEnvironment();




  inline bool isCollisionObjectName(const std::string& name)
  {
    return collision_objects_.find(name) != collision_objects_.end();
  }

  inline bool isGroupName(const std::string& name)
  {
    return group_data_map_.find(name) != group_data_map_.end();
  }

  inline bool selectableMarkerExists(const std::string& name)
  {
    return selectable_markers_.find(name) != selectable_markers_.end();
  }



public:
  void sendMarkers();


public:
  bool quit_threads_;

private:
  Ui::PRW ui;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  boost::recursive_mutex scene_mutex_;
  boost::recursive_mutex ui_mutex_;
  boost::recursive_mutex tracked_object_mutex_;



  planning_environment::CollisionModels* cm_;
  planning_models::KinematicState* robot_state_;
  bool collision_aware_;
  bool constrain_rp_;
  bool is_joint_control_active_;
  bool is_ik_control_active_;

  tf::TransformBroadcaster transform_broadcaster_;
  tf::TransformListener transform_listener_;


  //subscriber
  ros::Subscriber joint_state_subscriber_;
  std::map<std::string, double> robot_state_joint_values_;

  ros::Subscriber gesture_subscriber_;
  ros::Subscriber object_tracking_subscriber_;

  //publisher
  ros::Publisher marker_publisher_;
  ros::Publisher marker_array_publisher_;

  //interactive marker
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
  interactive_markers::MenuHandler::FeedbackCallback process_ik_controller_feedback_ptr_;
  interactive_markers::MenuHandler::FeedbackCallback process_menu_feedback_ptr_;
  interactive_markers::MenuHandler::FeedbackCallback process_marker_feedback_ptr_;

  //service client
  ros::ServiceClient get_planning_scene_client_;
  ros::ServiceClient planner_client_;
  ros::ServiceClient trajectory_filter_client_;

  //arm control
  PlanningGroupDataMap group_data_map_;
  std::string current_group_name_;
  //std::map<std::string, geometry_msgs::Pose> last_end_effector_poses_;


  arm_navigation_msgs::MotionPlanRequest last_motion_plan_request_;
  bool arm_is_moving_;

  //Menu, marker
  std::map<std::string, SelectableMarker> selectable_markers_;
  MenuHandlerMap menu_handler_map_;
  MenuMap menu_entry_maps_;

  //end effector menu handle
  interactive_markers::MenuHandler::EntryHandle menu_ee_save_state_;
  interactive_markers::MenuHandler::EntryHandle menu_ee_play_save_state_;
  interactive_markers::MenuHandler::EntryHandle menu_ee_last_good_state_;

  //std::vector<geometry_msgs::Pose> saved_ee_state_;

  EndEffectorStateMap saved_end_effector_state_;
  int last_saved_end_effector_state_id_;
  int selected_saved_state_id_;
  QTreeWidgetItem* selected_saved_state_item_;
  bool play_saved_state_;



  //collision object
  /// Map of collision objects names to messages sent to ROS.
  CollisionObjectMap collision_objects_;
  int last_collision_objects_id_;


  //tracked object
  std::vector<tf::Transform> tracked_objects_;
  tf::Transform following_point_;

};

#endif
