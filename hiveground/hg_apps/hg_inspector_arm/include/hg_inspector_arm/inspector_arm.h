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

#ifndef INSPECTOR_ARM_H_
#define INSPECTOR_ARM_H_


#include <qmainwindow.h>
#include <qmutex.h>
#include <qtimer.h>
#include "ui_inspector_arm.h"

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <planning_models/kinematic_state.h>
#include <visualization_msgs/MarkerArray.h>


#include <hg_inspector_arm/inspection_point.h>
#include <hg_inspector_arm/planning_base.h>

#include <hg_object_tracking/Hands.h>
#include <hg_user_interaction/Gestures.h>


typedef std::map<interactive_markers::MenuHandler::EntryHandle, std::string> MenuEntryHandleMap;
typedef std::map<std::string, MenuEntryHandleMap> MenuEntryMap;
typedef std::map<std::string, interactive_markers::MenuHandler> MenuHandlerMap;

#define FILE_MAGIC_MARKER 0x6602AAAA
#define FILE_VERSION_MARKER 101

class InspectorArm : public QMainWindow, PlanningBase
{
  Q_OBJECT

  friend class InspectionPointItem;

public:

  InspectorArm(QWidget *parent = 0, Qt::WFlags flags = 0);
  ~InspectorArm();

  bool run();

protected:
  bool initialize(const std::string& param_server_prefix);
  bool initializeInteractiveMarkerServer();
  bool initializePropertyEditor();
  bool initializeServiceClient();


  //Interactive marker
  void processMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  bool checkIK(const tf::Transform& pose, sensor_msgs::JointState& joint_state);
  bool checkIK(const geometry_msgs::Pose& pose, sensor_msgs::JointState& joint_state);
  bool checkIKConstraintAware(const tf::Transform& pose, sensor_msgs::JointState& joint_state);
  bool checkIKConstraintAware(const geometry_msgs::Pose& pose, sensor_msgs::JointState& joint_state);

  void addMarker(const std::string& name,
                   geometry_msgs::Pose pose = geometry_msgs::Pose(),
                   bool selectable = true,
                   double arrow_length = 0.05,
                   double scale = 2.0);
  std::string getMarkerName(const std::string& type);
  void addMarkerAtEndEffector();
  void addMarkerAtTool();
  void addInspectionPoint(const std::string& name, const geometry_msgs::Pose& pose, const sensor_msgs::JointState& joint_state);
  void addInspectionPointFrom(const std::string& name, const InspectionPointItem* item);
  void clearMarker();
  visualization_msgs::Marker makeBox(double size = 0.1,
                                      double r = 1.0, double g = 0.0, double b = 0.0, double a = 1.0);
  visualization_msgs::Marker makeArrow(double size = 0.1,
                                        double r = 1.0, double g = 0.0, double b = 0.0, double a = 1.0);
  visualization_msgs::Marker makeSphere(double size = 0.1,
                                         double r = 1.0, double g = 0.0, double b = 0.0, double a = 1.0);
  visualization_msgs::InteractiveMarkerControl& makeFreeMoveControl(visualization_msgs::InteractiveMarker &msg,
                                                                       std::vector<visualization_msgs::Marker>& markers);
  visualization_msgs::InteractiveMarkerControl& make6DOFControl(visualization_msgs::InteractiveMarker &msg,
                                                                  std::vector<visualization_msgs::Marker>& markers);
  visualization_msgs::InteractiveMarkerControl& makeSelectableControl(visualization_msgs::InteractiveMarker &msg,
                                                                         std::vector<visualization_msgs::Marker>& markers);
  void selectMarker(const std::string& name);
  void deselectMarker(const std::string& name);
  void selectOnlyOneMarker(const std::string& name);
  bool setMarkerOrientation(const std::string& name, double roll, double pitch, double yaw);

  void makeMenu();
  interactive_markers::MenuHandler::EntryHandle registerMenuEntry(interactive_markers::MenuHandler& handler,
                                                                     MenuEntryHandleMap& map, std::string name);

  void saveMarker();
  void loadMarker();

  //Inspection point property
  void updateExpandState();
  void addProperty(QtProperty *property, const QString &id);


  //Callback
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& message);
  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                                  const control_msgs::FollowJointTrajectoryResultConstPtr& result);
  void handsCallBack(const hg_object_tracking::HandsConstPtr message);
  void handGestureCallBack(const hg_user_interaction::GesturesConstPtr message);
  void bodyGestureCallBack(const hg_user_interaction::GesturesConstPtr message);
  void spaceNavigatorCallBack(const geometry_msgs::TwistConstPtr message);
  void updateMarkerCallbBack(const std::string& name, const tf::Transform& pose);
  void updateMarkerCallbBack(const std::string& name, const geometry_msgs::Pose& pose);


  //utility functions
  void lookAt(const tf::Vector3& at, const tf::Transform& from, double distance, tf::Transform& result);

Q_SIGNALS:
  void inspectionPointClickedSignal(InspectionPointItem *item);
  void inspectionPointMovedSignal(InspectionPointItem *item);
  void followPointSignal();

private Q_SLOTS:
  //Inspection point property
  void inspectionPointClicked(InspectionPointItem* item);
  void inspectionPointMoved(InspectionPointItem* item);
  void valueChanged(QtProperty *property, double value);
  void valueChanged(QtProperty *property, const QString &value);
  void valueChanged(QtProperty *property, const QColor &value);
  void valueChanged(QtProperty *property, const QFont &value);
  void valueChanged(QtProperty *property, const QPoint &value);
  void valueChanged(QtProperty *property, const QSize &value);

  //UI
  void on_pushButtonPlan_clicked();


  //Menu action
  //File Menu

  //Path Menu
  void on_actionAddMarker_triggered();
  void on_actionAddMarkerToTool_triggered();
  void on_actionClearMarker_triggered();
  void on_actionLoadMarker_triggered();
  void on_actionSaveMarker_triggered();
  void on_actionSaveMarkerAs_triggered();

  //Follow point
  void followPointSlot();

  //Marker
  void onMarkerArrayPublisherTimer();

  //Marker list
  void on_listWidgetMarker_itemClicked( QListWidgetItem * item );




protected:
  void closeEvent(QCloseEvent *evencurrentItemt);

public:
  bool quit_thread_;

private:
  Ui::InspectorArm ui;

  //property editor
  QtTreePropertyBrowser *property_editor_;
  QtDoublePropertyManager *double_manager_;
  QtStringPropertyManager *string_manager_;
  QtColorPropertyManager *color_manager_;
  QtFontPropertyManager *font_manager_;
  QtPointPropertyManager *point_manager_;
  QtSizePropertyManager *size_manager_;
  QtGroupPropertyManager *group_manager_;

  InspectionPointItem* current_item_;
  QMap<QtProperty *, QString> property_to_id_;
  QMap<QString, QtProperty *> id_to_property_;
  QMap<QString, bool> id_to_expanded_;



  //Interactive marker
  interactive_markers::InteractiveMarkerServer marker_server_;
  tf::TransformListener listener_;
  std::string world_frame_;
  std::string base_link_;
  std::string tool_frame_;
  tf::Transform last_feedback_pose_;
  kinematics_msgs::GetKinematicSolverInfo::Response ik_solver_info_;
  int name_count_;
  std::map<std::string, InspectionPointItem*> markers_;
  std::list<std::string> selected_markers_;
  interactive_markers::MenuHandler::FeedbackCallback marker_callback_ptr_;
  MenuEntryMap menu_entry_maps_;
  MenuHandlerMap menu_handler_map_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_top_add_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_top_clear_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_add_here_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_point_x_plus_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_point_x_minus_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_point_y_plus_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_point_y_minus_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_point_z_plus_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_point_z_minus_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_reset_position_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_reset_orientation_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_remove_;
  QString markers_save_file_name_;
  bool markers_touched_;


  //trajectory
  planning_models::KinematicState* robot_state_;
  planning_models::KinematicState* target_robot_state_;
  std_msgs::ColorRGBA start_color_;
  std_msgs::ColorRGBA end_color_;
  std_msgs::ColorRGBA stat_color_;
  std_msgs::ColorRGBA attached_color_;
  std_msgs::ColorRGBA bad_color_;
  std_msgs::ColorRGBA collision_color_;

  ros::Subscriber joint_state_subscriber_;
  sensor_msgs::JointState latest_joint_state_;
  QMutex mutex_joint_state_;

  typedef boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> > FollowJointTrajectoryClientPtr;
  std::map<std::string, FollowJointTrajectoryClientPtr> action_client_map_;
  ros::ServiceClient hg_cartesian_trajectory_client_;
  bool arm_is_active_;


  //gesture
  ros::Subscriber hands_subscriber_;
  ros::Subscriber hand_gestures_subscriber_;
  ros::Subscriber body_gestures_subscriber_;

  //3d mouse
  ros::Subscriber space_navigator_subscriber_;
  ros::Time space_navigator_last_update_;


  //Marker
  QTimer* marker_array_publisher_timer_;
  QMutex mutex_marker_array_;
  ros::Publisher marker_array_publisher_;
  visualization_msgs::MarkerArray marker_array_;

};



#endif /* INSPECTOR_ARM_H_ */
