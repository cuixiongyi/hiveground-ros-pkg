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
#include "ui_inspector_arm.h"

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <hg_inspector_arm/inspection_point.h>


typedef std::map<interactive_markers::MenuHandler::EntryHandle, std::string> MenuEntryHandleMap;
typedef std::map<std::string, MenuEntryHandleMap> MenuEntryMap;
typedef std::map<std::string, interactive_markers::MenuHandler> MenuHandlerMap;

class InspectorArm : public QMainWindow
{
  Q_OBJECT
public:

  InspectorArm(QWidget *parent = 0, Qt::WFlags flags = 0);
  ~InspectorArm();

  bool initialize();


  void addMarker(const std::string& name, geometry_msgs::Pose pose = geometry_msgs::Pose() ,double arrow_length = 0.1);
  void addMarkerAtEndEffector();

protected:
  bool initializeInteractiveMarkerServer();
  bool initializePropertyEditor();


  //Interactive marker
  void processMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  bool checkIK(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg);
  visualization_msgs::Marker makeArrow(visualization_msgs::InteractiveMarker &msg, double arrow_length);
  visualization_msgs::InteractiveMarkerControl& makeArrowControl(visualization_msgs::InteractiveMarker &msg,
                                                                 double arrow_length);
  void makeMenu();
  interactive_markers::MenuHandler::EntryHandle registerMenuEntry(interactive_markers::MenuHandler& handler,
                                                                     MenuEntryHandleMap& map, std::string name);
  //Interactive marker

  //Inspection point property
  void updateExpandState();
  void addProperty(QtProperty *property, const QString &id);


Q_SIGNALS:
  void inspectionPointClickedSignal(InspectionPointItem *item);
  void inspectionPointMovedSignal(InspectionPointItem *item);

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
  void on_pushButtonAddInspectionPoint_clicked();

protected:
  void closeEvent(QCloseEvent *evencurrentItemt);

public:
  bool quit_thread_;

private:
  Ui::InspectorArm ui;
  ros::NodeHandle nh_, nh_private_;

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
  ros::ServiceClient ik_query_client_;
  ros::ServiceClient ik_client_;
  kinematics_msgs::GetKinematicSolverInfo::Response response_;
  int name_count_;
  std::map<std::string, InspectionPointItem*> markers_;
  interactive_markers::MenuHandler::FeedbackCallback marker_callback_ptr_;
  MenuEntryMap menu_entry_maps_;
  MenuHandlerMap menu_handler_map_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_top_add_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_top_clear_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_add_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_add_here_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_reset_position_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_reset_orientation_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_remove_;
};



#endif /* INSPECTOR_ARM_H_ */
