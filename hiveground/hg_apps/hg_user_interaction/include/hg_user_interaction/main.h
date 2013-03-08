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

#ifndef USER_INTERACTION_H_
#define USER_INTERACTION_H_

#include <ros/ros.h>
#include <QtGui>
#include "ui_main.h"

#include <hg_object_tracking/Hands.h>
#include <kinect_msgs/Skeletons.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <hg_user_interaction/gesture_detector.h>
#include <ve_view.h>
#include <ve_scene.h>


namespace hg_user_interaction
{

class UserInteraction : public QMainWindow
{
  Q_OBJECT
public:
  UserInteraction(QWidget *parent = 0, Qt::WFlags flags = 0);
  ~UserInteraction();

  bool initialize();


protected:
  void skeletonsHandsCallback(const kinect_msgs::SkeletonsConstPtr& skeletons, const hg_object_tracking::HandsConstPtr& hands);
  void skeletonsCallback(const kinect_msgs::SkeletonsConstPtr& skelentons);
  void handsCallback(const hg_object_tracking::HandsConstPtr& hands);
  void publishTransforms(const kinect_msgs::SkeletonsConstPtr& skelentons);
  void publishTransform(int user,
                           const geometry_msgs::Transform& position,
                           std::string const& frame_id,
                           std::string const& child_frame_id);
  void getSkeletionMarker(const kinect_msgs::Skeleton& skeleton,
                             const std::string& frame_id,
                             const std_msgs::ColorRGBA& color_joint,
                             const std_msgs::ColorRGBA& color_link,
                             visualization_msgs::MarkerArray& marker_array);
private:
  void updateExpandState();
  void addProperty(QtProperty *property, const QString &id);


private Q_SLOTS:
  void gestureDetectorItemClicked(QObject* item);
  void valueChanged(QtProperty *property, double value);
  void valueChanged(QtProperty *property, int value);
  void valueChanged(QtProperty *property, bool value);
  void valueChanged(QtProperty *property, const QString &value);



public:
  Ui::MainWindow ui;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  bool quit_threads_;

  QtTreePropertyBrowser *property_editor_;
  QtDoublePropertyManager *double_manager_;
  QtIntPropertyManager * integer_manager_;
  QtBoolPropertyManager* bool_manager_;
  QtStringPropertyManager *string_manager_;
  QtGroupPropertyManager *group_manager_;

  GestureDetectorItem* current_item_;
  QMap<QtProperty *, QString> property_to_id_;
  QMap<QString, QtProperty *> id_to_property_;
  QMap<QString, bool> id_to_expanded_;

  ve::View* view_;
  ve::Scene* scene_;





  std::string skeletons_topic_;
  std::string hands_topic_;
  boost::shared_ptr<message_filters::Subscriber<kinect_msgs::Skeletons> > skeletons_sub_;
  boost::shared_ptr<message_filters::Subscriber<hg_object_tracking::Hands> > hands_sub_;
  typedef message_filters::sync_policies::ApproximateTime<kinect_msgs::Skeletons, hg_object_tracking::Hands> SkeltonsHandsSyncPolicy;
  boost::shared_ptr<message_filters::Synchronizer<SkeltonsHandsSyncPolicy> > skeltons_hands_sync_;


  tf::TransformBroadcaster br_;
  tf::TransformListener tf_listener_;
  ros::Subscriber skeletons_sub_raw_;
  ros::Publisher skeletons_markers_publisher_;
  int skeleton_marker_id_;

  std_msgs::ColorRGBA color_joint_;
  std_msgs::ColorRGBA color_link_;

  ros::Subscriber hands_sub_raw_;
  ros::Publisher hands_markers_publisher_;



};




}




#endif /* KINECT_INTERACTION_H_ */
