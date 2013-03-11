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


#include <hg_user_interaction/main.h>
#include <boost/thread.hpp>

using namespace hg_user_interaction;
using namespace message_filters;
using namespace kinect_msgs;
using namespace visualization_msgs;


UserInteraction::UserInteraction(QWidget *parent, Qt::WFlags flags)
  : QMainWindow(parent, flags),
    nh_(), nh_private_("~"),
    quit_threads_(false)
{
  ui.setupUi(this);
}

UserInteraction::~UserInteraction()
{

}

bool UserInteraction::initialize()
{
  nh_private_.getParam("skeletons_topic", skeletons_topic_);
  nh_private_.getParam("hands_topic", hands_topic_);

  tf_listener_.waitForTransform("base_link", "kinect_server", ros::Time(0), ros::Duration(5.0));
  color_joint_.r = 1.0; color_joint_.g = 0.0; color_joint_.b = 0.0; color_joint_.a = 0.5;
  color_link_.r = 0.0; color_link_.g = 1.0; color_link_.b = 0.0; color_link_.a = 0.5;


  skeletons_sub_raw_ = nh_private_.subscribe(skeletons_topic_, 1, &UserInteraction::skeletonsCallback, this);
  skeletons_markers_publisher_ = nh_private_.advertise<MarkerArray>("skeletons_makers", 128);
  body_gestures_publisher_ = nh_private_.advertise<Gestures>("body_gestures", 128);

  hands_sub_raw_ = nh_private_.subscribe(hands_topic_, 1, &UserInteraction::handsCallback, this);
  hand_markers_publisher_ = nh_private_.advertise<MarkerArray>("hands_makers", 128);
  hand_gestures_publisher_ = nh_private_.advertise<Gestures>("hand_gestures", 128);

  /*
  nh_private_.getParam("skeletons_topic", skeletons_topic_);
  nh_private_.getParam("hands_topic", hands_topic_);
  ROS_INFO_STREAM(skeletons_topic_ << ":" << hands_topic_);

  skeletons_sub_ = boost::shared_ptr<message_filters::Subscriber<kinect_msgs::Skeletons> >(new message_filters::Subscriber<kinect_msgs::Skeletons>(nh_, skeletons_topic_, 10));
  hands_sub_ = boost::shared_ptr<message_filters::Subscriber<hg_object_tracking::Hands> >(new message_filters::Subscriber<hg_object_tracking::Hands>(nh_, hands_topic_, 10));

  skeltons_hands_sync_ = boost::shared_ptr<message_filters::Synchronizer<SkeltonsHandsSyncPolicy> >(new message_filters::Synchronizer<SkeltonsHandsSyncPolicy>(SkeltonsHandsSyncPolicy(10), *skeletons_sub_, *hands_sub_));
  skeltons_hands_sync_->registerCallback(boost::bind(&UserInteraction::skeletonsHandsCallback, this, _1, _2));
  */

  double_manager_ = new QtDoublePropertyManager(this);
  string_manager_ = new QtStringPropertyManager(this);
  integer_manager_ = new QtIntPropertyManager(this);
  bool_manager_ = new QtBoolPropertyManager(this);
  group_manager_ = new QtGroupPropertyManager(this);

  connect(double_manager_, SIGNAL(valueChanged(QtProperty *, double)),
           this, SLOT(valueChanged(QtProperty *, double)));
  connect(integer_manager_, SIGNAL(valueChanged(QtProperty *, int)),
           this, SLOT(valueChanged(QtProperty *, int)));
  connect(bool_manager_, SIGNAL(valueChanged(QtProperty *, bool)),
           this, SLOT(valueChanged(QtProperty *, bool)));
  connect(string_manager_, SIGNAL(valueChanged(QtProperty *, const QString &)),
           this, SLOT(valueChanged(QtProperty *, const QString &)));

  QtDoubleSpinBoxFactory *doubleSpinBoxFactory = new QtDoubleSpinBoxFactory(this);
  QtCheckBoxFactory *checkBoxFactory = new QtCheckBoxFactory(this);
  QtSpinBoxFactory *spinBoxFactory = new QtSpinBoxFactory(this);
  QtLineEditFactory *lineEditFactory = new QtLineEditFactory(this);
  //QtEnumEditorFactory *comboBoxFactory = new QtEnumEditorFactory(this);


  property_editor_ = new QtTreePropertyBrowser(ui.dockWidgetPropertyEditor);
  property_editor_->setFactoryForManager(double_manager_, doubleSpinBoxFactory);
  property_editor_->setFactoryForManager(integer_manager_, spinBoxFactory);
  property_editor_->setFactoryForManager(bool_manager_, checkBoxFactory);
  property_editor_->setFactoryForManager(string_manager_, lineEditFactory);
  ui.dockWidgetPropertyEditor->setWidget(property_editor_);
  ui.dockWidgetPropertyEditor->setMinimumWidth(150);

  current_item_ = 0;

  view_ = new ve::View(this);
  ui.gridLayoutView->addWidget(view_);

  scene_ = new ve::Scene(view_);
  scene_->setSceneRect(0, 0, 800, 600);
  view_->setScene(scene_);
  scene_->set_mode(ve::Scene::MODE_CURSOR);
  view_->ensureVisible(scene_->itemsBoundingRect());

  connect(scene_, SIGNAL(signal_object_selected(QObject*)),
           this, SLOT(gestureDetectorItemClicked(QObject*)));

  connect(scene_, SIGNAL(signal_node_deleted(QObject*)),
          this, SLOT(gestureDetectorItemDeleted(QObject*)));

  GestureDetectorItem* item = 0;
  item = new GestureDetectorHandPushPull(nh_private_, QRect(50, 50, 50, 50));
  item->setObjectName("HandPushPull");
  if(item->initialize())
  {
    scene_->addItem(item);
    gesture_detector_items_[item->objectName()] = item;
  }

  item = new GestureBodyMovement(nh_private_, QRect(100, 50, 50, 50));
  item->setObjectName("BodyMovement");
  if(item->initialize())
  {
    scene_->addItem(item);
    gesture_detector_items_[item->objectName()] = item;
  }





  return true;
}


void UserInteraction::skeletonsHandsCallback(const kinect_msgs::SkeletonsConstPtr& skeletons, const hg_object_tracking::HandsConstPtr& hands)
{
  ROS_INFO_THROTTLE(1.0, __FUNCTION__);
  //ROS_INFO_STREAM(skeletons->header.stamp);
  //ROS_INFO_STREAM(hands->header.stamp);

  tf::StampedTransform tf;
  tf_listener_.lookupTransform("base_link", "kinect_server", ros::Time(0), tf);


  visualization_msgs::MarkerArray marker_array;
  skeleton_marker_id_ = 0;


  for (int i = 0; i < Skeletons::SKELETON_COUNT; i++)
  {
    if (skeletons->skeletons[i].skeleton_tracking_state == Skeleton::SKELETON_TRACKED)
    {
      getSkeletionMarker(skeletons->skeletons[i], "kinect_server", color_joint_, color_link_, marker_array);

      ROS_DEBUG_STREAM(
          "[" << i << "]: " << skeletons->skeletons[i].tracking_id << " : "<< skeletons->skeletons[i].user_index);
      tf::Transform skeleton[Skeleton::SKELETON_POSITION_COUNT];
      for (int j = 0; j < Skeleton::SKELETON_POSITION_COUNT; j++)
      {
        tf::transformMsgToTF(skeletons->skeletons[i].skeleton_positions[j], skeleton[j]);
        skeleton[j] = tf * skeleton[j];
      }

      tf::Vector3 wrist_left_to_right = skeleton[Skeleton::SKELETON_POSITION_WRIST_LEFT].getOrigin()
          - skeleton[Skeleton::SKELETON_POSITION_WRIST_RIGHT].getOrigin();
      tf::Vector3 hand_left_to_right = skeleton[Skeleton::SKELETON_POSITION_HAND_LEFT].getOrigin()
          - skeleton[Skeleton::SKELETON_POSITION_HAND_RIGHT].getOrigin();
      //ROS_INFO("skeleton_wrist_distance: %f", wrist_left_to_right.length());
      //ROS_INFO("skeleton_hand_distance: %f", hand_left_to_right.length());
    }
  }

  if(hands->hands.size() == 2)
  {
    tf::Transform left_hand;
    tf::Transform right_hand;
    if(hands->hands[0].hand_centroid.translation.x <= hands->hands[1].hand_centroid.translation.x)
    {
      tf::transformMsgToTF(hands->hands[0].hand_centroid, left_hand);
      tf::transformMsgToTF(hands->hands[1].hand_centroid, right_hand);
    }
    else
    {
      tf::transformMsgToTF(hands->hands[1].hand_centroid, left_hand);
      tf::transformMsgToTF(hands->hands[0].hand_centroid, right_hand);
    }

    tf::Vector3 hand_left_to_right = left_hand.getOrigin() - right_hand.getOrigin();
    ROS_INFO("hand_distance: %f", hand_left_to_right.length());


  }
}

void UserInteraction::skeletonsCallback(const kinect_msgs::SkeletonsConstPtr& skeletons)
{
  MarkerArray markers;
  skeleton_marker_id_ = 0;

  tf::StampedTransform tf;
  tf_listener_.lookupTransform("base_link", "kinect_server", ros::Time(0), tf);

  publishTransforms(skeletons);

  //copy
  kinect_msgs::SkeletonsPtr transformed_skeletons = kinect_msgs::SkeletonsPtr(new kinect_msgs::Skeletons);
  *transformed_skeletons = *skeletons;

  //transform coordinate
  transformed_skeletons->header.frame_id = "base_link";
  tf::Transform transformed_skeleton;
  for (int i = 0; i < Skeletons::SKELETON_COUNT; i++)
  {
    if (transformed_skeletons->skeletons[i].skeleton_tracking_state == Skeleton::SKELETON_TRACKED)
    {
      //getSkeletionMarker(skeletons->skeletons[i], "kinect_server", color_joint_, color_link_, markers);

      for (int j = 0; j < Skeleton::SKELETON_POSITION_COUNT; j++)
      {
        tf::transformMsgToTF(transformed_skeletons->skeletons[i].skeleton_positions[j], transformed_skeleton);
        transformed_skeleton = tf * transformed_skeleton;
        tf::transformTFToMsg(transformed_skeleton, transformed_skeletons->skeletons[i].skeleton_positions[j]);
      }
    }
  }

  //ROS_INFO_THROTTLE(1.0, __FUNCTION__);
  Gestures gestures;
  gestures.header = transformed_skeletons->header;
  GestureDetectorItem* item;

  {
    QMutexLocker locker(&mutex_callback_);
    Q_FOREACH(item, gesture_detector_items_)
    {
      Gesture gesture;
      item->addSkeletonsMessage(transformed_skeletons);
      int detected_gesture = item->lookForGesture(gesture);
      item->drawHistory(markers, transformed_skeletons->header.frame_id);
      item->drawResult(markers, transformed_skeletons->header.frame_id);
      if (detected_gesture != Gesture::GESTURE_NOT_DETECTED)
      {
        gestures.gestures.push_back(gesture);
      }
    }
  }

  if (body_gestures_publisher_.getNumSubscribers() != 0)
  {
    if (!gestures.gestures.empty())
    {
      body_gestures_publisher_.publish(gestures);
    }
  }

  if (skeletons_markers_publisher_.getNumSubscribers() != 0)
  {
    if (!markers.markers.empty())
    {
      skeletons_markers_publisher_.publish(markers);
    }
  }
}

void UserInteraction::handsCallback(const hg_object_tracking::HandsConstPtr& hands)
{
  //ROS_INFO_THROTTLE(1.0, __FUNCTION__);
  Gestures gestures;
  gestures.header = hands->header;
  MarkerArray markers;
  GestureDetectorItem* item;

  {
    QMutexLocker locker(&mutex_callback_);
    Q_FOREACH(item, gesture_detector_items_)
    {
      Gesture gesture;
      item->addHandsMessage(hands);
      int detected_gesture = item->lookForGesture(gesture);
      item->drawHistory(markers, hands->header.frame_id);
      item->drawResult(markers, hands->header.frame_id);
      if(detected_gesture != Gesture::GESTURE_NOT_DETECTED)
        gestures.gestures.push_back(gesture);
    }
  }

  gestures.header.stamp = ros::Time::now();

  if(hand_gestures_publisher_.getNumSubscribers() != 0)
  {
    if(!gestures.gestures.empty())
    {
      hand_gestures_publisher_.publish(gestures);
    }
  }

  if(hand_markers_publisher_.getNumSubscribers() != 0)
  {
    if(!markers.markers.empty())
    {
      hand_markers_publisher_.publish(markers);
    }
  }






  //ROS_INFO_THROTTLE(1.0, __FUNCTION__);
#if 0
  if (hands->hands.size() == 2)
  {
    tf::Transform left_hand;
    tf::Transform right_hand;
    if (hands->hands[0].hand_centroid.translation.x <= hands->hands[1].hand_centroid.translation.x)
    {
      tf::transformMsgToTF(hands->hands[0].hand_centroid, left_hand);
      tf::transformMsgToTF(hands->hands[1].hand_centroid, right_hand);
    }
    else
    {
      tf::transformMsgToTF(hands->hands[1].hand_centroid, left_hand);
      tf::transformMsgToTF(hands->hands[0].hand_centroid, right_hand);
    }

    tf::Vector3 hand_left_to_right = left_hand.getOrigin() - right_hand.getOrigin();
    ROS_INFO("hand_distance: %f", hand_left_to_right.length());

  }
#endif
}

void UserInteraction::publishTransforms(const SkeletonsConstPtr& skelentons)
{
  for (int i = 0; i < Skeletons::SKELETON_COUNT; i++)
  {
    if (skelentons->skeletons[i].skeleton_tracking_state == Skeleton::SKELETON_TRACKED)
    {
      //Body
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_HIP_CENTER], skelentons->header.frame_id, "hip_center");
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_SPINE], skelentons->header.frame_id, "spine");
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_SHOULDER_CENTER], skelentons->header.frame_id, "shoulder_center");
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_HEAD], skelentons->header.frame_id, "head");

      //Left hand
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_SHOULDER_LEFT], skelentons->header.frame_id, "shoulder_left");
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_ELBOW_LEFT], skelentons->header.frame_id, "elbow_left");
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_WRIST_LEFT], skelentons->header.frame_id, "wrist_left");
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_HAND_LEFT], skelentons->header.frame_id, "hand_left");

      //Right hand
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_SHOULDER_RIGHT], skelentons->header.frame_id, "shoulder_right");
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_ELBOW_RIGHT], skelentons->header.frame_id, "elbow_right");
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_WRIST_RIGHT], skelentons->header.frame_id, "wrist_right");
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_HAND_RIGHT], skelentons->header.frame_id, "hand_right");

      //Left leg
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_HIP_LEFT], skelentons->header.frame_id, "hip_left");
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_KNEE_LEFT], skelentons->header.frame_id, "knee_left");
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_ANKLE_LEFT], skelentons->header.frame_id, "ankle_left");
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_FOOT_LEFT], skelentons->header.frame_id, "foot_left");

      //Right leg
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_HIP_RIGHT], skelentons->header.frame_id, "hip_right");
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_KNEE_RIGHT], skelentons->header.frame_id, "knee_right");
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_ANKLE_RIGHT], skelentons->header.frame_id, "ankle_right");
      publishTransform(i, skelentons->skeletons[i].skeleton_positions[Skeleton::SKELETON_POSITION_FOOT_RIGHT], skelentons->header.frame_id, "foot_right");
    }
  }
}

void UserInteraction::publishTransform(int user,
                                              const geometry_msgs::Transform& position,
                                              std::string const& frame_id,
                                              std::string const& child_frame_id)
{


  char child_frame_no[128];
  snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

  tf::Transform transform;
  tf::transformMsgToTF(position, transform);

  br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
}

void UserInteraction::getSkeletionMarker(const Skeleton& skeleton,
                                                 const std::string& frame_id,
                                                 const std_msgs::ColorRGBA& color_joint,
                                                 const std_msgs::ColorRGBA& color_link,
                                                 visualization_msgs::MarkerArray& marker_array)
{
  Marker links;
  Marker joints;
  joints.type = Marker::SPHERE_LIST;
  joints.lifetime = ros::Duration(0.1);
  joints.ns = "KinectInteraction";
  joints.header.frame_id = frame_id;
  joints.id = skeleton_marker_id_++;
  joints.scale.x = joints.scale.y = joints.scale.z = 0.05;
  joints.pose.position.x = 0;
  joints.pose.position.y = 0;
  joints.pose.position.z = 0;
  joints.pose.orientation.x = 0;
  joints.pose.orientation.y = 0;
  joints.pose.orientation.z = 0;
  joints.pose.orientation.w = 1;
  joints.color = color_joint;
  links =  joints;
  links.type = Marker::LINE_LIST;
  links.id = skeleton_marker_id_++;
  links.color = color_link;
  links.scale.x = links.scale.y = links.scale.z = 0.02;

  geometry_msgs::Point point;
  for(int i = 0; i < Skeleton::SKELETON_POSITION_COUNT; i++)
  {
    point.x = skeleton.skeleton_positions[i].translation.x;
    point.y = skeleton.skeleton_positions[i].translation.y;
    point.z = skeleton.skeleton_positions[i].translation.z;
    joints.points.push_back(point);
  }
  marker_array.markers.push_back(joints);


  //upper body
  int shoulder_center_state = skeleton.skeleton_position_tracking_state[Skeleton::SKELETON_POSITION_SHOULDER_CENTER];
  if(shoulder_center_state == Skeleton::SKELETON_POSITION_TRACKED ||  shoulder_center_state == Skeleton::SKELETON_POSITION_INFERRED)
  {
    //Head
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_SHOULDER_CENTER]);
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_HEAD]);

    //Left
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_SHOULDER_CENTER]);
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_SHOULDER_LEFT]);

    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_SHOULDER_LEFT]);
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_ELBOW_LEFT]);

    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_ELBOW_LEFT]);
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_WRIST_LEFT]);

    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_WRIST_LEFT]);
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_HAND_LEFT]);

    //Right
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_SHOULDER_CENTER]);
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_SHOULDER_RIGHT]);

    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_SHOULDER_RIGHT]);
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_ELBOW_RIGHT]);

    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_ELBOW_RIGHT]);
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_WRIST_RIGHT]);

    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_WRIST_RIGHT]);
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_HAND_RIGHT]);
  }

  //lower body

  marker_array.markers.push_back(links);
}

void UserInteraction::updateExpandState()
{
    QList<QtBrowserItem *> list = property_editor_->topLevelItems();
    QListIterator<QtBrowserItem *> it(list);
    while (it.hasNext()) {
        QtBrowserItem *item = it.next();
        QtProperty *prop = item->property();
        id_to_expanded_[property_to_id_[prop]] = property_editor_->isExpanded(item);
    }
}

void UserInteraction::addProperty(QtProperty *property, const QString &id)
{
  property_to_id_[property] = id;
  id_to_property_[id] = property;
  QtBrowserItem *item = property_editor_->addProperty(property);
  if (id_to_expanded_.contains(id))
    property_editor_->setExpanded(item, id_to_expanded_[id]);
}

void UserInteraction::gestureDetectorItemClicked(QObject* item)
{
  updateExpandState();

  QMap<QtProperty *, QString>::ConstIterator it_prop = property_to_id_.constBegin();
  while (it_prop != property_to_id_.constEnd())
  {
    delete it_prop.key();
    it_prop++;
  }

  property_to_id_.clear();
  id_to_property_.clear();

  current_item_ = dynamic_cast<GestureDetectorItem*>(item);
  if (!current_item_)
  {
    return;
  }

  QtProperty *property;
  property = string_manager_->addProperty(tr("Name"));
  string_manager_->setValue(property, current_item_->objectName());
  addProperty(property, QLatin1String("name"));

  property = bool_manager_->addProperty(tr("Draw history"));
  bool_manager_->setValue(property, current_item_->getDrawHistory());
  addProperty(property, QLatin1String("draw_history"));

  property = bool_manager_->addProperty(tr("Draw result"));
  bool_manager_->setValue(property, current_item_->getDrawResult());
  addProperty(property, QLatin1String("draw_result"));




  switch(current_item_->rtti())
  {
    case GestureDetectorItem::Rtti_HandPushPull:
    {
      GestureDetectorHandPushPull* i = dynamic_cast<GestureDetectorHandPushPull*>(current_item_);
      property = double_manager_->addProperty("R1");
      double_manager_->setRange(property, 0.01, 1.0);
      double_manager_->setSingleStep(property, 0.01);
      double_manager_->setValue(property, i->getR1());
      addProperty(property, QLatin1String("r1"));

      property = double_manager_->addProperty("R2");
      double_manager_->setRange(property, 0.01, 1.0);
      double_manager_->setSingleStep(property, 0.01);
      double_manager_->setValue(property, i->getR2());
      addProperty(property, QLatin1String("r2"));

      property = double_manager_->addProperty("R3");
      double_manager_->setRange(property, 0.01, 1.0);
      double_manager_->setSingleStep(property, 0.01);
      double_manager_->setValue(property, i->getR3());
      addProperty(property, QLatin1String("r3"));

      property = double_manager_->addProperty("Time out");
      double_manager_->setRange(property, 0.01, 5.0);
      double_manager_->setSingleStep(property, 0.01);
      double_manager_->setValue(property, i->getTimeOut());
      addProperty(property, QLatin1String("time_out"));

      property = double_manager_->addProperty("Activating time");
      double_manager_->setRange(property, 0.01, 5.0);
      double_manager_->setSingleStep(property, 0.01);
      double_manager_->setValue(property, i->getActivatingTime());
      addProperty(property, QLatin1String("activating_time"));
    }
    break;
    case GestureDetectorItem::Rtti_BodyMovement:
    {
      GestureBodyMovement* i = dynamic_cast<GestureBodyMovement*>(current_item_);
      property = double_manager_->addProperty("R1");
      double_manager_->setRange(property, 0.01, 1.0);
      double_manager_->setSingleStep(property, 0.01);
      double_manager_->setValue(property, i->getR1());
      addProperty(property, QLatin1String("r1"));

      property = double_manager_->addProperty("R2");
      double_manager_->setRange(property, 0.01, 1.0);
      double_manager_->setSingleStep(property, 0.01);
      double_manager_->setValue(property, i->getR2());
      addProperty(property, QLatin1String("r2"));

      property = double_manager_->addProperty("R3");
      double_manager_->setRange(property, 0.01, 1.0);
      double_manager_->setSingleStep(property, 0.01);
      double_manager_->setValue(property, i->getR3());
      addProperty(property, QLatin1String("r3"));

      property = double_manager_->addProperty("Time out");
      double_manager_->setRange(property, 0.01, 5.0);
      double_manager_->setSingleStep(property, 0.01);
      double_manager_->setValue(property, i->getTimeOut());
      addProperty(property, QLatin1String("time_out"));

      property = double_manager_->addProperty("Activating time");
      double_manager_->setRange(property, 0.01, 5.0);
      double_manager_->setSingleStep(property, 0.01);
      double_manager_->setValue(property, i->getActivatingTime());
      addProperty(property, QLatin1String("activating_time"));
    }
    break;
    default:
      break;
  }


}

void UserInteraction::gestureDetectorItemDeleted(QObject* item)
{
  QGraphicsItem* i = dynamic_cast<QGraphicsItem*>(item);
  if(i)
  {
    ROS_DEBUG("removed: %s", item->objectName().toStdString().c_str());
    scene_->removeItem(i);
    gesture_detector_items_.remove(item->objectName());
    delete item;
    gestureDetectorItemClicked(0);
  }
}


void UserInteraction::valueChanged(QtProperty *property, double value)
{
  if (!property_to_id_.contains(property))
    return;

  if (!current_item_)
    return;

  QString id = property_to_id_[property];
  ROS_INFO("%s", id.toStdString().c_str());
  if(current_item_->rtti() == GestureDetectorItem::Rtti_HandPushPull)
  {
    GestureDetectorHandPushPull* i = dynamic_cast<GestureDetectorHandPushPull*>(current_item_);
    if(id == QLatin1String("r1")) i->setR1(value);
    else if(id == QLatin1String("r2")) i->setR2(value);
    else if(id == QLatin1String("r3")) i->setR3(value);
    else if(id == QLatin1String("time_out")) i->setTimeOut(value);
    else if(id == QLatin1String("activating_time")) i->setActivatingTime(value);
  }
  if(current_item_->rtti() == GestureDetectorItem::Rtti_BodyMovement)
  {
    GestureBodyMovement* i = dynamic_cast<GestureBodyMovement*>(current_item_);
    if(id == QLatin1String("r1")) i->setR1(value);
    else if(id == QLatin1String("r2")) i->setR2(value);
    else if(id == QLatin1String("r3")) i->setR3(value);
    else if(id == QLatin1String("time_out")) i->setTimeOut(value);
    else if(id == QLatin1String("activating_time")) i->setActivatingTime(value);
  }
}

void UserInteraction::valueChanged(QtProperty *property, int value)
{

}

void UserInteraction::valueChanged(QtProperty *property, bool value)
{
  if (!property_to_id_.contains(property))
    return;

  if (!current_item_)
    return;

  QString id = property_to_id_[property];
  ROS_INFO("%s", id.toStdString().c_str());


}

void UserInteraction::valueChanged(QtProperty *property, const QString &value)
{

}




UserInteraction* g_handle = NULL;
bool g_initialized = false;

void spin_function()
{
  ros::WallRate r(100.0);
  while (ros::ok() && !g_initialized)
  {
    r.sleep();
    ros::spinOnce();
  }
  while (ros::ok() && !g_handle->quit_threads_)
  {
    r.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hg_kinect_interaction", ros::init_options::NoSigintHandler);

  boost::thread spin_thread(boost::bind(&spin_function));

  QApplication a(argc, argv);



  UserInteraction w;
  g_handle = &w;
  w.show();

  g_initialized = w.initialize();
  if(!g_initialized)
    exit(-1);

  int ret = a.exec();

  spin_thread.join();

  return ret;
}
