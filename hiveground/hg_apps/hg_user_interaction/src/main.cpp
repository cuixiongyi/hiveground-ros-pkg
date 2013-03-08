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

  //skeleton_sub_ = nh_private_.subscribe(skeletons_topic_, 1, &UserInteraction::skeletonsCallback, this);
  skeletons_markers_publisher_ = nh_private_.advertise<MarkerArray>("skeletons_makers", 128);


  nh_private_.getParam("skeletons_topic", skeletons_topic_);
  nh_private_.getParam("hands_topic", hands_topic_);
  ROS_INFO_STREAM(skeletons_topic_ << ":" << hands_topic_);

  skeletons_sub_ = boost::shared_ptr<message_filters::Subscriber<kinect_msgs::Skeletons> >(new message_filters::Subscriber<kinect_msgs::Skeletons>(nh_, skeletons_topic_, 1));
  hands_sub_ = boost::shared_ptr<message_filters::Subscriber<hg_object_tracking::Hands> >(new message_filters::Subscriber<hg_object_tracking::Hands>(nh_, hands_topic_, 1));

  skeltons_hands_sync_ = boost::shared_ptr<message_filters::Synchronizer<SkeltonsHandsSyncPolicy> >(new message_filters::Synchronizer<SkeltonsHandsSyncPolicy>(SkeltonsHandsSyncPolicy(50), *skeletons_sub_, *hands_sub_));
  skeltons_hands_sync_->registerCallback(boost::bind(&UserInteraction::skeletonsHandsCallback, this, _1, _2));

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
      ROS_INFO("skeleton_wrist_distance: %f", wrist_left_to_right.length());
      ROS_INFO("skeleton_hand_distance: %f", hand_left_to_right.length());
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
  //ROS_INFO_THROTTLE(1.0, __FUNCTION__);
  //ROS_INFO_STREAM(skelentons->header);

  publishTransforms (skeletons);

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
      //ROS_INFO("wrist_distance: %f", wrist_left_to_right.length());
      //ROS_INFO("hand_distance: %f", hand_left_to_right.length());

    }
  }

  if (skeletons_markers_publisher_.getNumSubscribers() != 0)
  {
    if (!marker_array.markers.empty())
    {
      skeletons_markers_publisher_.publish(marker_array);
    }
  }
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
