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


#include <hg_kinect_interaction/kinect_interaction.h>
#include <boost/thread.hpp>

using namespace hg_kinect_interaction;

KinectInteraction::KinectInteraction(QWidget *parent, Qt::WFlags flags)
  : QMainWindow(parent, flags),
    nh_(), nh_private_("~"),
    quit_threads_(false)
{

}

KinectInteraction::~KinectInteraction()
{

}

bool KinectInteraction::initialize()
{


  skeleton_sub_ = nh_private_.subscribe("skeletons_in", 1, &KinectInteraction::skeletonsCallback, this);
  return true;
}


void KinectInteraction::skeletonsCallback(const kinect_msgs::SkeletonsConstPtr& message)
{
  ROS_INFO_THROTTLE(1.0, __FUNCTION__);
  //ROS_INFO_STREAM(message->header);





  publishTransforms(message);
}

void KinectInteraction::publishTransforms(const kinect_msgs::SkeletonsConstPtr& message)
{
  for (int i = 0; i < kinect_msgs::Skeletons::SKELETON_COUNT; i++)
  {
    if (message->skeletons[i].skeleton_tracking_state == kinect_msgs::Skeleton::SKELETON_TRACKED)
    {
      //Body
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_HIP_CENTER], message->header.frame_id, "hip_center");
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_SPINE], message->header.frame_id, "spine");
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_SHOULDER_CENTER], message->header.frame_id, "shoulder_center");
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_HEAD], message->header.frame_id, "head");

      //Left hand
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_SHOULDER_LEFT], message->header.frame_id, "shoulder_left");
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_ELBOW_LEFT], message->header.frame_id, "elbow_left");
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_WRIST_LEFT], message->header.frame_id, "wrist_left");
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_HAND_LEFT], message->header.frame_id, "hand_left");

      //Right hand
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_SHOULDER_RIGHT], message->header.frame_id, "shoulder_right");
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_ELBOW_RIGHT], message->header.frame_id, "elbow_right");
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_WRIST_RIGHT], message->header.frame_id, "wrist_right");
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_HAND_RIGHT], message->header.frame_id, "hand_right");

      //Left leg
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_HIP_LEFT], message->header.frame_id, "hip_left");
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_KNEE_LEFT], message->header.frame_id, "knee_left");
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_ANKLE_LEFT], message->header.frame_id, "ankle_left");
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_FOOT_LEFT], message->header.frame_id, "foot_left");

      //Right leg
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_HIP_RIGHT], message->header.frame_id, "hip_right");
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_KNEE_RIGHT], message->header.frame_id, "knee_right");
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_ANKLE_RIGHT], message->header.frame_id, "ankle_right");
      publishTransform(i, message->skeletons[i].skeleton_positions[kinect_msgs::Skeleton::SKELETON_POSITION_FOOT_RIGHT], message->header.frame_id, "foot_right");
    }
  }
}

void KinectInteraction::publishTransform(int user,
                                              const geometry_msgs::Transform& position,
                                              std::string const& frame_id,
                                              std::string const& child_frame_id)
{


  char child_frame_no[128];
  snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

  tf::Transform transform;
  tf::transformMsgToTF(position, transform);

  tf::Transform change_frame;
  change_frame.setOrigin(tf::Vector3(0, 0, 0));
  tf::Quaternion frame_rotation;
  frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
  change_frame.setRotation(frame_rotation);
  transform = change_frame * transform;

  br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
}


KinectInteraction* g_handle = NULL;
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



  KinectInteraction w;
  g_handle = &w;
  w.show();

  g_initialized = w.initialize();
  if(!g_initialized)
    exit(-1);

  int ret = a.exec();

  spin_thread.join();

  return ret;
}
