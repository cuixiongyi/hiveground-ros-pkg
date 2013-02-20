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

#ifndef KINECT_INTERACTION_H_
#define KINECT_INTERACTION_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <QtGui>
#include "ui_kinect_interaction.h"

#include <kinect_msgs/Skeletons.h>

namespace hg_kinect_interaction
{

class KinectInteraction : public QMainWindow
{
  Q_OBJECT
public:
  KinectInteraction(QWidget *parent = 0, Qt::WFlags flags = 0);
  ~KinectInteraction();

  bool initialize();

protected:
  void skeletonsCallback(const kinect_msgs::SkeletonsConstPtr& message);
  void publishTransforms(const kinect_msgs::SkeletonsConstPtr& message);
  void publishTransform(int user,
                           const geometry_msgs::Transform& position,
                           std::string const& frame_id,
                           std::string const& child_frame_id);
public:
  Ui::KinectInteraction ui;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  bool quit_threads_;


  tf::TransformBroadcaster br_;
  ros::Subscriber skeleton_sub_;

};




}




#endif /* KINECT_INTERACTION_H_ */
