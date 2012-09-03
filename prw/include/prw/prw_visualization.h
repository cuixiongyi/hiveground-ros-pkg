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

#ifndef PRW_VISUALIZATION_H
#define PRW_VISUALIZATION_H

#include <prw/prw_utils.h>

namespace hg
{

void makeIKControllerMarker(planning_environment::CollisionModels* cm,
                            interactive_markers::InteractiveMarkerServer* server,
                            interactive_markers::MenuHandler::FeedbackCallback& process_function_ptr_,
                            tf::Transform transform, std::string name, std::string description, float scale)
{
  visualization_msgs::InteractiveMarker marker;
  marker.header.frame_id = "/" + cm->getWorldFrameId();
  marker.pose.position.x = transform.getOrigin().x();
  marker.pose.position.y = transform.getOrigin().y();
  marker.pose.position.z = transform.getOrigin().z();
  marker.pose.orientation.w = transform.getRotation().w();
  marker.pose.orientation.x = transform.getRotation().x();
  marker.pose.orientation.y = transform.getRotation().y();
  marker.pose.orientation.z = transform.getRotation().z();
  marker.scale = 0.225f;
  marker.name = name;
  marker.description = description;

  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = false;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0;

  marker.controls.push_back(control);

  visualization_msgs::InteractiveMarkerControl control2;
  control2.always_visible = false;
  control2.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  control2.orientation.w = 1;
  control2.orientation.x = 0;
  control2.orientation.y = 1;
  control2.orientation.z = 0;

  visualization_msgs::Marker marker2;
  marker2.type = visualization_msgs::Marker::CUBE;
  marker2.scale.x = .2;
  marker2.scale.y = .15;
  marker2.scale.z = .002;
  marker2.pose.position.x = .1;
  marker2.color.r = 0;
  marker2.color.g = 0;
  marker2.color.b = 0.5;
  marker2.color.a = 1;
  control2.markers.push_back(marker2);
  marker2.scale.x = .1;
  marker2.scale.y = .35;
  marker2.pose.position.x = 0;
  control2.markers.push_back(marker2);

  marker.controls.push_back(control2);

  visualization_msgs::InteractiveMarkerControl control3;
  control3.always_visible = false;
  control3.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  control3.orientation.w = 1;
  control3.orientation.x = 0;
  control3.orientation.y = 0;
  control3.orientation.z = 1;

  visualization_msgs::Marker marker3;
  marker3.type = visualization_msgs::Marker::CUBE;
  marker3.scale.x = .2;
  marker3.scale.y = .002;
  marker3.scale.z = .15;
  marker3.pose.position.x = .1;
  marker3.color.r = 0;
  marker3.color.g = .5;
  marker3.color.b = 0;
  marker3.color.a = 1;
  control3.markers.push_back(marker3);
  marker3.scale.x = .1;
  marker3.scale.z = .35;
  marker3.pose.position.x = 0;
  control3.markers.push_back(marker3);

  marker.controls.push_back(control3);

  server->insert(marker, process_function_ptr_);
}





}

#endif
