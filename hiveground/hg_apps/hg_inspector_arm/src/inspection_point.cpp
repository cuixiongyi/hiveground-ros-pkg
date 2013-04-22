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

#include <hg_inspector_arm/inspector_arm.h>

int InspectionPointItem::RTTI = Rtti_Item;
int InspectionPointItem::rtti() const { return RTTI; }

int InspectionPointLookAt::RTTI = Rtti_LookAt;
int InspectionPointLookAt::rtti() const { return RTTI; }

InspectionPointItem::InspectionPointItem(InspectorArm* inspector_arm,
                                               interactive_markers::InteractiveMarkerServer* server,
                                               const geometry_msgs::Pose& pose)
  : inspector_arm_(inspector_arm), server_(server), pose_(pose)
{

}

InspectionPointItem::~InspectionPointItem()
{

}

bool InspectionPointItem::setPose(const geometry_msgs::Pose& pose, bool check_ik, bool update_marker)
{
  if(check_ik)
  {
    sensor_msgs::JointState joint_state = joint_state_;
    if(inspector_arm_->checkIKConstraintAware(pose, joint_state))
    {
      pose_ = pose;
      joint_state_ = joint_state;
      if(update_marker)
      {
        server_->setPose(name_.toStdString(), pose_);
        server_->applyChanges();
      }
      Q_EMIT inspector_arm_->inspectionPointMovedSignal(this);
      Q_EMIT inspector_arm_->followPointSignal();
      return true;
    }
    return false;
  }
  else
  {
    pose_ = pose;
    if(update_marker)
    {
      server_->setPose(name_.toStdString(), pose_);
      server_->applyChanges();
    }
    Q_EMIT inspector_arm_->inspectionPointMovedSignal(this);
    Q_EMIT inspector_arm_->followPointSignal();
    return true;
  }
}

bool InspectionPointItem::setPose(const tf::Transform& tf, bool check_ik, bool update_marker)
{
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(tf, pose);
  return setPose(pose, check_ik, update_marker);
}

void InspectionPointItem::move(double x, double y, double z)
{
  moveBy(x - pose_.position.x, y - pose_.position.y, z - pose_.position.z);
}

void InspectionPointItem::moveBy(double dx, double dy, double dz)
{
  if((dx != 0.0) || (dy != 0.0) || (dz != 0.0))
  {
    geometry_msgs::Pose old_pose = pose_;
    sensor_msgs::JointState joint_state = joint_state_;
    pose_.position.x += dx;
    pose_.position.y += dy;
    pose_.position.z += dz;

    tf::Transform marker_pose;
    tf::poseMsgToTF(pose_, marker_pose);
    tf::Transform pose_ee = marker_pose;
    if(name_.toStdString().rfind("marker_tool") != std::string::npos)
    {
      tf::StampedTransform tfs;
      inspector_arm_->listener_.lookupTransform(inspector_arm_->tool_frame_,
                                                inspector_arm_->ik_solver_info_.kinematic_solver_info.link_names[0],
                                                ros::Time(0), tfs);
      pose_ee = marker_pose * tfs;
    }


    if(inspector_arm_->checkIKConstraintAware(pose_ee, joint_state))
    {
      joint_state_ = joint_state;
      server_->setPose(name_.toStdString(), pose_);
      server_->applyChanges();
      Q_EMIT inspector_arm_->followPointSignal();
    }
    else
    {
      pose_ = old_pose;
      inspector_arm_->inspectionPointMoved(this);
    }
  }
}

void InspectionPointItem::rotate(double roll, double pitch, double yaw)
{
  rotateBy(roll - this->roll(), pitch - this->pitch(), yaw - this->yaw());
}

void InspectionPointItem::rotateBy(double dRoll, double dPitch, double dYaw)
{
  if((dRoll != 0.0) || (dPitch != 0.0) || (dYaw != 0.0))
  {
    geometry_msgs::Pose old_pose = pose_;
    sensor_msgs::JointState joint_state;
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose_.orientation, q);
    tf::Quaternion dq;
    dq.setRPY(dRoll, dPitch, dYaw);
    q = q * dq;
    tf::quaternionTFToMsg(q, pose_.orientation);

    tf::Transform marker_pose;
    tf::poseMsgToTF(pose_, marker_pose);
    tf::Transform pose_ee = marker_pose;

    if(name_.toStdString().rfind("marker_tool") != std::string::npos)
    {
      tf::StampedTransform tfs;
      inspector_arm_->listener_.lookupTransform(inspector_arm_->tool_frame_,
                                                inspector_arm_->ik_solver_info_.kinematic_solver_info.link_names[0],
                                                ros::Time(0), tfs);
      pose_ee = marker_pose * tfs;
    }

    if(inspector_arm_->checkIKConstraintAware(pose_ee, joint_state))
    {
      joint_state_ = joint_state;
      server_->setPose(name_.toStdString(), pose_);
      server_->applyChanges();
      Q_EMIT inspector_arm_->followPointSignal();
    }
    else
    {
      pose_ = old_pose;
      inspector_arm_->inspectionPointMoved(this);
    }
  }
}

void InspectionPointItem::save(QDataStream& out)
{
  out << RTTI;
  out << name_;
  out << pose_.position.x;
  out << pose_.position.y;
  out << pose_.position.z;
  out << pose_.orientation.x;
  out << pose_.orientation.y;
  out << pose_.orientation.z;
  out << pose_.orientation.w;
  out << (qint32)joint_state_.position.size();
  for(int i = 0; i < (int)joint_state_.position.size(); i++)
  {
    out << joint_state_.position[i];
  }
  out << marker_scale_;
}

void InspectionPointItem::load(QDataStream& in)
{
  in >> name_;
  in >> pose_.position.x;
  in >> pose_.position.y;
  in >> pose_.position.z;
  in >> pose_.orientation.x;
  in >> pose_.orientation.y;
  in >> pose_.orientation.z;
  in >> pose_.orientation.w;
  qint32 num_joint;
  in >> num_joint;
  joint_state_.position.resize(num_joint);
  for(int i = 0; i < num_joint; i++)
  {
    in >> joint_state_.position[i];
  }
  in >> marker_scale_;
}


