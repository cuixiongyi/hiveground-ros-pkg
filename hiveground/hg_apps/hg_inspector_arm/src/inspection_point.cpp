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

void InspectionPointItem::setPose(const geometry_msgs::Pose& pose)
{
  sensor_msgs::JointState joint_state;
  if(inspector_arm_->checkIKConstraintAware(pose, joint_state))
  {
    pose_ = pose;
    joint_state_ = joint_state;
    server_->setPose(name_.toStdString(), pose_);
    server_->applyChanges();
    inspector_arm_->inspectionPointMoved(this);
    Q_EMIT inspector_arm_->followPointSignal();
  }
}

void InspectionPointItem::setPose(const tf::Transform& tf)
{
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(tf, pose);
  setPose(pose);
}

void InspectionPointItem::move(double x, double y, double z)
{
  moveBy(x - pose_.position.x, y - pose_.position.y, z - pose_.position.z);
}

void InspectionPointItem::moveBy(double dx, double dy, double dz)
{
  if(dx || dy || dz)
  {
    geometry_msgs::Pose old_pose = pose_;
    sensor_msgs::JointState joint_state;
    pose_.position.x += dx;
    pose_.position.y += dy;
    pose_.position.z += dz;

    if(inspector_arm_->checkIKConstraintAware(pose_, joint_state))
    {
      joint_state_ = joint_state;
      server_->setPose(name_.toStdString(), pose_);
      server_->applyChanges();
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
  if((dRoll != 0.0) || (dPitch != 0) || (dYaw != 0))
  {
    geometry_msgs::Pose old_pose = pose_;
    sensor_msgs::JointState joint_state;
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose_.orientation, q);
    tf::Quaternion dq;
    dq.setRPY(dRoll, dPitch, dYaw);
    q = q * dq;
    tf::quaternionTFToMsg(q, pose_.orientation);

    if(inspector_arm_->checkIKConstraintAware(pose_, joint_state))
    {
      joint_state_ = joint_state;
      server_->setPose(name_.toStdString(), pose_);
      server_->applyChanges();
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


}


