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

#ifndef INSPECTION_POINT_H_
#define INSPECTION_POINT_H_

#include "qtpropertymanager.h"
#include "qteditorfactory.h"
#include "qttreepropertybrowser.h"

#include <sensor_msgs/JointState.h>

#include <hg_inspector_arm/inspection_point_setget.h>

class InspectorArm;
class InspectionPointLookAt;
class InspectionPointItem;


typedef QList<InspectionPointItem*> InspectionPointItemList;

class InspectionPointItem
{
public:
  InspectionPointItem(InspectorArm* inspector_arm,
                         interactive_markers::InteractiveMarkerServer* server,
                         const geometry_msgs::Pose& pose = geometry_msgs::Pose());
  virtual ~InspectionPointItem();

  enum RttiValue
  {
    Rtti_Item = 0,
    Rtti_LookAt = 1
  };

  virtual int rtti() const;
  static int RTTI;

  QString name() { return name_; }
  void setName(const QString& a) { name_ = a; }
  inline double x() const { return pose_.position.x; }
  inline double y() const { return pose_.position.y; }
  inline double z() const { return pose_.position.z; }
  inline double roll() const
  {
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose_.orientation, q);
    tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
    return roll;
  }
  inline double pitch() const
  {
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose_.orientation, q);
    tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
    return pitch;
  }
  inline double yaw() const
  {
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose_.orientation, q);
    tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
    return yaw;
  }

  IP_GETSET(double, marker_scale_, MarkerScale);

  void setX(double a) { move(a, y(), z()); }
  void setY(double a) { move(x(), a, z()); }
  void setZ(double a) { move(x(), y(), a); }
  void setRoll(double a) { rotate(a, pitch(), yaw()); }
  void setPitch(double a) { rotate(roll(), a, yaw()); }
  void setYaw(double a) { rotate(roll(), pitch(), a); }

  bool setPose(const geometry_msgs::Pose& pose, bool check_ik, bool update_marker=true);
  bool setPose(const tf::Transform& tf, bool check_ik, bool update_marker=true);
  void setJointState(const  sensor_msgs::JointState& joint_state) { joint_state_ = joint_state; }
  void move(double x, double y, double z);
  virtual void moveBy(double x, double y, double z);
  void rotate(double roll, double pitch, double yaw);
  virtual void rotateBy(double roll, double pitch, double yaw);

  geometry_msgs::Pose pose() const { return pose_; };
  sensor_msgs::JointState jointState() const { return joint_state_; };

  virtual void save(QDataStream& out);
  virtual void load(QDataStream& in);

protected:
  InspectorArm* inspector_arm_;
  interactive_markers::InteractiveMarkerServer* server_;
  QString name_;
  geometry_msgs::Pose pose_;
  sensor_msgs::JointState joint_state_;
  ros::Duration duration_; //time to stop before move to next point
  double marker_scale_;


};

class InspectionPointLookAt : public InspectionPointItem
{
public:
  InspectionPointLookAt();
  ~InspectionPointLookAt();

  void setZoom(double a);
  double zoomFactor() const { return zoom_factor_; }

  int rtti() const;
  static int RTTI;

private:
  double zoom_factor_;

};








#endif /* INSPECTION_POINT_H_ */
