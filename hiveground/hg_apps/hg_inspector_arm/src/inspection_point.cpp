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

void InspectionPointItem::move(double x, double y, double z)
{
  moveBy(x - tf_.getOrigin().getX(), y - tf_.getOrigin().getY(), z - tf_.getOrigin().getZ());
}

void InspectionPointItem::moveBy(double dx, double dy, double dz)
{
  if(dx || dy || dz)
  {
    tf::Vector3 origin = tf_.getOrigin();
    tf_.getOrigin().setValue(origin.x() + dx, origin.y() + dy, origin.z() + dz);
    geometry_msgs::Pose pose;
    tf::poseTFToMsg(tf_, pose);
    server_->setPose(name_.toStdString(), pose);
    server_->applyChanges();
  }
}

void InspectionPointItem::rotate(double roll, double pitch, double yaw)
{

}

void InspectionPointItem::rotateBy(double dRoll, double dPitch, double dYaw)
{

}

void InspectorArm::inspectionPointSelected(InspectionPointItem* item)
{

}
