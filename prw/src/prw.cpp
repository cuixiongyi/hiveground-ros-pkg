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

#include <unistd.h>

#include <ros/ros.h>
#include <prw/prw.h>
#include <QtGui/QApplication>
#include <QDebug>

using namespace ros::param;
using namespace std;

PRW::PRW(const hg::WorkspaceEditorParameters& parameters, QWidget *parent, Qt::WFlags flags) :
    QMainWindow(parent, flags),
    WorkspaceEditor(parameters),
    quit_threads_(false)
{
  ui.setupUi(this);

}

PRW::~PRW()
{
}

void PRW::initialize()
{
  //ROS_INFO_STREAM("Initializing...");

  //ROS_INFO_STREAM("Initialized");
}

void PRW::on_ik_move_go_clicked()
{
  qDebug() << __FUNCTION__;
  geometry_msgs::Pose pose;
  tf::Quaternion quat;
  quat.setRPY( ui.ik_move_roll->value(), ui.ik_move_yaw->value(), ui.ik_move_pitch->value());
  pose.position.x = ui.ik_move_x->value();
  pose.position.y = ui.ik_move_y->value();
  pose.position.z = ui.ik_move_z->value();
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
  interactive_marker_server_->setPose("arm", pose);
  interactive_marker_server_->applyChanges();

  PlanningGroupData& gc = group_map_[current_group_name_];
  tf::Transform cur = toBulletTransform(pose);
  setNewEndEffectorPosition(gc, cur, collision_aware_);
  last_ee_poses_[current_group_name_] = pose;

}

void PRW::on_ik_move_reset_clicked()
{
  qDebug() << __FUNCTION__;
  ui.ik_move_x->setValue(0.28);
  ui.ik_move_y->setValue(0.0);
  ui.ik_move_z->setValue(0.565);
  ui.ik_move_roll->setValue(0.0);
  ui.ik_move_pitch->setValue(0.0);
  ui.ik_move_yaw->setValue(0.0);
}

void PRW::on_new_scene_clicked()
{
  qDebug() << __FUNCTION__;
  //createNewPlanningScene("test", 0);
}

void PRW::on_new_mpr_clicked()
{
  qDebug() << __FUNCTION__;
  unsigned int plan_id = 0;
  unsigned int id;
  //createMotionPlanRequest(*robot_state_, *robot_state_,
    //                      "arm", "link5", plan_id, true, id);

}

void PRW::on_plan_clicked()
{
  qDebug() << __FUNCTION__;

}


void PRW::closeEvent(QCloseEvent *event)
{
  ROS_INFO("Close windows");
  quit_threads_ = true;
  event->accept();
}

#define CONTROL_SPEED 5

PRW* prw_ = NULL;
bool initialized_ = false;

void marker_function()
{
  unsigned int counter = 0;
  while(ros::ok())
  {
    if(initialized_)
    {
      if(counter % CONTROL_SPEED == 0)
      {
        counter = 1;
        prw_->sendMarkers();
      }
      else
      {
        counter++;
      }

      if(prw_->quit_threads_)
      {
        break;
      }
    }
    usleep(5000);
  }
}


void spin_function()
{
  ros::WallRate r(100.0);
  while (ros::ok() && !initialized_)
  {
    r.sleep();
    ros::spinOnce();
  }
  while (ros::ok() && !prw_->quit_threads_)
  {
    r.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "personal_robotic_workspace", ros::init_options::NoSigintHandler);

  boost::thread spin_thread(boost::bind(&spin_function));
  boost::thread marker_thread(boost::bind(&marker_function));

  QApplication a(argc, argv);

  hg::WorkspaceEditorParameters parameters;

  PRW w(parameters);
  w.initialize();
  prw_ = &w;
  w.show();

  initialized_ = true;

  int ret = a.exec();

  spin_thread.join();

  return ret;
}
