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


#include <ros/ros.h>
#include <prw/prw.h>
#include <QtGui/QApplication>

using namespace ros::param;
using namespace std;

PRW::PRW(hg::WorkspaceEditorParameters* parameters, QWidget *parent, Qt::WFlags flags) :
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

}

void PRW::on_ik_move_reset_clicked()
{
  ui.ik_move_x->setValue(0.0);
  ui.ik_move_y->setValue(0.0);
  ui.ik_move_z->setValue(0.0);
  ui.ik_move_roll->setValue(0.0);
  ui.ik_move_pitch->setValue(0.0);
  ui.ik_move_yaw->setValue(0.0);
}

void PRW::closeEvent(QCloseEvent *event)
{
  ROS_INFO("Close windows");
  quit_threads_ = true;
  event->accept();
}

PRW* prw_ = NULL;
bool initialized_ = false;
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

  QApplication a(argc, argv);

  hg::WorkspaceEditorParameters parameters;


  param<int>("number_of_arm", parameters.number_of_arm_, 1);
  stringstream ss;
  string value;
  string label;
  for(int i = 0; i < parameters.number_of_arm_; i++)
  {
    ss.clear();
    ss << "arm_group_" << i;
    ss >> label;
    param<std::string>(label, value, "none");
    parameters.arm_group_.push_back(value);

    ss.clear();
    ss << "arm_controller_" << i;
    ss >> label;
    param<std::string>(label, value, "none");
    parameters.arm_controller_.push_back(value);

    ss.clear();
    ss << "ik_name_" << i;
    ss >> label;
    param<std::string>(label, value, "none");
    parameters.ik_name_.push_back(value);

    ss.clear();
    ss << "ik_link_" << i;
    ss >> label;
    param<std::string>(label, value, "none");
    parameters.ik_link_.push_back(value);

    ss.clear();
    ss << "non_collision_ik_name_" << i;
    ss >> label;
    param<std::string>(label, value, "none");
    parameters.non_collision_ik_name_.push_back(value);
  }

  //ROS_INFO_STREAM(parameters.arm_group_);
  param<std::string>("planner_service_name", parameters.planner_service_name_, "none");
  param<std::string>("trajectory_filter_service_name", parameters.trajectory_filter_service_name_, "none");
  param<std::string>("set_planning_scene_diff_name", parameters.set_planning_scene_diff_name_, "none");
  param<std::string>("visualizer_topic_name", parameters.visualizer_topic_name_, "none");


  //ParameterDialog dialog(parameters);
  //dialog.exec();

  //GET_ROS_PARAM("arm_group_0", parameters.arm_group_[0])
  //ros::param::param<std::string>()

  //ros::param::param<std::string>()


  PRW w(&parameters);
  w.initialize();
  prw_ = &w;
  w.show();

  initialized_ = true;

  int ret = a.exec();

  spin_thread.join();

  return ret;
}
