/*
 * Copyright (c) 2013, HiveGround Co., Ltd.
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
 *      * Neither the name of the HiveGround Co., Ltd., nor the name of its
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
 *
 */

#include <sys/mman.h>
#include <signal.h>

#include <denso_robots/vp6242_robot.h>
#include <hg_controller_manager/hg_controller_manager.h>

#define REALTIME 0

using namespace std;

bool g_quit = false;

void quitRequested(int sig)
{
  g_quit = true;
}

boost::thread g_control_thread;

void controlThread()
{
#if REALTIME
  //set up thread priority
  int retcode;
  int policy;
  pthread_t thread_id = (pthread_t)g_control_thread.native_handle();
  struct sched_param param;

  if ((retcode = pthread_getschedparam(thread_id, &policy, &param)) != 0)
  {
    errno = retcode;
    perror("pthread_getschedparam");
  }

  ROS_INFO_STREAM("Control thread inherited: policy= " << ((policy == SCHED_FIFO) ? "SCHED_FIFO" : (policy == SCHED_RR) ? "SCHED_RR" : (policy == SCHED_OTHER) ? "SCHED_OTHER" : "???") << ", priority=" << param.sched_priority);

  policy = SCHED_FIFO;
  param.sched_priority = sched_get_priority_max(policy);

  if ((retcode = pthread_setschedparam(thread_id, policy, &param)) != 0)
  {
    errno = retcode;
    perror("pthread_setschedparam");
  }

  ros::Duration(1.0).sleep();

  if ((retcode = pthread_getschedparam(thread_id, &policy, &param)) != 0)
  {
    errno = retcode;
    perror("pthread_getschedparam");
  }

  ROS_INFO_STREAM("Control thread changed: policy= " << ((policy == SCHED_FIFO) ? "SCHED_FIFO" :
		                                         (policy == SCHED_RR) ? "SCHED_RR" :
		                                         (policy == SCHED_OTHER) ? "SCHED_OTHER" : "???") << ", priority=" << param.sched_priority);
  //set up thread priority
#endif

  ros::NodeHandle nh("~");

  urdf::Model urdf_model;
  urdf_model.initParam("robot_description");

  denso_common::VP6242Robot vp6242(nh, urdf_model, "arm0_");
  denso_common::VP6242Robot vp6242_1(nh, urdf_model, "arm1_");


  hg_controller_manager::ControllerManager cm(&vp6242, nh, "arm0_");
  hg_controller_manager::ControllerManager cm_1(&vp6242_1, nh, "arm1_");

  hg_realtime_tools::RealtimePublisher<sensor_msgs::JointState> pub_joint_state(nh, "/joint_states", 1);

  if (!(vp6242.start() && vp6242_1.start()))
  {
    ROS_ERROR("Cannot start robot!");
    return;
  }

  //cm.loadController("JPAC0");
  cm.loadController("JPAC0");
  cm_1.loadController("JPAC1");

  ros::Rate rate(1000);
  ros::Time last_joint_state_publish_time = ros::Time::now();
  while (!g_quit)
  {
    if(!(vp6242.read() && vp6242_1.read()))
    {
      g_quit = true;
      break;
    }

    cm.update(ros::Time::now(), rate.expectedCycleTime());
    cm_1.update(ros::Time::now(), rate.expectedCycleTime());

    if(!(vp6242.write() && vp6242_1.write()))
    {
      g_quit = true;
      break;
    }

    double dt_joint_state = (ros::Time::now() - last_joint_state_publish_time).toSec();
    if(dt_joint_state > (10.0 * rate.expectedCycleTime().toSec()))
    {
      if (pub_joint_state.trylock())
      {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name.insert(msg.name.end(), vp6242.getJointName().begin(), vp6242.getJointName().end());
        msg.name.insert(msg.name.end(), vp6242_1.getJointName().begin(), vp6242_1.getJointName().end());
        msg.position.insert(msg.position.end(), vp6242.getJointPosition().begin(), vp6242.getJointPosition().end());
        msg.position.insert(msg.position.end(), vp6242_1.getJointPosition().begin(), vp6242_1.getJointPosition().end());
        msg.velocity.insert(msg.velocity.end(), vp6242.getJointVelocity().begin(), vp6242.getJointVelocity().end());
        msg.velocity.insert(msg.velocity.end(), vp6242_1.getJointVelocity().begin(), vp6242_1.getJointVelocity().end());
        msg.effort.insert(msg.effort.end(), vp6242.getJointEffort().begin(), vp6242.getJointEffort().end());
        msg.effort.insert(msg.effort.end(), vp6242_1.getJointEffort().begin(), vp6242_1.getJointEffort().end());
        pub_joint_state.msg_ = msg;
        pub_joint_state.unlockAndPublish();
        last_joint_state_publish_time = ros::Time::now();
      }
    }

    rate.sleep();
  }

  vp6242.stop();
  vp6242_1.stop();
  ros::shutdown();
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "vp6242_helping_hand_node");

  ros::NodeHandle nh("~");

#if REALTIME  
  // Keep the kernel from swapping us out
  if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
    perror("mlockall");
    return -1;
  }
#endif

  // Catch attempts to quit
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  g_control_thread = boost::thread(&controlThread);

  ros::spin();

  g_control_thread.join();

  return 0;
}


