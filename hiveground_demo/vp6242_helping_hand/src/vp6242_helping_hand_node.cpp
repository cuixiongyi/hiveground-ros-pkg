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

using namespace std;

bool g_quit = false;

void quitRequested(int sig)
{
  g_quit = true;
}

boost::thread g_control_thread;

void controlThread()
{
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

  ROS_INFO_STREAM("Control thread changed: policy= " << ((policy == SCHED_FIFO) ? "SCHED_FIFO" : (policy == SCHED_RR) ? "SCHED_RR" : (policy == SCHED_OTHER) ? "SCHED_OTHER" : "???") << ", priority=" << param.sched_priority);
  //set up thread priority

  ros::NodeHandle nh("~");

  urdf::Model urdf_model;
  urdf_model.initParam("robot_description");

  denso_common::VP6242Robot vp6242(nh, urdf_model);

  hg_controller_manager::ControllerManager cm(&vp6242, nh);

  if (!vp6242.start())
  {
    ROS_ERROR("Cannot start robot!");
    return;
  }

  cm.loadController("JPAC0");

  ros::Rate rate(1000);
  while (!g_quit)
  {
    if(!vp6242.read())
    {
      g_quit = true;
      break;
    }

    cm.update(ros::Time::now(), ros::Duration(0.001));

    if(!vp6242.write())
    {
      g_quit = true;
      break;
    }

    rate.sleep();
  }

  vp6242.stop();
  ros::shutdown();
}

int main(int argc, char** argv)
{
  // Keep the kernel from swapping us out
  if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
    perror("mlockall");
    return -1;
  }

  ros::init(argc, argv, "vp6242_helping_hand_node");

  ros::NodeHandle nh("~");

  // Catch attempts to quit
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  g_control_thread = boost::thread(&controlThread);

  ros::spin();

  g_control_thread.join();

  return 0;
}


