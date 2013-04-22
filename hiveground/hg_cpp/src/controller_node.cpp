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

#include <signal.h>
#include <sys/mman.h>

#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <hg_cpp/controller_node.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sensor_msgs/JointState.h>

using namespace hg;
using namespace std;

#define REAL_TIME 1


bool g_quit = false;

void quitRequested(int sig)
{
  g_quit = true;
}

boost::thread g_control_thread;



ControllerNode::ControllerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
    nh_(nh),
    nh_private_(nh_private),
    is_simulated_(true),
    loop_rate_(50.0),
    joint_publish_rate_(10.0),
    diagnostic_publish_rate_(10.0),
    controller_plugin_loader_("hg_cpp", "hg::Controller"),
    joint_plugin_loader_("hg_cpp", "hg::Joint"),
    pub_joint_state_(nh, "/joint_states", 1)
{
  ROS_ASSERT(urdf_model_.initParam("robot_description"));
  ROS_INFO("Successfully parsed urdf file");


  //note settings
  ROS_ASSERT(nh_private_.getParam("is_simulated", is_simulated_));
  ROS_INFO_STREAM("simulated mode: " << is_simulated_);
  ROS_ASSERT(nh_private_.getParam("loop_rate", loop_rate_));
  ROS_INFO_STREAM("loop rate: " << loop_rate_);
  ROS_ASSERT(nh_private_.getParam("joint_publish_rate", joint_publish_rate_));
  ROS_INFO_STREAM("joint publish rate: " << joint_publish_rate_);
  ROS_ASSERT(nh_private_.getParam("diagnostic_publish_rate", diagnostic_publish_rate_));
  ROS_INFO_STREAM("diagnostic publish rate: " << diagnostic_publish_rate_);


  //add joints
  XmlRpc::XmlRpcValue joints;
  nh_private_.getParam("joints", joints);
  if(joints.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_FATAL("invalid YAML structure");
    ROS_BREAK();
  }

  for(XmlRpc::XmlRpcValue::iterator it = joints.begin(); it != joints.end(); it++)
  {
    string name = it->first;
    string type;
    if(!nh_private_.getParam("joints/" + name + "/type", type))
    {
      ROS_FATAL_STREAM(name + " has no type information");
      ROS_BREAK();
    }

    try
    {
      boost::shared_ptr<hg::Joint> joint = joint_plugin_loader_.createInstance("joint_plugins/" + type);
      joint->initilize(this, name);
      joints_.push_back(joint);
      ROS_INFO_STREAM("added joint " + name + " to " + nh_private_.getNamespace());
      pub_joint_state_.msg_.name.push_back(joint->name_);

    }
    catch (pluginlib::PluginlibException& e)
    {
      ROS_FATAL("The %s plugin failed to load for some reason. Error: %s",
                type.c_str(), e.what());
      ROS_BREAK();
    }
  }
  pub_joint_state_.msg_.position.resize(joints_.size());
  pub_joint_state_.msg_.velocity.resize(joints_.size());
  pub_joint_state_.msg_.effort.resize(joints_.size());

  //add controllers
  XmlRpc::XmlRpcValue controllers;
  nh_private_.getParam("controllers", controllers);
  ROS_ASSERT(controllers.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for(XmlRpc::XmlRpcValue::iterator it = controllers.begin(); it != controllers.end(); it++)
  {
    string name = it->first;
    string type;
    if(!nh_private_.getParam("controllers/" + name + "/type", type))
    {
      ROS_FATAL_STREAM(name + " has no type information");
      ROS_BREAK();
    }

    try
    {
      boost::shared_ptr<hg::Controller> controller = controller_plugin_loader_.createInstance("controller_plugins/" + type);
      controller->initilize(this, name);
      controllers_.push_back(controller);
      ROS_INFO_STREAM("added controller " + name + " to " + nh_private_.getNamespace());
    }
    catch (pluginlib::PluginlibException& e)
    {
      ROS_FATAL("The %s plugin failed to load for some reason. Error: %s", type.c_str(), e.what());
      ROS_BREAK();
    }
  }



  //setup publisher
  //publisher_joint_state_ = nh_private_.advertise<sensor_msgs::JointState>("/joint_states", 1);
  //publisher_diagnostic_ = nh_private_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);


}

ControllerNode::~ControllerNode()
{

}

static const int NSEC_PER_SECOND = 1e+9;
static const int USEC_PER_SECOND = 1e6;
static inline double now()
{
  struct timespec n;
  clock_gettime(CLOCK_MONOTONIC, &n);
  return double(n.tv_nsec) / NSEC_PER_SECOND + n.tv_sec;
}

static void timespecInc(struct timespec &tick, int nsec)
{
  tick.tv_nsec += nsec;
  while (tick.tv_nsec >= NSEC_PER_SECOND)
  {
    tick.tv_nsec -= NSEC_PER_SECOND;
    tick.tv_sec++;
  }
}

void ControllerNode::run()
{

#if REAL_TIME
  //set priority
  int retcode;
  int policy;
  pthread_t thread_id = (pthread_t)g_control_thread.native_handle();
  struct sched_param param;

  if ((retcode = pthread_getschedparam(thread_id, &policy, &param)) != 0)
  {
    errno = retcode;
    perror("pthread_getschedparam");
    exit(EXIT_FAILURE);
  }

  ROS_INFO_STREAM(
      "Control thread inherited: policy= " << ((policy == SCHED_FIFO) ? "SCHED_FIFO" : (policy == SCHED_RR) ? "SCHED_RR" : (policy == SCHED_OTHER) ? "SCHED_OTHER" : "???") << ", priority=" << param.sched_priority);

  policy = SCHED_FIFO;
  param.sched_priority = sched_get_priority_max(policy);

  if ((retcode = pthread_setschedparam(thread_id, policy, &param)) != 0)
  {
    errno = retcode;
    ROS_ERROR("pthread_setschedparam");
    return;
  }

  ros::Duration(1.0).sleep();

  if ((retcode = pthread_getschedparam(thread_id, &policy, &param)) != 0)
  {
    errno = retcode;
    ROS_ERROR("pthread_getschedparam");
    return;
  }

  ROS_INFO_STREAM(
      "Control thread changed: policy= " << ((policy == SCHED_FIFO) ? "SCHED_FIFO" : (policy == SCHED_RR) ? "SCHED_RR" : (policy == SCHED_OTHER) ? "SCHED_OTHER" : "???") << ", priority=" << param.sched_priority);
#endif

  //start all controllers
  std::vector<boost::shared_ptr<hg::Controller> >::iterator it;
  for(it = controllers_.begin(); it != controllers_.end(); it++)
  {
    (*it)->startup();
  }

  struct timespec tick;
  clock_gettime(CLOCK_REALTIME, &tick);
  int period = 1e+6; // 1 ms in nanoseconds

  // Snap to the nearest second
  tick.tv_sec = tick.tv_sec;
  tick.tv_nsec = (tick.tv_nsec / period + 1) * period;
  clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);




  ROS_INFO("Start");
  int count = 0;
  int publish_count = 0;
  while (!g_quit)
  {
    //update all controllers
    for(it = controllers_.begin(); it != controllers_.end(); it++)
    {
      (*it)->update();
    }

    //publish message
    publish_count++;
    if(publish_count >= 10)
    {
      if(pub_joint_state_.trylock())
      {
        pub_joint_state_.msg_.header.stamp = ros::Time::now();
        size_t i = 0;
        std::vector<boost::shared_ptr<hg::Joint> >::iterator joint_it;
        for(joint_it = joints_.begin(); joint_it != joints_.end(); joint_it++, i++)
        {
          pub_joint_state_.msg_.position[i] = (*joint_it)->position_;
          pub_joint_state_.msg_.velocity[i] = (*joint_it)->velocity_;
          pub_joint_state_.msg_.effort[i] = 0.0;
        }
        pub_joint_state_.unlockAndPublish();
        publish_count = 0;
      }
    }

    timespecInc(tick, period);
    struct timespec before;
    clock_gettime(CLOCK_REALTIME, &before);
    if ((before.tv_sec + double(before.tv_nsec) / NSEC_PER_SECOND)
        > (tick.tv_sec + double(tick.tv_nsec) / NSEC_PER_SECOND))
    {
      //ROS_ERROR("Overrun");
      printf("overrun");
      // We overran, snap to next "period"
      tick.tv_sec = before.tv_sec;
      tick.tv_nsec = (before.tv_nsec / period) * period;
      timespecInc(tick, period);
    }

    // Sleep until end of period
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
  }

  printf("exit\n");

  //stop all controllers
  for(it = controllers_.begin(); it != controllers_.end(); it++)
  {
    (*it)->shutdown();
  }

  printf("shutdown\n");

  ros::shutdown();

}

void ControllerNode::publish()
{

}


int main(int argc, char** argv)
{
#if REAL_TIME
  // Keep the kernel from swapping us out
  if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
    perror("mlockall");
    return -1;
  }
#endif

  ros::init(argc, argv, "hgROS");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);



  hg::ControllerNode node(nh, nh_private);

  g_control_thread = boost::thread(&ControllerNode::run, &node);

  ros::spin();

  g_control_thread.join();

  return 0;
}




