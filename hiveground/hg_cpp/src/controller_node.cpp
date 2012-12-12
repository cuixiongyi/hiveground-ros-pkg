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
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <hg_cpp/controller_node.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sensor_msgs/JointState.h>


using namespace hg;
using namespace std;

ControllerNode::ControllerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
    nh_(nh),
    nh_private_(nh_private),
    is_simulated_(true),
    loop_rate_(50.0),
    joint_publish_rate_(10.0),
    diagnostic_publish_rate_(10.0),
    controller_plugin_loader_("controller_interface", "hg::Controller"),
    joint_plugin_loader_("joint_interface", "hg::Joint")
{



  //note settings
  ROS_ASSERT(nh_private_.getParam("simulated", is_simulated_));
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
      boost::shared_ptr<hg::Joint> joint = joint_plugin_loader_.createInstance("hg_cpp/" + type);
      joint->initilize(this, name);
      joints_.push_back(joint);
      ROS_INFO_STREAM("added joint " + name + " to " + nh_private_.getNamespace());
    }
    catch (pluginlib::PluginlibException& e)
    {
      ROS_FATAL("The %s plugin failed to load for some reason. Error: %s",
                type.c_str(), e.what());
      ROS_BREAK();
    }
  }

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
      boost::shared_ptr<hg::Controller> controller = controller_plugin_loader_.createInstance("hg_cpp/" + type);
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
  publisher_joint_state_ = nh_private_.advertise<sensor_msgs::JointState>("/joint_states", 1);
  publisher_diagnostic_ = nh_private_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
}

ControllerNode::~ControllerNode()
{

}

void ControllerNode::run(sig_atomic_t volatile *is_shutdown)
{
  //start all controllers
  std::vector<boost::shared_ptr<hg::Controller> >::iterator it;
  for(it = controllers_.begin(); it != controllers_.end(); it++)
  {
    (*it)->startup();
  }


  ros::Rate loop_rate(loop_rate_);
  while (!(*is_shutdown))
  {
    //publish message
    publish();

    //update all controllers
    for(it = controllers_.begin(); it != controllers_.end(); it++)
    {
      (*it)->update();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  //stop all controllers
  for(it = controllers_.begin(); it != controllers_.end(); it++)
  {
    (*it)->shutdown();
  }
}

void ControllerNode::publish()
{
  if (ros::Time::now() > next_joint_publish_time_)
  {
    sensor_msgs::JointState message;
    message.header.stamp = ros::Time::now();
    std::vector<boost::shared_ptr<hg::Joint> >::iterator it;
    for(it = joints_.begin(); it != joints_.end(); it++)
    {
      message.name.push_back((*it)->name_);
      message.position.push_back((*it)->position_);
      message.velocity.push_back((*it)->velocity_);
    }
    publisher_joint_state_.publish(message);
    next_joint_publish_time_ = ros::Time::now() + ros::Duration(1.0 / joint_publish_rate_);
  }

  if (ros::Time::now() > next_diagnostic_publish_time_)
  {
    next_diagnostic_publish_time_ = ros::Time::now() + ros::Duration(1.0 / diagnostic_publish_rate_);
  }
}


/*
 * Override shutdown. Adopt from
 * http://answers.ros.org/question/27655/what-is-the-correct-way-to-do-stuff-before-a-node/
 */

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "HgRos", ros::init_options::NoSigintHandler);
  signal(SIGINT, mySigIntHandler);

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

  hg::ControllerNode node(nh, nh_private);
  node.run(&g_request_shutdown);

  ros::shutdown();

  return 0;
}




