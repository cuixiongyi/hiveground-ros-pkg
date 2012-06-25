/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Mahisorn Wongphati
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
 */

#include <hg_cpp/hg_node.h>

using namespace hg;
using namespace std;

Node::Node() :
    node_handle_("~"),
    simulate_(true),
    loop_rate_(50.0)
{
  //simulate ?
  node_handle_.getParam("simulate", simulate_);
  if(simulate_)
    ROS_INFO_STREAM("start " + node_handle_.getNamespace() + " node in simulated mode");
  else
    ROS_INFO_STREAM("start " + node_handle_.getNamespace() + " node in real mode");



  //add joints
  XmlRpc::XmlRpcValue joints;
  node_handle_.getParam("joints", joints);
  if(joints.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_FATAL("invalid YAML structure");
    ROS_BREAK();
  }

  for(XmlRpc::XmlRpcValue::iterator it = joints.begin(); it != joints.end(); it++)
  {
    string name = it->first;
    string type;
    if(!node_handle_.getParam("joints/" + name + "/type", type))
    {
      ROS_FATAL_STREAM(name + "has no type information");
      ROS_BREAK();
    }

    if(type == "built-in")
    {
      ROS_INFO_STREAM("added joint : " + name);
    }
    else
    {
      ROS_FATAL_STREAM("joint with " + type + " is not support");
    }



  }



  //add controllers
  XmlRpc::XmlRpcValue controllers;
  node_handle_.getParam("controllers", controllers);
  ROS_ASSERT(controllers.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for(XmlRpc::XmlRpcValue::iterator it = joints.begin(); it != joints.end(); it++)
  {

  }
}

Node::~Node()
{

}

void Node::run()
{
  ros::Rate loop_rate(loop_rate_);
  while (node_handle_.ok())
  {
    //publish message
    publish();

    //execute all controllers and joints

    ros::spinOnce();
    loop_rate.sleep();
  }

}

void Node::publish()
{
  ROS_INFO_STREAM_THROTTLE(1.0, "hello");
}
