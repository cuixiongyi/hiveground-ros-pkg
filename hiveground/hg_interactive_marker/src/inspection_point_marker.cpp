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

#include <interactive_markers/interactive_marker_server.h>
#include <hg_interactive_marker/inspection_point.h>





int main(int argc, char** argv)
{
  ros::init(argc, argv, "inspection_point_marker");

  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("inspection_point_marker");

  hg_interactive_marker::InspectionPoint ip(server, "test", "/base_link", "haha");
  hg_interactive_marker::InspectionPoint ip2(server, "test1", "/base_link", "haha");
  hg_interactive_marker::InspectionPoint ip3(server, "test2", "/base_link", "haha");
  hg_interactive_marker::InspectionPoint ip4(server, "test3", "/base_link", "haha");
  hg_interactive_marker::InspectionPoint ip5(server, "test4", "/base_link", "haha");


  // 'commit' changes and send to all clients
  server.applyChanges();

  // start the ROS main loop
  ros::spin();
}
 
