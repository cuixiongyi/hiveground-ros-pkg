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

#ifndef HG_RC7M_JOINT_H_
#define HG_RC7M_JOINT_H_

#include <pluginlib/class_list_macros.h>
#include <hg_cpp/hg_joint.h>

namespace hg_plugins
{

class RC7MJoint : public hg::Joint
{
public:
  /**
   * A default constructor.
   */
  RC7MJoint();

  /**
   * A destructor.
   */
  ~RC7MJoint();

  /**
   * An initializing function.
   */
  void initilize(hg::Node* node, const std::string& name);

  /**
   * Load joint information from URDF.
   */
  bool get_joint_info_urdf();
  /**
   * Interpolate joint position after dt.
   */
  double interpolate(double dt);

  /**
   * Set feedback data from sensor (encoder, camera, ...).
   */
  void set_feedback_data(double feedback);

  /**
   * Set joint position.
   */
  double set_position(double position);

  /**
   * Get a diagnostics message for this joint.
   */
  diagnostic_msgs::DiagnosticStatus get_diagnostics();
};

}



#endif