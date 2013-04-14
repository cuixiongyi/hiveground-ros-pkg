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

#ifndef VP6242_ROBOT_H_
#define VP6242_ROBOT_H_


#include <hg_controller_manager/hg_joint_command_interface.h>
#include <hg_controller_manager/hg_robot_hardware.h>
#include <urdf/model.h>
#include <bcap/bcap_net.h>
#include <boost/thread.hpp>
#include <sensor_msgs/JointState.h>
#include <hg_realtime_tools/realtime_publisher.h>

namespace denso_common
{

class VP6242Robot : public hg_controller_manager::RobotHardware
{
public:
  VP6242Robot(const ros::NodeHandle& nh, const urdf::Model& urdf_model , const std::string& prefix="");
  virtual ~VP6242Robot();

  bool read();
  bool write();

  bool start();
  bool stop();

protected:


  //degree
  bool setGetPosition(std::vector<float>& position, std::vector<float>& result);

  //degree
  bool getJointPosition(std::vector<float>& position);


  bool setMotor(bool on);

  void publishJointState();

protected:
  ros::NodeHandle nh_;

  hg_controller_manager::JointStateInterface js_interface_;
  hg_controller_manager::PositionJointInterface pj_interface_;

  int joint_size_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_position_last_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<std::string> joint_name_;

  std::vector<double> joint_position_limit_upper_;
  std::vector<double> joint_position_limit_lower_;

  ros::Time last_update_;
  ros::Time last_published_joint_state_;
  std::vector<float> command_degree_;
  std::vector<float> result_degree_;

  urdf::Model urdf_;

  boost::shared_ptr<BCapNet> bcap_;
  std::string ip_;
  std::string port_;
  uint32_t h_controller_;
  uint32_t h_task_;
  uint32_t h_robot_;
  uint32_t h_joint_angle_variable_;
  bool motor_on_;
  bool is_started_;
  int slave_mode_;
  boost::mutex bcap_mutex_;

  hg_realtime_tools::RealtimePublisher<sensor_msgs::JointState> pub_joint_state_;
};


}



#endif /* VP6242_ROBOT_H_ */
