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

#ifndef HG_VE026A_CONTROLLER_H_
#define HG_VE026A_CONTROLLER_H_

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <boost/thread.hpp>
#include <queue>

#include <hg_cpp/hg_controller.h>
#include <bcap/bcap_serial.h>
#include <ve026a_controller/ve026a_joint.h>

namespace hg_plugins
{

class VE062AController : public hg::Controller
{

public:
  /**
   * A default constructor.
   */
  VE062AController();

  /**
   * A destructor.
   *
   */
  ~VE062AController();

  /**
   * An initializing function.
   */
  void initilize(hg::Node* node, const std::string& name);

  /**
   * Start the controller, do any hardware setup needed.
   */
  void startup();

  /**
   * Do any read/writes to device.
   */
  void update();

  /**
   * Control loop of RC7M.
   * Execute in separated thread.
   */
  void control();

  /**
   * Stop the controller, do any hardware shutdown needed.
   */
  void shutdown();

  /**
   * Is the controller actively sending commands to joints/robots?
   */
  bool active();

  bool setJointsSpeed(double speed);

private:
  /**
   * Turn motor on/off.
   */
  BCAP_HRESULT setMotor(bool on_off);

  /**
   * Get feedback from robot and update joints.
   * @
   */
  BCAP_HRESULT getJointFeedback(bool set_desired_position_ = false);

  //action server
  void followJointGoalActionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);

public:
  BCapSerial bcap_;
  std::string port_;
  uint32_t hController_;
  uint32_t hTask_;
  uint32_t hRobot_;
  uint32_t hPositionVariable;
  uint32_t hAngleVariable;
  bool motor_on_;
  bool slave_mode_on_;
  bool is_preempted_;
  int preempted_point_;
  int new_start_point_;
  trajectory_msgs::JointTrajectory last_trajectory_;


  bool is_running_;
  boost::thread control_thread_;
  boost::mutex control_mutex_;

  typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> FollowJointTrajectoryActionServer;
  typedef boost::shared_ptr<FollowJointTrajectoryActionServer> FollowJointTrajectoryActionServerPtr;
  FollowJointTrajectoryActionServerPtr action_server_;
  control_msgs::FollowJointTrajectoryGoalConstPtr action_goal_;

  boost::mutex queue_mutex_;
  typedef std::queue<trajectory_msgs::JointTrajectory> JointTrajectoryQueue;
  JointTrajectoryQueue joint_trajecgtory_queue_;
  trajectory_msgs::JointTrajectory current_joint_trajecgtory_;
  bool process_trajectory_;



};

}//hg_plugins

#endif



#if 0
#ifndef _hg_denso_ve026a_controller_h_
#define _hg_denso_ve026a_controller_h_

#include <hg_cpp/controller.h>
#include <hg_cpp/joint.h>
#include <hg_cpp/denso/denso_bcap_serial.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <sensor_msgs/JointState.h>
#include <actionlib_msgs/GoalStatus.h>
#include <kinematics_msgs/KinematicSolverInfo.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <std_msgs/Bool.h>

#include <hg_msgs/MoveArmAction.h>

#include <boost/thread.hpp>
#include <queue>

namespace hg {

class HgROS;

class DensoVe026a_BCapController: public Controller {
public:
	DensoVe026a_BCapController(hg::HgROS* hg_ros, const std::string& name);
	~DensoVe026a_BCapController();

	void startup();
	void update();
	void shutdown();
	bool active();

	//void move_arm_action_callback();

	//trajectory_msgs::JointTrajectory motion_to_trajectory(
	//hg_msgs::ArmAction action, std_msgs::Header header);

	void action_callback();

	void command_callback(const trajectory_msgs::JointTrajectory& message);

	void execute_trajectory();

private:
	void initialize_ve026a();
	bool set_motor(bool on_off);
	void set_motor_callback(const std_msgs::BoolConstPtr& flag);

	/**
	 * Control thread function.
	 */
	void control_loop();

	/**
	 * Set joints position of VE026a.
	 * @param joint positions in degree (float [7])
	 * @param current joint positions in degree (float [8])
	 */
	bool set_joints(float* positions, float* results);
public:

	bool is_initialized_;
	uint32_t b_cap_controller_;
	uint32_t b_cap_robot_;

	std::string port_name_;
	int baud_rate_;
	int data_bits_;
	int stop_bits_;
	int parity_;

	BCapSerial bcap_serial_;

	double control_rate_;
	bool is_running_;
	bool is_motor_on_;
	boost::thread control_thread_;
	boost::mutex mutex_control_;
	boost::thread follow_control_thread_;
	std::queue<trajectory_msgs::JointTrajectory> trajectory_queue_; // Use STL queue to store data
	boost::mutex mutex_; // The mutex to synchronise on
	boost::condition_variable condition_;

	bool trajectory_is_executing_;
	bool trajectory_is_ok_;
	std::string action_name_;

	tf::TransformListener listener_;

	//ros::ServiceClient get_ik_client_;
	//ros::ServiceClient get_ik_solver_info_client_;
	//kinematics_msgs::GetKinematicSolverInfo arm_solver_info_;

	//actionlib::SimpleActionServer<hg_msgs::MoveArmAction> move_arm_action_server_;

	boost::shared_ptr<actionlib::SimpleActionServer<
			control_msgs::FollowJointTrajectoryAction> > action_server_;
	control_msgs::FollowJointTrajectoryGoalConstPtr action_goal_;

	ros::Subscriber trajectory_command_;
	ros::Subscriber motor_on_off_command_;

};

}

#endif
#endif

