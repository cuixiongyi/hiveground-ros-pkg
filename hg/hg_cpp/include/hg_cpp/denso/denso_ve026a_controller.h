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
#include <hg_msgs/MoveArmAction.h>

#include <boost/thread.hpp>
#include <queue>

namespace hg
{

class HgROS;

class DensoVe026a_BCapController : public Controller
{
public:
	DensoVe026a_BCapController(hg::HgROS* hg_ros, const std::string& name);
	~DensoVe026a_BCapController();

	void startup();
	void update();
	void shutdown();
	bool active();

	void move_arm_action_callback();

	trajectory_msgs::JointTrajectory motion_to_trajectory(
			hg_msgs::ArmAction action, std_msgs::Header header);

	void action_callback();

	void command_callback(const trajectory_msgs::JointTrajectory& message);

	void execute_trajectory(const trajectory_msgs::JointTrajectory& message);
	void execute_trajectory2();


private:
	void initialize_ve026a();
	bool set_motor(bool on_off);

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
	boost::thread control_thread_;
	boost::thread follow_control_thread_;
	std::queue<trajectory_msgs::JointTrajectory> trajectory_queue_;				// Use STL queue to store data
	boost::mutex mutex_;							// The mutex to synchronise on
	boost::condition_variable condition_;

	bool trajectory_is_executing_;
	bool trajectory_is_ok_;
	std::string action_name_;

	tf::TransformListener listener_;

	ros::ServiceClient get_ik_client_;
	ros::ServiceClient get_ik_solver_info_client_;
	kinematics_msgs::GetKinematicSolverInfo arm_solver_info_;

	actionlib::SimpleActionServer<hg_msgs::MoveArmAction> move_arm_action_server_;

	boost::shared_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> > action_server_;
	control_msgs::FollowJointTrajectoryGoalConstPtr action_goal_;

	ros::Subscriber trajectory_command_;

};


}



#endif

