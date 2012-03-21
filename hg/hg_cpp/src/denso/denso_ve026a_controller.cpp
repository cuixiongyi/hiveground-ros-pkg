#include <hg_cpp/denso/denso_ve026a_controller.h>
#include <boost/any.hpp>


using namespace std;
using namespace hg;

#define PI	3.141592
#define DEG_TO_RAD (PI/180.0)
#define RAD_TO_DEG (180.0f/PI)

DensoVe026a_BCapController::DensoVe026a_BCapController(hg::HgROS* hg_ros, const std::string& name)
		: Controller(hg_ros, name),
		  is_initialized_(false),
		  listener_(hg_ros->node_handle_)
		  //move_arm_action_server_("move_arm", false)
{
	string port_param_name = "controllers/" + name_ + "/port";
	//cout << port_param_name << endl;
	if(!node_handle_.hasParam(port_param_name))
	{
		ROS_ERROR_STREAM(name_ + ": couldn't find " + port_param_name +
				"please check if ALL parameters have been set correctly");
	}
	node_handle_.param<std::string>(
			port_param_name, port_name_, "/dev/ttyUSB0");
	ROS_INFO_STREAM(name_ + ": serial port: " + port_name_);

	//control rate
	node_handle_.param<double>(
			"controllers/" + name_ + "/control_rate", control_rate_, 100);
	ROS_INFO_STREAM(name_ + ": control_rate: " << control_rate_);


	//add joints
	XmlRpc::XmlRpcValue joints;
	node_handle_.getParam("controllers/" + name_ + "/joints", joints);
	ROS_ASSERT(joints.getType() == XmlRpc::XmlRpcValue::TypeArray);
	for(int i = 0; i < joints.size(); i++)
	{
		std::string joint_name;
		ROS_ASSERT(joints[i].getType() == XmlRpc::XmlRpcValue::TypeString);
		joint_name = static_cast<std::string>(joints[i]);
		JointMap::iterator itr = hg_ros->joints_.find(joint_name);
		if(itr != hg_ros->joints_.end())
		{
			joints_.insert(*itr);
			ROS_INFO_STREAM(name_ << " added joint: " << joint_name);
		}
		else
		{
			ROS_WARN_STREAM(name_ << " joint: " << joint_name << " not found");
		}
	}

	//set joint controller
	for(JointMap::iterator itr = joints_.begin(); itr != joints_.end(); itr++)
	{
		itr->second->controller_ = this;
	}

	ROS_INFO("Waiting for arm_kinematics services...");
	ros::service::waitForService("/arm_kinematics/get_ik");
	ros::service::waitForService("/arm_kinematics/get_ik_solver_info");

	get_ik_client_ = node_handle_.serviceClient<kinematics_msgs::GetPositionIK>
			("/arm_kinematics/get_ik");
	get_ik_solver_info_client_ =
			node_handle_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>
				("/arm_kinematics/get_ik_solver_info");



	ROS_INFO("arm_kinematics services connected");
	//move arm action server
	//move_arm_action_server_.registerGoalCallback(
		//	boost::bind(&DensoVe026a_BCapController::move_arm_action_callback, this, ));


	//action server
	//get name
	trajectory_is_executing_ = false;
	node_handle_.getParam("controllers/" + name_ + "/action_name", action_name_);
	cout << action_name_ << endl;

	//create server
	action_server_  = boost::shared_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> >
	(new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(
			node_handle_,
			action_name_,
			false));

	//set callback
	action_server_->registerGoalCallback(boost::bind(&DensoVe026a_BCapController::action_callback, this));
	/*
	//normal tarjectory command
	tarjectory_command_ = node_handle_.subscribe(name_ + "/command", 1,
			&DensoVe026a_BCapController::command_callback, this);
	*/


	//ROS_INFO_STREAM("Start " + name_ + )
}

DensoVe026a_BCapController::~DensoVe026a_BCapController()
{

}

void DensoVe026a_BCapController::startup()
{
	if(!hg_ros_->simulate_)
	{
		int error = bcap_serial_.Open(port_name_);
		if(error) return;
		error = bcap_serial_.Initialise(115200, 8, 1, SerialPort::PARITY_NONE);
		if(error) return;

		ROS_INFO_STREAM(name_ + ": serial port connected");
	}

	initialize_ve026a();

	if(!is_initialized_)
		return;


	//start action server
	//move_arm_action_server_.start();
	action_server_->start();


	//create control thread
	control_thread_ = boost::thread(&DensoVe026a_BCapController::control_loop, this);
	follow_control_thread_ = boost::thread(&DensoVe026a_BCapController::execute_trajectory, this);

	//start control loop
	is_running_ = true;


	ROS_INFO_STREAM(name_ << " started");
}

void DensoVe026a_BCapController::update()
{

}

void DensoVe026a_BCapController::shutdown()
{
	set_motor(false);
	is_running_ = false;
	control_thread_.join();
}

bool DensoVe026a_BCapController::active()
{
	return false;
}
#if 0
void DensoVe026a_BCapController::move_arm_action_callback()
{
	if(!get_ik_solver_info_client_.call(arm_solver_info_))
	{
		ROS_ERROR_STREAM(name_ + " cannot get_ik_solver_info");
		return;
	}
	hg_msgs::MoveArmGoalConstPtr goal = move_arm_action_server_.acceptNewGoal();

	hg_msgs::MoveArmResult result;
	if(move_arm_action_server_.isPreemptRequested())
	{
		ROS_INFO("move_arm_action preempted");
		result.success = false;
		move_arm_action_server_.setPreempted(result);
		return;
	}

	//std::vector<std::pa>
	std::vector<std::pair<std::string, boost::any> > computed_actions;
	for(int i = 0; i < goal->motions.size(); i++)
	{
		hg_msgs::ArmAction action = goal->motions[i];

		if(action.type == hg_msgs::ArmAction::MOVE_ARM)
		{
			ROS_INFO("move arm");
			//change motion to tarjectory
			trajectory_msgs::JointTrajectory message =
					motion_to_trajectory(action, goal->header);

			if(message.points.size() == 0)
			{
				ROS_INFO("Trajectory empty");
				result.success = false;
				move_arm_action_server_.setAborted(result);
				return;
			}
			//follow_control_thread_ =
				//boost::thread(&DensoVe026a_BCapController::execute_trajectory, this, message );
			//execute_trajectory(message);

			std::pair<std::string, boost::any> move("move", message);
			computed_actions.push_back(move);
		}
		else
		{
			ROS_INFO("move gripper");
			std::pair<std::string, boost::any> gripper("gripper", action);
			computed_actions.push_back(gripper);
			//joints_.at("J7")->set_position(action.command);
		}//if action
	}//for motion

	//execute trajectory


	ROS_INFO("computed action: %d", computed_actions.size());
	bool first_move = true;
	trajectory_msgs::JointTrajectory trajectory;
	for(int i = 0; i < computed_actions.size(); i++)
	{
		if(computed_actions[i].first == "move")
		{
			cout << "move arm: "
				 << (boost::any_cast<trajectory_msgs::JointTrajectory>(&(computed_actions[i].second)))->header.stamp.toSec()
				 << endl;
			if(first_move)
			{
				trajectory = *(boost::any_cast<trajectory_msgs::JointTrajectory>(&(computed_actions[i].second)));
				first_move = false;

			}
			else
			{
				trajectory_msgs::JointTrajectory message = *(boost::any_cast<trajectory_msgs::JointTrajectory>(&(computed_actions[i].second)));
				message.points[0].time_from_start +=
						trajectory.points[trajectory.points.size() - 1].time_from_start;
				trajectory.points.push_back(message.points[0]);
			}
		}
		else if(computed_actions[i].first == "gripper")
		{
			if(trajectory.points.size() > 0)
			{
				//move arm before gripper
				trajectory_is_ok_ = false;
				follow_control_thread_ =
					boost::thread(&DensoVe026a_BCapController::execute_trajectory, this, trajectory);
				follow_control_thread_.join();
				if(!trajectory_is_ok_)
				{
					ROS_INFO("execute_trajectory fail");
					return;
				}
				first_move = true;
			}
			//move gripper
			joints_.at("J7")->set_position((boost::any_cast<hg_msgs::ArmAction>(&(computed_actions[i].second)))->command);
			ros::Duration((boost::any_cast<hg_msgs::ArmAction>(&(computed_actions[i].second)))->move_time).sleep();
			cout << "move gripper: "
				 << (boost::any_cast<hg_msgs::ArmAction>(&(computed_actions[i].second)))->command
				 << endl;
		}
	}

	boost::unique_lock<boost::mutex> lock(mutex_);
	trajectory_queue_.push(trajectory);
	condition_.notify_one();
}

trajectory_msgs::JointTrajectory DensoVe026a_BCapController::motion_to_trajectory(
		hg_msgs::ArmAction action, std_msgs::Header header)
{

	ROS_INFO("1");
	geometry_msgs::PoseStamped ps;
	ps.header.frame_id = header.frame_id;
	ps.pose = action.goal;
	geometry_msgs::PoseStamped pose;
	listener_.transformPose("base_link", ps, pose);
	ROS_INFO("2");


	kinematics_msgs::GetPositionIK::Request request;
	request.timeout = ros::Duration(5.0);
	request.ik_request.pose_stamped.header.frame_id = "base_link";
	request.ik_request.ik_link_name = "link5";
	request.ik_request.pose_stamped.pose = pose.pose;

	ROS_INFO("request: %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f",
			pose.pose.position.x,
			pose.pose.position.y,
			pose.pose.position.z,
			pose.pose.orientation.x,
			pose.pose.orientation.y,
			pose.pose.orientation.z,
			pose.pose.orientation.w);
	//pose.pose.orientation.

	request.ik_request.ik_seed_state.joint_state.name =
			arm_solver_info_.response.kinematic_solver_info.joint_names;
	request.ik_request.ik_seed_state.joint_state.position.resize(
			request.ik_request.ik_seed_state.joint_state.name.size());
	for(int i = 0; i < request.ik_request.ik_seed_state.joint_state.name.size(); i++)
	{
		string name = request.ik_request.ik_seed_state.joint_state.name[i];
		JointMap::iterator itr = joints_.find(name);
		cout << name << " " << itr->second->position_ << endl;
		request.ik_request.ik_seed_state.joint_state.position[i] = itr->second->position_;

	}
	ROS_INFO("3");
	//request.ik_request.ik_seed_state.joint_state.position.

	//tires = 0;
	kinematics_msgs::GetPositionIK::Response respond;
	trajectory_msgs::JointTrajectory message;
	if(get_ik_client_.call(request, respond))
	{
		ROS_INFO("get IK respond!");
		cout << respond.error_code << endl;
		if(respond.error_code.val == respond.error_code.SUCCESS)
		{
			ROS_INFO("Hurey!!" );
			if(!get_ik_solver_info_client_.call(arm_solver_info_))
			{
				ROS_ERROR_STREAM(name_ + " cannot get_ik_solver_info!");
			}
			else
			{
				message.joint_names =
						arm_solver_info_.response.kinematic_solver_info.joint_names;
				trajectory_msgs::JointTrajectoryPoint point;
				//point.positions =

				//message.points
				point.positions.resize(respond.solution.joint_state.position.size());
				point.velocities.resize(respond.solution.joint_state.position.size());
				for(int i = 0; i < respond.solution.joint_state.position.size(); i++)
				{
					cout << "Joint " << message.joint_names[i] << " ";
					cout << respond.solution.joint_state.position[i] << endl;

					point.positions[i] = respond.solution.joint_state.position[i];
					point.velocities[i] = 0.0;

				}

				cout << "Move time: " << action.move_time.toSec() << endl;
				if(action.move_time > ros::Duration(0.0))
				{
					//move according to a specified time
					point.time_from_start = action.move_time;
				}
				else
				{
					//move according to some specified time
					point.time_from_start = ros::Duration(5.0);
				}

				message.points.push_back(point);
				message.header.stamp = ros::Time::now() + ros::Duration(0.01);


			}
		}
	}
	else
	{
		ROS_INFO("get IK request failed!");
	}

	return message;
}
#endif

void DensoVe026a_BCapController::action_callback()
{
	action_goal_ = action_server_->acceptNewGoal();
	ROS_INFO_STREAM(name_ + ": Action goal received");

	trajectory_msgs::JointTrajectory trajectory = action_goal_->trajectory;

	control_msgs::FollowJointTrajectoryResult result;

	//check joint names
	for(size_t i = 0; i < trajectory.joint_names.size(); i++)
	{
		JointMap::iterator itr = joints_.find(trajectory.joint_names[i]);
		if(itr == joints_.end())
		{
			result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
			action_server_->setAborted(result, "Trajectory joint names does not match action controlled joints");
			return;
		}
	}


	//check points
	if(trajectory.points.size() == 0)
	{
		result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL ;
		action_server_->setAborted(result, "Trajectory empty");
		return;
	}


	ROS_INFO_STREAM("Trajectort points: " << trajectory.points.size());
	for(int i = 0; i < trajectory.points.size(); i++)
	{
		trajectory_msgs::JointTrajectoryPoint point =  trajectory.points[i];
		std::vector<double> target_positions = point.positions;
		ros::Time end_time = ros::Time::now() + point.time_from_start;
		//ROS_INFO("Trajectory[%d]: end-time: %6.3f", i, end_time.toSec());
	}

	boost::unique_lock<boost::mutex> lock(mutex_);
	trajectory_queue_.push(trajectory);
	condition_.notify_one();

	//action_server_->setAborted(result, "Test");
	//execute_trajectory(trajectory);
}

void DensoVe026a_BCapController::command_callback(const trajectory_msgs::JointTrajectory& message)
{
}

void DensoVe026a_BCapController::execute_trajectory()
{
	control_msgs::FollowJointTrajectoryResult result;
	while(is_running_)
	{

		// Acquire lock on the queue
		boost::unique_lock<boost::mutex> lock(mutex_);

		// When there is no data, wait till someone fills it.
		// Lock is automatically released in the wait and obtained
		// again after the wait
		while (trajectory_queue_.size()==0) condition_.wait(lock);

		ROS_INFO("Got a new trajectory!!!");
		// Retrieve the data from the queue
		trajectory_msgs::JointTrajectory trajectory = trajectory_queue_.front();
		trajectory_queue_.pop();


		ros::Time time = ros::Time::now();
		ros::Time start_time = trajectory.header.stamp;

		//ROS_INFO("%6.3f %6.3f", time.toSec(), start_time.toSec());

		std::vector<double> last_positions;
		for(JointMap::iterator itr = joints_.begin(); itr != joints_.end(); itr++)
		{
			last_positions.push_back(itr->second->position_);
			cout << "last position of "
				 << itr->first << ":"
				 << itr->second->position_
				 << endl;

		}

		for(int i = 0; i < trajectory.points.size(); i++)
		{
			while((ros::Time::now() + ros::Duration(0.01)) < start_time)
			{
				//wait for time
				ros::Duration(0.01).sleep();
			}


			trajectory_msgs::JointTrajectoryPoint point =  trajectory.points[i];
			std::vector<double> target_positions = point.positions;
			//for(int j = 0; j < target_positions.size(); j++)
			//{
				//cout << "target joint[" << j << "]" <<  " position: " << target_positions[j] << endl;
			//}

			ros::Time end_time = start_time + point.time_from_start;
			//ROS_INFO("Trajectory[%d]: end-time: %6.3f", i, end_time.toSec());

			std::vector<double> error, velocity;
			error.resize(target_positions.size());
			velocity.resize(target_positions.size());
			double command;
			ros::Rate rate(50.0);
			while((ros::Time::now()+ros::Duration(0.01)) < end_time)
			{
				JointMap::iterator itr = joints_.begin();
				for(int k = 0; k < error.size(); k++, itr++)
				{
					error[k] = target_positions[k] - last_positions[k];
					velocity[k] =
							fabs(error[k] / (50.0 * (end_time - ros::Time::now()).toSec()));
					//ROS_INFO_THROTTLE(1, "v: %f", velocity[k]);

					if(fabs(error[k]) > 0.001)
					{
						command = error[k];
						if(command > velocity[k])
						{
							command = velocity[k];
						}
						else if(command > velocity[k])
						{
							command = -velocity[k];
						}
						last_positions[k] += command;
						//ROS_INFO_THROTTLE(1, "cmd: %f", last_positions[k]);
						itr->second->set_position(last_positions[k]);
					}
					else
					{
						//reached
						velocity[k] = 0;
					}

				}

				rate.sleep();
			}
		}
		ROS_INFO("Trajectory done!!!");
		result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
		action_server_->setSucceeded(result, "test!");
	}
}

void DensoVe026a_BCapController::control_loop()
{
	ros::Rate loop_rate(control_rate_);
	float command[7], result[8];
	memset(command, 0, sizeof(command));
	memset(result, 0, sizeof(result));
	int i;
	JointMap::iterator itr;
	while(is_running_)
	{
		//ROS_INFO_STREAM(name_ + ": set command");
		for(itr = joints_.begin(), i = 0; itr != joints_.end(); itr++, i++)
		{
			command[i] = itr->second->interpolate(1.0/control_rate_) * RAD_TO_DEG;
		}

		if(!hg_ros_->simulate_)
		{
			//update joint positions
			if(!set_joints(command, result))
			{
				return;
			}

			//ROS_INFO_STREAM(name_ + ": set feedback");
			for(itr = joints_.begin(), i = 0; itr != joints_.end(); itr++, i++)
			{
				itr->second->set_feedback_data(result[i] * DEG_TO_RAD);
			}
		}
		else
		{
			for(itr = joints_.begin(), i = 0; itr != joints_.end(); itr++, i++)
			{
				itr->second->set_feedback_data(command[i] * DEG_TO_RAD);
			}
		}
		loop_rate.sleep();
	}

	condition_.notify_all();
}

void DensoVe026a_BCapController::initialize_ve026a()
{
	if(hg_ros_->simulate_)
	{
		is_initialized_ = true;
		return;
	}

	if(!is_initialized_)
	{
		ROS_INFO_STREAM(name_ + ": initializing ...");
		BCAP_HRESULT hr;

		//connect controller
		hr = bcap_serial_.bCap_ControllerConnect(
			"b-CAP",
			"CaoProv.DENSO.VRC",
			"localhost",
			"@EventDisable=false,Wpj=*",
			b_cap_controller_);
		if(FAILED(hr))
		{
			ROS_ERROR_STREAM(name_ + ": controller connect fail");
			return;
		}
		else
		{
			ROS_INFO_STREAM(name_ + ": controller connected");
		}

		//get robot
		hr = bcap_serial_.bCap_ControllerGetRobot(
			b_cap_controller_,
			"VE026A",
			"$IsIDHandle$",
			&b_cap_robot_);
		if(FAILED(hr))
		{
			ROS_ERROR_STREAM(name_ + ": get robot fail");
			return;
		}
		else
		{
			ROS_INFO_STREAM(name_ + ": get robot successful");
		}

		//turn off motor
		if(!set_motor(false))
			return;

		//set slave mode
		int mode = 258;
		long result;
		hr = bcap_serial_.bCap_RobotExecute2(
			b_cap_robot_,
			"slvChangeMode",
			VT_I4,
			1,
			&mode,
			&result);
		if(FAILED(hr))
		{
			ROS_ERROR_STREAM(name_ + ": cannot change to slave mode");
			return;
		}
		else
		{
			ROS_INFO_STREAM(name_ + ": change robot slave mode successful");
		}


		float joints[8];
		float results[8];
		memset(joints, 0, sizeof(float)*8);
		memset(results, 0, sizeof(float)*8);

		//get joint positions
		if(!set_joints(joints, results))
			return;

		if(!set_joints(results, joints))
			return;

		//setup current position and desired position
		JointMap::iterator itr;
		int i;
		for(itr = joints_.begin(), i = 0; itr != joints_.end(); itr++, i++)
		{
			itr->second->set_feedback_data(joints[i] * DEG_TO_RAD);
			itr->second->set_position(joints[i] * DEG_TO_RAD);
		}

		//turn on motor
		if(!set_motor(true))
			return;

		ROS_INFO_STREAM(name_ + " is initialized");
		is_initialized_ = true;
	}
	else
		ROS_WARN_STREAM(name_ + " is already initialized");

}

bool DensoVe026a_BCapController::set_motor(bool on_off)
{
	if(hg_ros_->simulate_) return true;

	BCAP_HRESULT hr;
	int mode = (on_off) ? 1 : 0;
	long result;
	hr = bcap_serial_.bCap_RobotExecute2(
		b_cap_robot_,
		"Motor",
		VT_I2,
		1,
		&mode,
		&result);
	if(FAILED(hr))
	{
		ROS_ERROR_STREAM(name_ + ": set motor fail");
		return false;
	}
	else
	{
		if(on_off)
			ROS_INFO_STREAM(name_ + ": set motor on");
		else
			ROS_INFO_STREAM(name_ + ": set motor off");
		return true;
	}
}

bool DensoVe026a_BCapController::set_joints(float* position, float* result)
{
	if(hg_ros_->simulate_) return true;

	BCAP_HRESULT hr;
	hr = bcap_serial_.bCap_RobotExecute2(
		b_cap_robot_,
		"slvMove",
		VT_R4 | VT_ARRAY,
		7,
		position,
		result);
	if(FAILED(hr))
	{
		ROS_ERROR_STREAM(name_ + ": set joints fail");
		return false;
	}
	else
	{
		return true;
	}
}

