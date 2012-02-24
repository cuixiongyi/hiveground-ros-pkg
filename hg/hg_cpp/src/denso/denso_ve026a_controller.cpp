#include <hg_cpp/denso/denso_ve026a_controller.h>



using namespace std;
using namespace hg;

#define PI	3.141592
#define DEG_TO_RAD (PI/180.0)
#define RAD_TO_DEG (180.0f/PI)

DensoVe026a_BCapController::DensoVe026a_BCapController(hg::HgROS* hg_ros, const std::string& name)
		: Controller(hg_ros, name),
		  is_initialized_(false),
		  listener_(hg_ros->node_handle_),
		  move_arm_action_server_("move_arm", false)
{
	string port_param_name = "controllers/" + name_ + "/port";
	cout << port_param_name << endl;
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
		joints_.insert(*(hg_ros->joints_.find(joint_name)));
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
	move_arm_action_server_.registerGoalCallback(
			boost::bind(&DensoVe026a_BCapController::move_arm_action_callback, this));

	/*
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
	move_arm_action_server_.start();


	//create control thread
	control_thread_ = boost::thread(&DensoVe026a_BCapController::control_loop, this);

	//start control loop
	is_running_ = true;
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

void DensoVe026a_BCapController::move_arm_action_callback()
{
	if(!get_ik_solver_info_client_.call(arm_solver_info_))
	{
		ROS_ERROR_STREAM(name_ + " cannot get_ik_solver_info");
		return;
	}
	hg_msgs::MoveArmGoalConstPtr goal = move_arm_action_server_.acceptNewGoal();
	if(move_arm_action_server_.isPreemptRequested())
	{
		ROS_INFO("move_arm_action preempted");
		move_arm_action_server_.setPreempted();
		return;
	}

	for(int i = 0; i < goal->motions.size(); i++)
	{
		hg_msgs::ArmAction action = goal->motions[i];

		if(action.type == hg_msgs::ArmAction::MOVE_ARM)
		{
			ROS_INFO("haha");
			//change motion to tarjectory
			motion_to_trajectory(action, goal->header);
		}
		else
		{

		}

	}

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
				ROS_ERROR_STREAM(name_ + " cannot get_ik_solver_info");
			}
			else
			{

				for(int i = 0; i < respond.solution.joint_state.position.size(); i++)
				{
					cout << respond.solution.joint_state.position[i] << endl;
				}
			}
		}
	}
	else
	{
		ROS_INFO("get IK request failed!");
	}


	//trajectory_msgs::JointTrajectory message;
	return message;
}

void DensoVe026a_BCapController::action_callback()
{
	//action_goal_ = action_server_->acceptNewGoal();
}

void DensoVe026a_BCapController::command_callback(const trajectory_msgs::JointTrajectory& message)
{
}

void DensoVe026a_BCapController::execute_trajectory(const trajectory_msgs::JointTrajectory& message)
{

}

void DensoVe026a_BCapController::control_loop()
{
	ros::Rate loop_rate(control_rate_);
	float command[7], result[8];
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

