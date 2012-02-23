#include <hg_cpp/denso/denso_ve026a_controller.h>

using namespace std;
using namespace hg;

DensoVe026a_BCapController::DensoVe026a_BCapController(hg::HgROS* hg_ros, const std::string& name)
		: Controller(hg_ros, name),
		  is_initialized_(false)
{
	string port_param_name = "controllers/" + name_ + "/port";
	if(!node_handle_.hasParam(port_param_name))
	{
		ROS_ERROR_STREAM(name_ + "couldn't find " + port_param_name +
				"check if ALL parameters have been set correctly");
	}
	node_handle_.param<std::string>(
			port_param_name, port_name_, "/dev/ttyUSB0");
	ROS_INFO_STREAM(name_ + " serial port: " + port_name_);

	//action joint

	//action server

	//


	//ROS_INFO_STREAM("Start " + name_ + )
}

DensoVe026a_BCapController::~DensoVe026a_BCapController()
{

}

void DensoVe026a_BCapController::startup()
{
	int error = bcap_serial_.Open(port_name_);
	if(error) return;
	error = bcap_serial_.Initialise(115200, 8, 1, SerialPort::PARITY_NONE);
	if(error) return;

	ROS_INFO_STREAM(name_ + " serial port connected");

	initialize_ve026a();
}

void DensoVe026a_BCapController::update()
{

}

void DensoVe026a_BCapController::shutdown()
{
	set_motor(false);
}

bool DensoVe026a_BCapController::active()
{
	return false;
}

void DensoVe026a_BCapController::initialize_ve026a()
{
	if(!is_initialized_)
	{
		ROS_INFO_STREAM(name_ + " initializing ...");
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
			ROS_ERROR_STREAM(name_ + " controller connect fail");
			return;
		}
		else
		{
			ROS_INFO_STREAM(name_ +" controller connected");
		}

		//get robot
		hr = bcap_serial_.bCap_ControllerGetRobot(
			b_cap_controller_,
			"VE026A",
			"$IsIDHandle$",
			&b_cap_robot_);
		if(FAILED(hr))
		{
			ROS_ERROR_STREAM(name_ + " get robot fail");
			return;
		}
		else
		{
			ROS_INFO_STREAM(name_ + " get robot successful");
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
			ROS_ERROR_STREAM(name_ + " cannot change to slave mode");
			return;
		}
		else
		{
			ROS_INFO_STREAM(name_ + " change robot slave mode successful");
		}


		float joints[7];
		float results[8];
		memset(joints, 0, sizeof(float)*7);
		memset(results, 0, sizeof(float)*8);

		//get joint positions
		if(!set_joints(joints, results))
			return;

		//set joint at current position before turn on motor
		if(!set_joints(results, joints))
			return;

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
		ROS_ERROR_STREAM(name_ + " set motor fail");
		return false;
	}
	else
	{
		if(on_off)
			ROS_INFO_STREAM(name_ + " set motor on");
		else
			ROS_INFO_STREAM(name_ + " set motor off");
		return true;
	}
}

bool DensoVe026a_BCapController::set_joints(float* position, float* result)
{
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
		ROS_ERROR_STREAM(name_ + " set joints fail");
		return false;
	}
	else
	{
		return true;
	}
}

