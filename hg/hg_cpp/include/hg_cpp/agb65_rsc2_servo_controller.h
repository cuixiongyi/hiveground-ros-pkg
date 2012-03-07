#ifndef _hg_agb65_rsc2_servo_controller_
#define _hg_agb65_rsc2_servo_controller_

#include <hg_cpp/hg_ros.h>
#include <hg_cpp/controller.h>
#include <hg_cpp/joint.h>
#include <hg_cpp/serial_port.h>



namespace hg
{

class HgROS;

class Agb65Rsc2ServoController : public Controller
{
public:
	Agb65Rsc2ServoController(hg::HgROS* hg_ros, const std::string& name)
		: Controller(hg_ros, name)
	{
		std::string port_param_name = "controllers/" + name_ + "/port";
		//std::cout << port_param_name << std::endl;
		if(!node_handle_.hasParam(port_param_name))
		{
			ROS_ERROR_STREAM(name_ + ": couldn't find " + port_param_name +
					"please check if ALL parameters have been set correctly");
		}

		node_handle_.param<std::string>(
				port_param_name, port_name_, "/dev/ttyUSB0");
		ROS_INFO_STREAM(name_ + ": serial port: " + port_name_);

		//baudrate
		std::string baudrate_param_name = "controllers/" + name_ + "/baudrate";
		node_handle_.param<int>(
				baudrate_param_name, baud_rate_, 9600);
		ROS_INFO_STREAM(name_ << ": baudrate port: " << baud_rate_);

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



	}

	~Agb65Rsc2ServoController()
	{

	}

	void startup()
	{
		if(!hg_ros_->simulate_)
		{
			int error = serial_.Open(port_name_);
			if(error) return;
			error = serial_.Initialise(baud_rate_, 8, 1, SerialPort::PARITY_NONE);
			if(error) return;

			ROS_INFO_STREAM(name_ + ": serial port connected");

			uint8_t data[] = {255, 3, 4, 2, 0, 127, 20};
			serial_.Out(data, 7, 100);
		}


	}

	void update()
	{

	}

	void shutdown()
	{

	}

	bool active()
	{
		return false;
	}


	std::string port_name_;
	int baud_rate_;
	int data_bits_;
	int stop_bits_;
	int parity_;

	SerialPort serial_;

	double control_rate_;
	bool is_running_;

};

}

#endif //_hg_agb65_rsc2_servo_controller_
