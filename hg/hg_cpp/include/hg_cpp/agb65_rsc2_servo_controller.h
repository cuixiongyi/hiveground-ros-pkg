#ifndef _hg_agb65_rsc2_servo_controller_
#define _hg_agb65_rsc2_servo_controller_

#include <hg_cpp/hg_ros.h>
#include <hg_cpp/controller.h>
#include <hg_cpp/joint.h>
#include <hg_cpp/serial_port.h>



namespace hg
{

class HgROS;


/**
 * Asakusagiken AGB65 RSC2 servo controller class.
 * Home page: http://robotsfx.com/robot/AGB65_RSC2.html
 * Protocol:
 * Drive all servos: 	[255][ID][Length(14)][Command(1)][P0][P1]...[P10][P11][Speed]
 * Drive single servo: 	[255][ID][Length(4)][Command(2)][Servo ID][Position][Speed]
 * Stop all servos:		[255][ID][Length(1)][Command(3)]
 * Stop single servo: 	[255][ID][Length(2)][Command(4)][Servo ID]
 * 180 deg mode: 		[255][ID][Length(1)][Command(5)]
 * 0-255 mode:			[255][ID][Length(1)][Command(6)]
 * Self-test:			[255][ID][Length(1)][254]
 * Respond(RX):			[255][ID][Length(1)][254]
 */
class Agb65Rsc2ServoController : public Controller
{
public:
	enum Commands {
		DRIVE_ALL_SERVOS = 1,
		DRIVE_SERVO,
		STOP_ALL_SERVOS,
		STOP_SERVO,
		MODE_180,
		MODE_255, //default when turn on
		SELF_TEST = 254
	};


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
		ROS_INFO_STREAM(name_ + ": control_rate: " << control_rate);

		//add joints
		XmlRpc::XmlRpcValue joints;
		node_handle_.getParam("controllers/" + name_ + "/joints", joints);
		ROS_ASSERT(joints.getType() == XmlRpc::XmlRpcValue::TypeArray);
		for(int i = 0; i < joints.size(); i++)
		{
			std::string joint_name;
			ROS_ASSERT(joints[i].getType() == XmlRpc::XmlRpcValue::TypeString);
			joint_name = static_cast<std::string>(joints[i]);
			//if(hg_ros->joints_.(joint_name))
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

			//set 180 degree mode
			//uint8_t data1[] = {255, 3, 1, 5};
			//serial_.Out(data1, 4, 100);

			//uint8_t data[] = {255, 3, 4, 2, 0, 180, 1};
			//uint8_t data[] = {255, 3, 4, 2, 0, 80, 1};
			//serial_.Out(data, 7, 100);
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
