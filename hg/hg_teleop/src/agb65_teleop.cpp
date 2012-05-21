#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "topic_tools/MuxSelect.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

#include <ctime>
#include <iostream>
#include <string>
#include <sstream>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>


using boost::asio::ip::tcp;

const int PUBLISH_FREQ = 25;
const int NUM_JOINTS = 4;
const int PORT = 9560;

using namespace std;

std::string make_daytime_string() {
	using namespace std;
	// For time_t, time and ctime;
	time_t now = time(0);
	return ctime(&now);
}

class Agb65Teleop;
class tcp_connection: public boost::enable_shared_from_this<tcp_connection>
{
public:
	typedef boost::shared_ptr<tcp_connection> pointer;

	Agb65Teleop* agb65_;
	tcp::socket socket_;
	std::string message_;
	boost::array<char, 8192> buffer_;

	static pointer create(boost::asio::io_service& io_service);

	tcp::socket& socket();

	void start();

	tcp_connection(boost::asio::io_service& io_service);

	void handle_write(const boost::system::error_code& /*error*/, size_t /*bytes_transferred*/);

	void handle_read(const boost::system::error_code& e, std::size_t bytes_transferred);

};

class Agb65Teleop
{
public:
	ros::NodeHandle node_handle_, node_handle_private_;
	ros::Subscriber joy_sub_;
	ros::Subscriber joy_state_;
	//ros::Subscriber joint_state_;
	ros::Publisher joint_pubs_[NUM_JOINTS];
	ros::Publisher motor_on_off_pubs_[NUM_JOINTS];
	float joint_angles_[NUM_JOINTS];
	float joint_limits_up_[NUM_JOINTS];
	float joint_limits_down_[NUM_JOINTS];
	float joy_increase_step_;
	//bool joint_updated_;
	tcp::acceptor acceptor_;

	Agb65Teleop(boost::asio::io_service& io);
	~Agb65Teleop();

	void start_accept();
	void handle_accept(tcp_connection::pointer new_connection,
			const boost::system::error_code& error);
	void update_from_socket(std::string command);
	void init();
	void joint_callback(const sensor_msgs::JointStateConstPtr& joint_msg);
	void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg);
	void publish();
};




tcp_connection::pointer tcp_connection::create(boost::asio::io_service& io_service) {
	return pointer(new tcp_connection(io_service));
}

tcp::socket& tcp_connection::socket() {
	return socket_;
}

void tcp_connection::start() {
	socket_.async_read_some(boost::asio::buffer(buffer_),
					boost::bind(&tcp_connection::handle_read, shared_from_this(),
							boost::asio::placeholders::error,
							boost::asio::placeholders::bytes_transferred));


}


tcp_connection::tcp_connection(boost::asio::io_service& io_service) :
	socket_(io_service) {
}

void tcp_connection::handle_write(const boost::system::error_code& /*error*/, size_t /*bytes_transferred*/) {
}

void tcp_connection::handle_read(const boost::system::error_code& e, std::size_t bytes_transferred)
{
	if(!e)
	{
		ROS_INFO("get: %d byte", (int)bytes_transferred);
		std::cout.write(buffer_.data(), bytes_transferred);
		std::cout << std::endl;



		std::string command(buffer_.data());
		agb65_->update_from_socket(command);


		socket_.async_read_some(boost::asio::buffer(buffer_),
						boost::bind(&tcp_connection::handle_read, shared_from_this(),
								boost::asio::placeholders::error,
								boost::asio::placeholders::bytes_transferred));

	}
}


Agb65Teleop::Agb65Teleop(boost::asio::io_service& io)
	:  node_handle_private_("~"),
	   //joint_updated_(false),
	   acceptor_(io, tcp::endpoint(tcp::v4(), PORT))
{
	start_accept();
}

Agb65Teleop::~Agb65Teleop()
{

}

void Agb65Teleop::start_accept()
{
	tcp_connection::pointer new_connection =
		  tcp_connection::create(acceptor_.io_service());

	acceptor_.async_accept(new_connection->socket(), boost::bind(
			&Agb65Teleop::handle_accept, this,
			new_connection,
			boost::asio::placeholders::error));

}

void Agb65Teleop::handle_accept(tcp_connection::pointer new_connection,
		const boost::system::error_code& error)
{
	if (!error)
	{
		new_connection->agb65_ = this;
		new_connection->start();
		ROS_INFO("Client connected");
		start_accept();

	}
	else
	{
		ROS_INFO("Client connect error");
	}
}

void Agb65Teleop::update_from_socket(std::string command)
{
	std::stringstream ss;
	ss << command;
	for(int i = 0; i < NUM_JOINTS ; i++)
	{
		ss >> joint_angles_[i];

		if(joint_angles_[i] >= 9.0)
		{
			//turn on motor
			std_msgs::Bool flag;
			flag.data = 1;
			motor_on_off_pubs_[i].publish(flag);
			return;
		}
		else if(joint_angles_[i] <= -9.0)
		{
			//turn off motor
			std_msgs::Bool flag;
			flag.data = 0;
			motor_on_off_pubs_[i].publish(flag);
			return;
		}

		joint_angles_[i] = joint_angles_[i]*M_PI;
	}




	publish();

}

void Agb65Teleop::init()
{
	joy_sub_ = node_handle_.subscribe("joy",
			1, &Agb65Teleop::joy_callback, this);


	//joint_state_ = node_handle_.subscribe("joint_states",
	//					10, &Agb65Teleop::joint_callback, this);

	for(int i = 0; i < NUM_JOINTS ; i++)
	{
		std::stringstream ss;
		std::string s;
		ss << "/HgROS/" << "SRV" << i << "/command";
		ss >> s;
		ROS_INFO_STREAM(s);
		joint_pubs_[i] = node_handle_.advertise<std_msgs::Float32>(s, 1);

		ss.clear();
		ss << "/HgROS/" << "SRV" << i << "/motor_on_off";
		ss >> s;
		ROS_INFO_STREAM(s);
		motor_on_off_pubs_[i] = node_handle_.advertise<std_msgs::Bool>(s, 1);

		joint_angles_[i] = 1.57;
		joy_increase_step_ = 0.05;

		joint_limits_up_[i] = 3.14;
		joint_limits_down_[i] = 0.0;
	}

	//motor_on_off_pub_ = node_handle_.advertise<std_msgs::Bool>("/HgROS/ve026a_controller/motor_on_off", 1);
}

void Agb65Teleop::joint_callback(const sensor_msgs::JointStateConstPtr& joint_msg)
{
	/*
	if(!joint_updated_)
	{
		ROS_INFO("Joint state!! %d", joint_msg->name.size());
		for(int i = 0; i < joint_msg->name.size(); i++)
		{
			ROS_INFO_STREAM(joint_msg->name[i] << " " << joint_msg->position[i]);
			joint_angles_[i] = joint_msg->position[i];
		}
		joint_updated_ = true;
	}
	*/
}

void Agb65Teleop::joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
	//ROS_INFO("Joystick!!");
	stringstream ss;

	if(joy_msg->buttons.size() < 12)
	{
		ROS_WARN("You need at less 12 buttons joystick for control all axes (excepting the gripper)");
		return;
	}

	std::vector<int32_t> buttons = joy_msg->buttons;
	std::vector<float> axes = joy_msg->axes;


	//J1
	if(axes[4] > 0) joint_angles_[0] += joy_increase_step_;
	if(axes[4] < 0) joint_angles_[0] -= joy_increase_step_;

	//J2
	if(axes[5] > 0) joint_angles_[1] += joy_increase_step_;
	if(axes[5] < 0) joint_angles_[1] -= joy_increase_step_;

	//J3
	if(buttons[0]) joint_angles_[2] += joy_increase_step_;
	if(buttons[2]) joint_angles_[2] -= joy_increase_step_;


	//J4
	if(buttons[3])
	{
		joint_angles_[0] += joy_increase_step_;
		joint_angles_[1] += joy_increase_step_;
		joint_angles_[2] += joy_increase_step_;
	}
	if(buttons[1])
	{
		joint_angles_[0] -= joy_increase_step_;
		joint_angles_[1] -= joy_increase_step_;
		joint_angles_[2] -= joy_increase_step_;
	}

	/*
	//J1
	if(buttons[6]) joint_angles_[4] += joy_increase_step_;
	if(buttons[4]) joint_angles_[4] -= joy_increase_step_;

	//J2
	if(buttons[7]) joint_angles_[5] += joy_increase_step_;
	if(buttons[5]) joint_angles_[5] -= joy_increase_step_;



	//J7
	if(buttons[10]) joint_angles_[6] += joy_increase_step_;
	if(buttons[11]) joint_angles_[6] -= joy_increase_step_;
	*/

	if(buttons[8])
	{
		for(int i = 0; i < NUM_JOINTS; i++)
		{
			joint_angles_[i] = 1.57;
		}
	}

	publish();
}


void Agb65Teleop::publish()
{
	//if(!joint_updated_) return;
	for(int i = 0; i < NUM_JOINTS; i++)
	{
		if(joint_angles_[i] > joint_limits_up_[i])
			joint_angles_[i] = joint_limits_up_[i];
		if(joint_angles_[i] < joint_limits_down_[i])
			joint_angles_[i] = joint_limits_down_[i];

		std::cout << joint_angles_[i] << " ";

		std_msgs::Float32 msg;
		msg.data = joint_angles_[i];
		joint_pubs_[i].publish(msg);
	}
	std::cout << endl;
}



int main(int argc, char** argv)
{


	ros::init(argc, argv, "ve026a_teleop");

	boost::asio::io_service io;
	Agb65Teleop abg65_teleop(io);
	abg65_teleop.init();
	boost::thread t(boost::bind(&boost::asio::io_service::run, &io));
	ros::Rate rate(PUBLISH_FREQ);
	while(ros::ok())
	{
		ros::spinOnce();
		//ve026a_teleop.read_socket();
		rate.sleep();
	}
	return 0;

}
