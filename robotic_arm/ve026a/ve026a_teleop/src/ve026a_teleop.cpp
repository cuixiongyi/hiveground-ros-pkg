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

const int PUBLISH_FREQ = 50;
const int NUM_JOINTS = 7;

using namespace std;

std::string make_daytime_string() {
	using namespace std;
	// For time_t, time and ctime;
	time_t now = time(0);
	return ctime(&now);
}

class Ve026aTeleop
{
public:
	ros::NodeHandle node_handle_, node_handle_private_;
	ros::Subscriber joy_sub_;
	ros::Subscriber joy_state_;
	ros::Subscriber joint_state_;
	ros::Publisher joint_pubs_[NUM_JOINTS];
	float joint_angles_[NUM_JOINTS];
	float joint_limits_up_[NUM_JOINTS];
	float joint_limits_down_[NUM_JOINTS];
	float joy_increase_step_;
	bool joint_updated_;

	tcp::socket socket_;
	tcp::acceptor acceptor_;

	/// Strand to ensure the connection's handlers are not called concurrently.
	//boost::asio::io_service::strand strand_;

	/// Buffer for incoming data.
	boost::array<char, 128> buffer_;


	Ve026aTeleop(boost::asio::io_service& io)
		:  node_handle_private_("~"),
		   joint_updated_(false),
		   socket_(io),
		   acceptor_(io, tcp::endpoint(tcp::v4(), 9559))
		   //strand_(io)
	{
		start_accept();
	}

	~Ve026aTeleop()
	{

	}

	void start_accept() {
		acceptor_.async_accept(socket_, boost::bind(
				&Ve026aTeleop::handle_accept, this, boost::asio::placeholders::error));

	}

	void handle_accept(const boost::system::error_code& error)
	{
		if (!error)
		{
		  //new_connection->start();
		  //start_accept();
			std::string message = make_daytime_string();
			ROS_INFO("Client connected");
			boost::asio::async_write(socket_, boost::asio::buffer(message),
				boost::bind(&Ve026aTeleop::handle_write, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
		}
	}

	void handle_write(const boost::system::error_code& error, size_t size)
	{
		ROS_INFO("Write to client");
	}

	void handle_read(const boost::system::error_code& error, size_t size)
	{
		ROS_INFO("read requested");
		if(!error)
		{
			ROS_INFO("get %d bytes", size);
		}
		else
		{
			ROS_INFO("read_error", size);
		}
	}


	void read_socket()
	{
		if(socket_.is_open())
		{
				/*
				socket_.async_read_some(boost::asio::buffer(buffer_),
							boost::bind(&Ve026aTeleop::handle_read, this,
							boost::asio::placeholders::error,
							boost::asio::placeholders::bytes_transferred));
				*/

			if(socket_.available())
			{
				boost::array<char, 128> buf;
				boost::system::error_code error;

				size_t len = socket_.read_some(boost::asio::buffer(buf), error);

				ROS_INFO("Get %d bytes", len);

				if (error == boost::asio::error::eof)
				{
					ROS_INFO("Connection closed");
					socket_.close();
					start_accept();
					//break; // Connection closed cleanly by peer.
				} else if (error)
					throw boost::system::system_error(error); // Some other error.

				//std::cout.write(buf.data(), len);
				if(len == 62)
				{
					ROS_INFO("get: %s", buf.data());

					std::stringstream ss;
					ss << buf.data();
					for(int i = 0; i < NUM_JOINTS ; i++)
					{
						ss >> joint_angles_[i];
					}
				}
			}
		}
	}

	void init()
	{
		joy_sub_ = node_handle_.subscribe("joy",
				1, &Ve026aTeleop::joy_callback, this);


		joint_state_ = node_handle_.subscribe("joint_states",
						10, &Ve026aTeleop::joint_callback, this);

		for(int i = 0; i < NUM_JOINTS ; i++)
		{
			std::stringstream ss;
			std::string s;
			ss << "/HgROS/" << "J" << i+1 << "/command";
			ss >> s;
			ROS_INFO_STREAM(s);
			joint_pubs_[i] = node_handle_.advertise<std_msgs::Float32>(s, 1);
			joint_angles_[i] = 0;
			joy_increase_step_ = 0.1;
		}

		//J1
		joint_limits_up_[0] = 1.83259;
		joint_limits_down_[0] = -1.39626;

		//J2
		joint_limits_up_[1] = 1.57075;
		joint_limits_down_[1] = -0.61086;

		//J3
		joint_limits_up_[2] = 2.35619;
		joint_limits_down_[2] = -1.30899;

		//J4
		joint_limits_up_[3] = 2.44346;
		joint_limits_down_[3] = -2.44346;

		//J5
		joint_limits_up_[4] = 1.83259;
		joint_limits_down_[4] = -1.57075;

		//J6
		joint_limits_up_[5] = 2.44346;
		joint_limits_down_[5] = -2.44346;

		//J7
		joint_limits_up_[6] = 0.43633;
		joint_limits_down_[6] = -0.63633;




	}

	void joint_callback(const sensor_msgs::JointStateConstPtr& joint_msg)
	{
		if(!joint_updated_)
		{
			ROS_INFO("Joint state!! %d", joint_msg->name.size());
			for(int i = 0; i < joint_msg->name.size(); i++)
			{
				ROS_INFO_STREAM(joint_msg->name[i] << " " << joint_msg->position[i]);
				joint_angles_[i] = joint_msg->position[i];
			}
			joint_updated_ = true;
			joint_state_.shutdown();
		}
	}

	void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg)
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
		if(buttons[3]) joint_angles_[3] += joy_increase_step_;
		if(buttons[1]) joint_angles_[3] -= joy_increase_step_;

		//J1
		if(buttons[6]) joint_angles_[4] += joy_increase_step_;
		if(buttons[4]) joint_angles_[4] -= joy_increase_step_;

		//J2
		if(buttons[7]) joint_angles_[5] += joy_increase_step_;
		if(buttons[5]) joint_angles_[5] -= joy_increase_step_;



		//J7
		if(buttons[10]) joint_angles_[6] += joy_increase_step_;
		if(buttons[11]) joint_angles_[6] -= joy_increase_step_;


		if(buttons[8])
		{
			for(int i = 0; i < NUM_JOINTS; i++)
			{
				joint_angles_[i] = 0;
			}
		}


		/*
		for(int i = 0; i < joy_msg->buttons.size(); i++)
		{
			std::cout << joy_msg->buttons[i] << " ";
		}
		std::cout << endl;
		*/
		/*
		for(int i = 0; i < joy_msg->axes.size(); i++)
		{
			//switch()

			if(joy_msg->axes[i] > 0)
			{
				joint_angles_[i] += 0.1;
			}
			else if(joy_msg->axes[i] < 0)
			{
				joint_angles_[i] -= 0.1;
			}

			if(joint_angles_[i] > joint_limits_up_[i])
				joint_angles_[i] = joint_limits_up_[i];
			if(joint_angles_[i] < joint_limits_down_[i])
				joint_angles_[i] = joint_limits_down_[i];

			std::cout << joy_msg->axes[i] << " ";
			//std::cout << joint_angles_[i] << " ";

	        //std_msgs::Float32 msg;
	        //msg.data = joint_angles_[i];

			//joint_pubs_[i].publish(msg);
		}
		std::cout << endl;
		*/

		for(int i = 0; i < NUM_JOINTS; i++)
		{
			std::cout << joint_angles_[i] << " ";
		}
		std::cout << endl;

		publish();
	}


	void publish()
	{
		if(!joint_updated_) return;
		for(int i = 0; i < NUM_JOINTS; i++)
		{
			if(joint_angles_[i] > joint_limits_up_[i])
				joint_angles_[i] = joint_limits_up_[i];
			if(joint_angles_[i] < joint_limits_down_[i])
				joint_angles_[i] = joint_limits_down_[i];

			std_msgs::Float32 msg;
			msg.data = joint_angles_[i];
			joint_pubs_[i].publish(msg);
		}
	}




};


int main(int argc, char** argv)
{


	ros::init(argc, argv, "ve026a_teleop");

	boost::asio::io_service io;
	Ve026aTeleop ve026a_teleop(io);
	ve026a_teleop.init();
	boost::thread t(boost::bind(&boost::asio::io_service::run, &io));
	ros::Rate rate(PUBLISH_FREQ);
	while(ros::ok())
	{
		ros::spinOnce();
		ve026a_teleop.read_socket();
		rate.sleep();
	}

	//t.interrupt();
	//t.join();

	return 0;

}
