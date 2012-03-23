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

const int PUBLISH_FREQ = 20;
const int NUM_JOINTS = 7;

using namespace std;

class Ve026aTeleop
{
public:
	ros::NodeHandle node_handle_, node_handle_private_;
	ros::Subscriber joy_sub_;
	ros::Publisher joint_pubs_[NUM_JOINTS];


	Ve026aTeleop()
		: node_handle_private_("~")
	{

	}

	~Ve026aTeleop()
	{

	}

	void init()
	{
		joy_sub_ = node_handle_.subscribe("joy",
				10, &Ve026aTeleop::joy_callback, this);
		for(int i = 0; i < NUM_JOINTS ; i++)
		{
			std::stringstream ss;
			std::string s;
			ss << "/HgROS/" << "J" << i+1 << "/command";
			ss >> s;
			ROS_INFO_STREAM(s);
			joint_pubs_[i] = node_handle_.advertise<std_msgs::Float32>(s, 1);
		}
	}

	void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg)
	{
		ROS_INFO("Joystick!!");
		stringstream ss;
		for(int i = 0; i < joy_msg->buttons.size(); i++)
		{

		}
	}





};




int main(int argc, char** argv)
{
	ros::init(argc, argv, "ve026a_teleop");

	Ve026aTeleop ve026a_teleop;

	ve026a_teleop.init();

	ros::Rate rate(PUBLISH_FREQ);

	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
