#ifndef _hg_publisher_
#define _hg_publisher_

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sensor_msgs/JointState.h>

#include <hg_cpp/hg_ros.h>
#include <hg_cpp/controller.h>
#include <hg_cpp/joint.h>

namespace hg
{

class DiagnoticsPublisher
{
	ros::Publisher publisher_;
public:
	DiagnoticsPublisher(hg::HgROS* hg_ros)
	{
		//publisher_ = hg_ros->node_handle_.advertise<diagnostic_msgs::>("diagnostics", 1);
	}

	void update(const ControllerMap& controllers, const JointMap& joints)
	{

	}
};

class JointStatePublisher
{
	ros::Publisher publisher_;
public:
	JointStatePublisher(hg::HgROS* hg_ros)
	{
		//publisher_ = hg_ros->node_handle_.advertise<sensor_msgs::JointState>("joint_states", 1);
	}

	void update(const ControllerMap& controllers, const JointMap& joints)
	{

	}
};

}




#endif
