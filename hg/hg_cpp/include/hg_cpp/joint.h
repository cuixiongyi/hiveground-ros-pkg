#ifndef _hg_joints_h_
#define _hg_joints_h_

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#include <hg_cpp/hg_ros.h>

namespace hg
{

class Controller;

class Joint
{
public:
	Joint(hg::HgROS* hg_ros, const std::string& name)
		: name_(name),
		  position_(0),
		  velocity_(0),
		  last_update_(ros::Time::now()),
		  hg_ros_(hg_ros)
	{ }

	virtual ~Joint() = 0;

	virtual double interpolate(double dt) = 0;

	virtual double set_feedback_data(double feedback) = 0;

	virtual double set_goal_position(double position) = 0;

	virtual diagnostic_msgs::DiagnosticStatus get_diagnostics()
	{
		diagnostic_msgs::DiagnosticStatus message;
		message.name = name_;
		message.level = diagnostic_msgs::DiagnosticStatus::OK;
		message.message = "OK";
		return message;
	}


	std::string name_;

	double position_;
	double velocity_;
	ros::Time last_update_;

	boost::shared_ptr<hg::Controller> controller_;
	hg::HgROS* hg_ros_;
};


}



#endif
