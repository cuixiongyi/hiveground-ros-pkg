#ifndef _hg_joints_h_
#define _hg_joints_h_

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#include <hg_cpp/hg_ros.h>
#include <urdf/model.h>

namespace hg
{

class Joint
{
public:
	Joint(hg::HgROS* hg_ros, const std::string& name)
		: name_(name),
		  lower_(0), upper_(0), velocity_limit_(0),
		  position_(0),
		  velocity_(0),
		  last_update_(ros::Time::now()),
		  hg_ros_(hg_ros),
		  node_handle_(hg_ros->node_handle_)
	{
		std::string robot;
		if(!node_handle_.getParam("joints/" + name_ +"/robot", robot))
		{
			ROS_ERROR_STREAM(name_ + "couldn't find robot type" +
					" check if ALL parameters have been set correctly");
			return;
		}

		//Load joint information for URDF
		std::string param_name = "robot_description/" + robot;
		std::string full_param_name;
		std::string xml_string;
		node_handle_.searchParam(param_name, full_param_name);
		node_handle_.getParam(param_name.c_str(), xml_string);
		if (xml_string.size()==0)
		{
			ROS_ERROR_STREAM(name_ +
					": unable to load robot model from "
					"parameter server robot_description\n");
			return;
		}

		urdf::Model model;
		if (!model.initString(xml_string))
		{
			ROS_ERROR_STREAM(name_ + ": failed to parse urdf file");
			return;
		}
		ROS_INFO_STREAM(name_ + ": successfully parsed urdf file");

		if(model.getName() != robot)
		{
			ROS_ERROR_STREAM(name_ +": wrong urdf file");
			return;
		}

		boost::shared_ptr<const urdf::Joint> joint = model.getJoint(name_);
		if(!joint)
		{
			ROS_ERROR_STREAM(name_ +": cannot find joint information");
			return;
		}

		lower_ = joint->limits->lower;
		upper_ = joint->limits->upper;
		velocity_limit_ = joint->limits->velocity;
	}

	virtual ~Joint() { }

	virtual double interpolate(double dt) = 0;

	virtual void set_feedback_data(double feedback) = 0;

	virtual double set_position(double position) = 0;

	virtual diagnostic_msgs::DiagnosticStatus get_diagnostics()
	{
		diagnostic_msgs::DiagnosticStatus message;
		message.name = name_;
		message.level = diagnostic_msgs::DiagnosticStatus::OK;
		message.message = "OK";
		return message;
	}


	std::string name_;
	double lower_, upper_, velocity_limit_;
	double position_;
	double velocity_;
	ros::Time last_update_;

	hg::HgROS* hg_ros_;
	hg::Controller* controller_;
	ros::NodeHandle& node_handle_;
};


}



#endif
