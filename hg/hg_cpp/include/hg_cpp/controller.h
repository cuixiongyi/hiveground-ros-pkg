#ifndef _hg_controller_h_
#define _hg_controller_h_

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

namespace hg
{

class HgROS;

class Controller
{
public:
	Controller(HgROS& hg_ros, const std::string& name);
	virtual ~Controller() = 0;

	virtual void startup() = 0;
	virtual void update() = 0;
	virtual void shutdown() = 0;
	virtual bool active() { return false; }
	virtual diagnostic_msgs::DiagnosticStatus get_diagnostics()
	{
		diagnostic_msgs::DiagnosticStatus message;
		message.name = name_;
		message.level = diagnostic_msgs::DiagnosticStatus::OK;
		message.message = "OK";
		return message;
	}


	std::string name_;
	bool fake_;
	bool pause_;

	std::vector<std::string>	joint_names_;
	std::vector<double>			joint_positions_;
	std::vector<double>			joint_velocities_;


};










}


#endif
