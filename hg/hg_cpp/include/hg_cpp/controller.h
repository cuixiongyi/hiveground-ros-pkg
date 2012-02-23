#ifndef _hg_controller_h_
#define _hg_controller_h_

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#include <hg_cpp/hg_ros.h>

namespace hg
{

/**
 * Controller abstract class.
 */
class Controller
{
public:

	/**
	 * Constructs a Controller instance.
	 * @param hg_ros HgROS instanc.
	 * @param name the controller name.
	 */
	Controller(hg::HgROS* hg_ros, const std::string& name)
		: name_(name),
		  pause_(false),
		  hg_ros_(hg_ros),
		  node_handle_(hg_ros->node_handle_)
	{ }

	/**
	 * Destructor
	 */
	virtual ~Controller()
	{ };

	/**
	 * Start the controller, do any hardware setup needed.
	 */
	virtual void startup() = 0;

	/**
	 * Do any read/writes to device.
	 */
	virtual void update() = 0;

	/**
	 * Stop the controller, do any hardware shutdown needed.
	 */
	virtual void shutdown() = 0;

	/**
	 * Is the controller actively sending commands to joints/robots
	 */
	virtual bool active() = 0;

	/**
	 * Get a diagnostics message for this controller.
	 */
	virtual diagnostic_msgs::DiagnosticStatus get_diagnostics()
	{
		diagnostic_msgs::DiagnosticStatus message;
		message.name = name_;
		message.level = diagnostic_msgs::DiagnosticStatus::OK;
		message.message = "OK";
		return message;
	}


	std::string name_;
	bool pause_;

	JointMap joints_;

	hg::HgROS* hg_ros_;
	ros::NodeHandle& node_handle_;
};










}


#endif
