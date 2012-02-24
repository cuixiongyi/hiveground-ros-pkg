#ifndef _hg_ros_h_
#define _hg_ros_h_


#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sensor_msgs/JointState.h>

#include <boost/smart_ptr.hpp>


namespace hg
{

class Controller;
class Joint;

typedef std::pair<std::string, boost::shared_ptr<hg::Controller> > ControllerPair;
typedef std::pair<std::string, boost::shared_ptr<hg::Joint> > JointPair;
typedef std::map<std::string, boost::shared_ptr<hg::Controller> > ControllerMap;
typedef std::map<std::string, boost::shared_ptr<hg::Joint> > JointMap;

class HgROS
{
public:
	HgROS();
	~HgROS();

	void run();

	void publish();

	ros::NodeHandle node_handle_;

	double update_rate_;
	bool simulate_;




	ros::Publisher publisher_diagnostic_;
	ros::Duration diagnotic_duration_;
	ros::Time next_diagnotic_time_;

	ros::Publisher publisher_joint_state_;
	ros::Duration joint_state_duration_;
	ros::Time next_joint_state_time_;

	ControllerMap controllers_;
	JointMap joints_;


};


}



#endif
