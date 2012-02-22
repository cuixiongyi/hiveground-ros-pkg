#ifndef _hg_ros_h_
#define _hg_ros_h_


#include <ros/ros.h>

#include <sensor_msgs/JointState.h>


#include <boost/smart_ptr.hpp>


namespace hg
{

class Controller;
class Joint;

class HgROS
{
public:
	HgROS();
	~HgROS();

	double update_rate_;
	bool simulate_;



	ros::Publisher publisher_joint_state_;
	ros::Publisher publisher_diagnostic_;



	std::vector<boost::shared_ptr<hg::Controller> > controllers_;
	std::vector<boost::shared_ptr<hg::Joint> > joints_;

	ros::NodeHandle node_handle_;
};


}



#endif
