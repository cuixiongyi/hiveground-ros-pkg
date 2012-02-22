#ifndef _hg_ros_h_
#define _hg_ros_h_

#include <ros/ros.h>
#include <boost/smart_ptr.hpp>
#include <hg_cpp/controller.h>

namespace hg
{

class HgROS
{
public:


	std::vector<boost::shared_ptr<hg::Controller> > controller_;
};


}



#endif
