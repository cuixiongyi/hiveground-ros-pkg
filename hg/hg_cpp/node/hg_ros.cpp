#include <hg_cpp/hg_ros.h>
#include <hg_cpp/denso/denso_ve026a_controller.h>

using namespace hg;


HgROS::HgROS()
	: node_handle_("~")
{
	boost::shared_ptr<hg::Controller>
		ctrl(new DensoVe026a_BCapController(this, "ve026a"));
	controllers_.push_back(ctrl);
}

HgROS::~HgROS()
{

}






int main(int argc, char** argv)
{
	// initialize ROS, specify name of node
	ros::init(argc, argv, "HgROS");

	HgROS hg_ros;
	ros::Rate loop_rate(5); // Hz
	while(hg_ros.node_handle_.ok())
	{
		ros::spinOnce();
		ros::spinOnce();
	}
	return 0;
}
