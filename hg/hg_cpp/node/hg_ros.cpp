#include <hg_cpp/hg_ros.h>
#include <hg_cpp/denso/denso_ve026a_controller.h>

using namespace hg;
using namespace std;



HgROS::HgROS()
	: node_handle_("~")
{
	XmlRpc::XmlRpcValue controllers;
	node_handle_.getParam("controllers", controllers);
	ROS_ASSERT(controllers.getType() == XmlRpc::XmlRpcValue::TypeStruct);

	XmlRpc::XmlRpcValue::iterator i;
	for (i = controllers.begin(); i != controllers.end(); i++)
	{
		string name = i->first;
		cout << i->first << endl;
		string type;
		node_handle_.getParam("controllers/" + name + "/type", type);


		cout << "\t" << type << endl;

		if(type == "ve026a_controller")
		{
			boost::shared_ptr<hg::Controller>
				controller(new DensoVe026a_BCapController(this, name));
			controllers_.push_back(controller);
		}
		else
		{
			ROS_ERROR_STREAM("Unrecognized controller: " + type);
		}
	}
}

HgROS::~HgROS()
{

}

void HgROS::run()
{
	//start all controller
	for(int i = 0; i < controllers_.size(); i++)
	{
		controllers_[i]->startup();
	}


	ros::Rate loop_rate(5); // Hz
	while(node_handle_.ok())
	{

		ros::spinOnce();
		loop_rate.sleep();
	}


	//shutdown all controller
	for(int i = 0; i < controllers_.size(); i++)
	{
		controllers_[i]->shutdown();
	}
}




int main(int argc, char** argv)
{
	// initialize ROS, specify name of node
	ros::init(argc, argv, "HgROS");

	HgROS hg_ros;
	hg_ros.run();

	return 0;
}
