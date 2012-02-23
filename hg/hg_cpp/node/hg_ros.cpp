#include <hg_cpp/hg_ros.h>

//Joints
#include <hg_cpp/joint _buildin.h>


//Controllers
#include <hg_cpp/denso/denso_ve026a_controller.h>



using namespace hg;
using namespace std;



HgROS::HgROS()
	: node_handle_("~"),
	  simulate_(false)
{
	//Add joints
	XmlRpc::XmlRpcValue joints;
	node_handle_.getParam("joints", joints);
	ROS_ASSERT(joints.getType() == XmlRpc::XmlRpcValue::TypeStruct);
	XmlRpc::XmlRpcValue::iterator itr;
	for (itr = joints.begin(); itr != joints.end(); itr++)
	{
		string name = itr->first;
		string type;
		if(!node_handle_.getParam("joints/" + name + "/type", type))
		{
			continue;
		}

		if(type == "buildin")
		{
			ROS_INFO_STREAM("add buildin joint: " + name);
			boost::shared_ptr<hg::Joint>
				joint(new JointBuildin(this, name));
			joints_.insert(JointPair(name, joint));
		}
		else
		{
			ROS_ERROR_STREAM("Unrecognized joint: " + type);
		}
	}


	//Add controllers
	XmlRpc::XmlRpcValue controllers;
	node_handle_.getParam("controllers", controllers);
	ROS_ASSERT(controllers.getType() == XmlRpc::XmlRpcValue::TypeStruct);

	for (itr = controllers.begin(); itr != controllers.end(); itr++)
	{
		string name = itr->first;
		string type;
		if(!node_handle_.getParam("controllers/" + name + "/type", type))
		{
			continue;
		}

		if(type == "ve026a_controller")
		{
			ROS_INFO_STREAM("add ve026a_controller: " + name);
			boost::shared_ptr<hg::Controller>
				controller(new DensoVe026a_BCapController(this, name));
			controllers_.insert(ControllerPair(name, controller));
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
	for(ControllerMap::iterator itr = controllers_.begin();
			itr != controllers_.end(); itr++)
	{
		itr->second->startup();
	}


	ros::Rate loop_rate(5); // Hz
	while(node_handle_.ok())
	{

		ros::spinOnce();
		loop_rate.sleep();
	}


	//shutdown all controller
	for(ControllerMap::iterator itr = controllers_.begin();
			itr != controllers_.end(); itr++)
	{
		itr->second->shutdown();
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
