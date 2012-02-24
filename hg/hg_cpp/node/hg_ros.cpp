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
	node_handle_.getParam("simulate", simulate_);
	if(simulate_)
		ROS_INFO_STREAM("Simulated HgROS");


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

	//publisher
	publisher_diagnostic_ = node_handle_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
	diagnotic_duration_ = ros::Duration(1.0); //1 Hz
	next_diagnotic_time_ = ros::Time::now() + diagnotic_duration_;

	publisher_joint_state_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 1);
	joint_state_duration_= ros::Duration(1.0/10.0); //10 Hz
	next_joint_state_time_ = ros::Time::now() + joint_state_duration_;

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


	ros::Rate loop_rate(100); // Hz
	while(node_handle_.ok())
	{
		publish();


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


void HgROS::publish()
{
	if(ros::Time::now() > next_joint_state_time_)
	{
		sensor_msgs::JointState message;
		message.header.stamp = ros::Time::now();
		for(JointMap::iterator itr = joints_.begin(); itr != joints_.end(); itr++)
		{
			message.name.push_back(itr->second->name_);
			message.position.push_back(itr->second->position_);
			message.velocity.push_back(itr->second->velocity_);
		}
		publisher_joint_state_.publish(message);
		next_joint_state_time_ = ros::Time::now() + joint_state_duration_;
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
