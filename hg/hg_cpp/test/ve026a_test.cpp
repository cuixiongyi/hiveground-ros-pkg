#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <hg_msgs/MoveArmAction.h>
#include <hg_msgs/ArmAction.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

using namespace std;

int main (int argc, char **argv)
{
	ros::init(argc, argv, "test_prw_ve026a");

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<hg_msgs::MoveArmAction> ac("move_arm", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	hg_msgs::MoveArmGoal goal;
	goal.header.stamp = ros::Time::now();
	goal.header.frame_id = "base_link";
	hg_msgs::ArmAction action;
	//action.type = hg_msgs::ArmAction::MOVE_ARM;
	action.type = hg_msgs::ArmAction::MOVE_GRIPPER;
	action.goal.position.x = 0.064107;
	action.goal.position.y = 0.110339;
	action.goal.position.z = 0.289558;
	action.command = 0.0;


	action.goal.orientation.x = -0.404339;
	action.goal.orientation.y = 0.10551;
	action.goal.orientation.z = 0.229387;
	action.goal.orientation.w = 0.879067;
	action.move_time = ros::Duration(10.0);

	goal.motions.push_back(action);

	action.type = hg_msgs::ArmAction::MOVE_ARM;
	action.goal.position.x = 0.064107;
	action.goal.position.y = 0.110339;
	action.goal.position.z = 0.289558;
	action.command = 0.0;


	action.goal.orientation.x = -0.404339;
	action.goal.orientation.y = 0.10551;
	action.goal.orientation.z = 0.229387;
	action.goal.orientation.w = 0.879067;
	action.move_time = ros::Duration(20.0);




	goal.motions.push_back(action);

	ac.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(40.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
		ROS_INFO("Action did not finish before the time out.");

	//exit
	return 0;
}
