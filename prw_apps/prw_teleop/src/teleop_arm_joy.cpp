#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <string>
#include <sstream>

using namespace std;

class TeleopArm
{
public:
  TeleopArm()
  {
    ros::NodeHandle nh_private_("~");
    sub_ = nh_.subscribe("joy", 1, &TeleopArm::joyCallback, this);
    ROS_DEBUG("teleop_arm started");

    nh_private_.param("move_rate", move_rate_, 0.02);
    ROS_INFO("Joint move rate is %f rad/s", move_rate_);

    nh_private_.param("dead_man_button", dead_man_button_, 4);
    ROS_INFO("Dead man button is %d", dead_man_button_);

    //Joint command
    for(int i = 0; i < 6; i++)
    {
      //axis map
      stringstream ss1;
      ss1 << "axis_j" << i;
      nh_private_.param(ss1.str(), axis_map_[i], i);
      ROS_INFO("Mapped joint %d to axis %d", i, axis_map_[i]);

      //publish
      stringstream ss2;
      ss2 << "command" << i;
      joint_pub_[i] = nh_.advertise<std_msgs::Float64>(ss2.str(), 1, false);
    }



    //Gripper command
    //gripper_pub_ = nh_.advertise<std_msgs::Float64>(ss.str(), 1, false);


  }

  void joyCallback(const sensor_msgs::JoyConstPtr& joy)
  {
    //ROS_INFO("Got a joy msg");
    //if(joy->axes.size() < 6)
    //{
      //ROS_WARN_THROTTLE(1, "need a joy stick with 6 axes for controlling all joints");
    //}
    if(dead_man_button_ >= joy->buttons.size())
    {
      ROS_ERROR_THROTTLE(1.0, "Not enough buttons on the joy stick");
      return;
    }

    if(joy->axes.size() < 6)
    {
      ROS_ERROR_THROTTLE(1.0, "Not enough axes on the joy stick");
      return;
    }


    if(joy->buttons[dead_man_button_] != 1)
    {
      ROS_WARN_THROTTLE(1.0, "Press dead man button (%d) to enable", dead_man_button_);
      return;
    }

    std_msgs::Float64 msg;
    for(int i = 0; i < 6; i++)
    {
      if(fabs(joy->axes[axis_map_[i]]) > 0.5)
      {
        msg.data = (joy->axes[axis_map_[i]] > 0.1) ? move_rate_ : -move_rate_;
        ROS_INFO("move joint %d %f rad", msg.data);
        joint_pub_[i].publish(msg);
      }
    }


  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  double move_rate_;
  int dead_man_button_;
  int axis_map_[6];
  ros::Publisher joint_pub_[6];
  //ros::Publisher gripper_pub_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_arm");

  TeleopArm teleop_arm;

  ros::spin();

}
