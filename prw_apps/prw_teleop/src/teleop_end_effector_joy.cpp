#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

#include <string>
#include <sstream>

using namespace std;

class TeleopEndeffectorJoy
{
  enum ControlType {
    AXIS,
    BUTTONS
  };

public:
  TeleopEndeffectorJoy()
  {
    ros::NodeHandle nh_private_("~");
    sub_ = nh_.subscribe("joy", 1, &TeleopEndeffectorJoy::joyCallback, this);
    ROS_DEBUG("teleop_arm started");

    nh_private_.param("move_rate", move_rate_, 0.01);
    ROS_INFO("End effector moving rate is %f m/repeating rate", move_rate_);

    nh_private_.param("rotate_rate", rotate_rate_, 0.01);
    ROS_INFO("End effector rotating rate is %f rad/repeating rate", rotate_rate_);

    nh_private_.param("dead_man_button", dead_man_button_, 4);
    ROS_INFO("Dead man button is %d", dead_man_button_);

    for(int i = 0; i < 6; i++)
    {
      //axis map
      stringstream ss1;
      ss1 << "axis" << i;
      std::string value;
      nh_private_.getParam(ss1.str(), value);

      stringstream ss2;
      ss2 << "axis" << i << "_flip";
      nh_private_.getParam(ss2.str(), axis_flip[i]);


      if(value.at(0) == 'A')
      {
        control_type_[i] = AXIS;
        sscanf(value.c_str(), "A_%d", &axis_map_[i]);
        ROS_INFO("Mapped axis %d to axis %d flipped %d", i, axis_map_[i], axis_flip[i]);
      }
      else if(value.at(0) == 'B')
      {
        control_type_[i] = BUTTONS;
        sscanf(value.c_str(), "B_%d_%d", &button_map_[i].first, &button_map_[i].second);
        ROS_INFO("Mapped axis %d to buttons %d and %d flipped %d", i, button_map_[i].first, button_map_[i].second, axis_flip[i]);
      }
      else
      {
        ROS_FATAL("Invalid axis mapping value");
        ROS_BREAK();
      }
    }

    pose_pub_ = nh_.advertise<geometry_msgs::Pose>("command", 1, false);


  }

  void joyCallback(const sensor_msgs::JoyConstPtr& joy)
  {
    //ROS_INFO("Got a joy msg");
    //if(joy->axes.size() < 6)
    //{
      //ROS_WARN_THROTTLE(1, "need a joy stick with 6 axes for controlling all joints");
    //}
    if(dead_man_button_ >= (int)joy->buttons.size())
    {
      ROS_ERROR_THROTTLE(1.0, "Not enough buttons on the joy stick");
      return;
    }

    if(joy->buttons[dead_man_button_] != 1)
    {
      ROS_WARN_THROTTLE(1.0, "Press dead man button (%d) to enable", dead_man_button_);
      return;
    }
    bool updated = false;
    for(int i = 0; i < 6; i++)
    {
      if(control_type_[i] == AXIS)
      {
        //ROS_INFO("%d %d %f", i, axis_map_[i], joy->axes[axis_map_[i]]);
        if(fabs(joy->axes[axis_map_[i]]) > 0.5)
        {
          if(i < 3)
            results_[i] = joy->axes[axis_map_[i]] > 0 ? move_rate_ : -move_rate_;
          else
            results_[i] = joy->axes[axis_map_[i]] > 0 ? rotate_rate_ : -rotate_rate_;
          updated = true;
        }
        else
        {
          results_[i] = 0;
        }
      }
      else if(control_type_[i] == BUTTONS)
      {
        //ROS_INFO("%d %d %d %d %d", i, button_map_[i].first, button_map_[i].second, joy->buttons[button_map_[i].first], joy->buttons[button_map_[i].second]);
        if(joy->buttons[button_map_[i].first])
        {
          results_[i] = (i < 3) ? move_rate_ : rotate_rate_;
          updated = true;
        }
        else if(joy->buttons[button_map_[i].second])
        {
          results_[i] = (i < 3) ? -move_rate_ : -rotate_rate_;
          updated = true;
        }
        else
        {
          results_[i] = 0;
        }
      }
      if(axis_flip[i]) results_[i] = -results_[i];
    }

    if(updated)
    {
      //ROS_INFO("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f", results_[0], results_[1], results_[2], results_[3], results_[4],results_[5]);
      tf::Quaternion quat;
      quat.setRPY(results_[3], results_[4],results_[5]);
      geometry_msgs::Pose msg;
      msg.position.x = results_[0];
      msg.position.y = results_[1];
      msg.position.z = results_[2];
      msg.orientation.x = quat.x();
      msg.orientation.y = quat.y();
      msg.orientation.z = quat.z();
      msg.orientation.w = quat.w();
      pose_pub_.publish(msg);
      ROS_INFO_STREAM(msg);
    }



  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  double move_rate_;
  double rotate_rate_;
  int dead_man_button_;

  ControlType control_type_[6];
  bool axis_flip[6];
  int axis_map_[6];
  std::pair<int, int> button_map_[6];
  double results_[6];

  //int axis_map_[6];
  //ros::Publisher joint_pub_[6];
  ros::Publisher pose_pub_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_endeffector_joy");

  TeleopEndeffectorJoy teleop_endeffector_joy;

  ros::spin();

}
