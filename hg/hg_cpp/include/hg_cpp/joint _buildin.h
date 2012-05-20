#ifndef _hg_joints_buildin_h_
#define _hg_joints_buildin_h_

#include <std_msgs/Float32.h>

#include <hg_cpp/joint.h>
#include <hg_cpp/controller.h>

namespace hg
{

class JointBuildin : public Joint
{
public:
	JointBuildin(hg::HgROS* hg_ros, const std::string& name)
		: Joint(hg_ros, name),
		  touched_(false),
		  motor_off_(true),
		  desired_position_(0),
		  last_command_(0),
		  speed_(0)
	{

		joint_command_ = node_handle_.subscribe(name_ + "/command", 1,
				&JointBuildin::joint_command_callback, this);

		ROS_INFO_STREAM(name_ << " lower: " << lower_ << " uppper: "
				<< upper_ << " velocity: " << velocity_limit_);
	}

	~JointBuildin()
	{

	}

	double interpolate(double dt)
	{
		if(touched_)
		{
			//ROS_INFO_STREAM("desired_position " << desired_position_);
			//ROS_INFO_STREAM("last_command " << last_command_);
			//ROS_INFO_STREAM("command limit " << (velocity_limit_*dt));
			//compute command, limit velocity (rad/s)
			double command = desired_position_ - last_command_;
			if(command > (velocity_limit_*dt))
			{
				command = (velocity_limit_*dt);
				//ROS_WARN_STREAM(name_ + " is reached velocity limit (+)");
			}
			else if(command < -(velocity_limit_*dt))
			{
				command = -(velocity_limit_*dt);
				//ROS_WARN_STREAM(name_ + " is reached velocity limit (-)");
			}

			//ROS_INFO_STREAM("command1 " << command);

			//compute angle, apply limit (lower, upper)
			command = last_command_ + command;
			if(command > upper_)
			{
				command = upper_;
				//ROS_WARN_STREAM(name_ + " is reached upper limit");
			}
			else if(command < lower_)
			{
				command = lower_;
				//ROS_WARN_STREAM(name_ + " is reached lower limit");
			}

			//ROS_INFO_STREAM("command2 " << command);

			//store last command
			last_command_ = command;

			speed_ = command/dt;

			//ROS_INFO_STREAM("speed " << speed_);

			//reach target
			if(last_command_ == desired_position_)
			{
				//ROS_INFO_STREAM("last_command_ " << last_command_);
				touched_ = false;
			}

			//is simulated?
			if(hg_ros_->simulate_)
			{
				position_ = last_command_;
			}

			//ROS_INFO_STREAM("command " << command);
			return command;
		} else {
			return desired_position_; //do not move and hold position
		}
	}

	void set_feedback_data(double feedback)
	{
		double last_position = position_;
		position_ = feedback;
		ros::Time t = ros::Time::now();
		velocity_ = ((position_ - last_position) * 1.0e9)/
				(t - last_update_).toNSec();
		last_update_ = t;


		if(motor_off_)
			last_command_ = feedback;
		//ROS_INFO_STREAM("fb velocity " << velocity_);
		//ROS_INFO_STREAM("fb last_position " << last_position);
		//ROS_INFO_STREAM("fb feedback " << feedback);
	}

	double set_position(double position)
	{
		touched_ = true;
		motor_off_ = false;
		desired_position_ = position;
		return position;
	}

	void joint_command_callback(const std_msgs::Float32& message)
	{
		//std::cout << message.data << std::endl;
		if(controller_)
		{
			if(controller_->active())
			{
				//Under and action control, do not interfere
				return;
			}
		}
		//ROS_INFO_STREAM("start " << message.data);
		touched_ = true;
		motor_off_ = false;
		desired_position_ = message.data;
	}



	ros::Subscriber joint_command_;

	bool touched_;
	bool motor_off_;
	double desired_position_;
	double last_command_;
	double speed_;


};


}



#endif
