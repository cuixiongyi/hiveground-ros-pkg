/*
 * Copyright (c) 2013, HiveGround Co., Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the HiveGround Co., Ltd., nor the name of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mahisorn Wongphati
 *
 */


#ifndef HG_JOINT_POSITION_ACTION_CONTROLLER_H_
#define HG_JOINT_POSITION_ACTION_CONTROLLER_H_

#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <hg_controller_manager/hg_controller.h>
#include <hg_controller_manager/hg_joint_command_interface.h>
#include <hg_realtime_tools/realtime_publisher.h>
#include <hg_realtime_tools/realtime_box.h>

namespace hg_common_controller
{

template <class Action>
class RTServerGoalHandle
{
private:
  ACTION_DEFINITION(Action);

  //typedef actionlib::ActionServer<Action>::GoalHandle GoalHandle;
  typedef actionlib::ServerGoalHandle<Action> GoalHandle;
  typedef boost::shared_ptr<Result> ResultPtr;

  uint8_t state_;

  bool req_abort_;
  bool req_succeed_;
  ResultConstPtr req_result_;

public:
  GoalHandle gh_;
  ResultPtr preallocated_result_;  // Preallocated so it can be used in realtime

  RTServerGoalHandle(GoalHandle &gh, const ResultPtr &preallocated_result = ResultPtr((Result*)NULL))
    : req_abort_(false), req_succeed_(false), gh_(gh), preallocated_result_(preallocated_result)
  {
    if (!preallocated_result_)
      preallocated_result_.reset(new Result);
  }

  void setAborted(ResultConstPtr result = ResultConstPtr((Result*)NULL))
  {
    if (!req_succeed_ && !req_abort_)
    {
      req_result_ = result;
      req_abort_ = true;
    }
  }

  void setSucceeded(ResultConstPtr result = ResultConstPtr((Result*)NULL))
  {
    if (!req_succeed_ && !req_abort_)
    {
      req_result_ = result;
      req_succeed_ = true;
    }
  }

  bool valid()
  {
    return gh_.getGoal() != NULL;
  }

  void runNonRT(const ros::TimerEvent &te)
  {
    using namespace actionlib_msgs;
    if (valid())
    {
      actionlib_msgs::GoalStatus gs = gh_.getGoalStatus();
      if (req_abort_ && gs.status == GoalStatus::ACTIVE)
      {
        if (req_result_)
          gh_.setAborted(*req_result_);
        else
          gh_.setAborted();
      }
      else if (req_succeed_ && gs.status == GoalStatus::ACTIVE)
      {
        if (req_result_)
          gh_.setSucceeded(*req_result_);
        else
          gh_.setSucceeded();
      }
    }
  }
};

class JointTolerance
{
public:
  JointTolerance(double _position = 0, double _velocity = 0, double _acceleration = 0) :
      position(_position), velocity(_velocity), acceleration(_acceleration)
  {
  }

  bool violated(double p_err, double v_err = 0, double a_err = 0) const
  {
    return (position > 0 && fabs(p_err) > position) || (velocity > 0 && fabs(v_err) > velocity)
        || (acceleration > 0 && fabs(a_err) > acceleration);
  }

  double position;
  double velocity;
  double acceleration;
};

class JointPositionActionController : public hg_controller_manager::Controller<hg_controller_manager::PositionJointInterface>
{
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> FJTAS;
  typedef FJTAS::GoalHandle GoalHandleFollow;
  typedef RTServerGoalHandle<control_msgs::FollowJointTrajectoryAction> RTGoalHandleFollow;


public:

  bool init(hg_controller_manager::PositionJointInterface* hw, ros::NodeHandle &n);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  void stopping(const ros::Time& time);

private:
  ros::NodeHandle nh_;
  size_t joint_count_;
  std::vector<hg_controller_manager::JointHandle> joint_position_command_;
  ros::Time last_time_;
  boost::scoped_ptr<FJTAS> action_server_follow_;
  ros::Timer goal_handle_timer_;
  boost::shared_ptr<RTGoalHandleFollow> rt_active_goal_follow_;
  //std::vector<JointTolerance> default_trajectory_tolerance_;
  //std::vector<JointTolerance> default_goal_tolerance_;
  //double default_goal_time_constraint_;



  void goalCBFollow(GoalHandleFollow gh);
  void cancelCBFollow(GoalHandleFollow gh);
  void preemptActiveGoal();
  void commandTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &msg,
                         boost::shared_ptr<RTGoalHandleFollow> gh_follow = boost::shared_ptr<RTGoalHandleFollow>((RTGoalHandleFollow*)NULL));
  static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b);

  // coef[0] + coef[1]*t + ... + coef[5]*t^5
  struct Spline
  {
    std::vector<double> coef;

    Spline() :
        coef(6, 0.0)
    { }
  };

  struct Segment
  {
    double start_time;
    double duration;
    std::vector<Spline> splines;

    std::vector<JointTolerance> trajectory_tolerance;
    std::vector<JointTolerance> goal_tolerance;
    double goal_time_tolerance;

    boost::shared_ptr<RTGoalHandleFollow> gh_follow; // Goal handle for the newer FollowJointTrajectory action
  };
  typedef std::vector<Segment> SpecifiedTrajectory;


  hg_realtime_tools::RealtimeBox<boost::shared_ptr<const SpecifiedTrajectory> > current_trajectory_box_;

  std::vector<double> q, qd, qdd;  // Preallocated in init
  // Samples, but handling time bounds.  When the time is past the end
  // of the spline duration, the position is the last valid position,
  // and the derivatives are all 0.
  static void sampleSplineWithTimeBounds(const std::vector<double>& coefficients, double duration, double time,
                                         double& position, double& velocity, double& acceleration);


};



}



#endif /* HG_JOINT_POSITION_ACTION_CONTROLLER_H_ */
