/*
 * Software License Agreement (BSD License)
 *
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
 * Code modify/copied from spline_smoothers/cubic_trajectory.cpp
 */

#ifndef HG_TRAJECTORY_H_
#define HG_TRAJECTORY_H_

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <arm_navigation_msgs/JointLimits.h>
#include <spline_smoother/spline_smoother_utils.h>
#include <spline_smoother/cubic_trajectory.h>

namespace hg_trajectory
{


inline bool checkTrajectoryConsistency(trajectory_msgs::JointTrajectory& in)
{
  unsigned int length = in.points.size();
  unsigned int num_joints = in.joint_names.size();

  double prev_time = -1.0;

  for (unsigned int i = 0; i < length; i++)
  {
    if (in.points[i].positions.size() != num_joints)
    {
      ROS_ERROR(
          "Number of positions (%d) at trajectory index %d doesn't match number of joint names (%d)", (int) in.points[i].positions.size(), (int) i, (int) num_joints);
      return false;
    }
    if (in.points[i].time_from_start.toSec() < prev_time)
    {
      ROS_ERROR(
          "Time of waypoint at trajectory index %d (%f) is not greater than or equal to the previous time (%f)", (int) i, in.points[i].time_from_start.toSec(), prev_time);
      return false;
    }
    if (in.points[i].time_from_start.toSec() < 0.0)
    {
      ROS_ERROR(
          "Time of waypoint at trajectory index %d (%f) is negative", (int) i, in.points[i].time_from_start.toSec());
      return false;
    }
    prev_time = in.points[i].time_from_start.toSec();
    if (in.points[i].velocities.size()
        != in.points[i].positions.size())
      in.points[i].velocities.resize(num_joints, 0.0);
    if (in.points[i].accelerations.size()
        != in.points[i].positions.size())
      in.points[i].accelerations.resize(num_joints, 0.0);
  }
  return true;
}

inline bool getWaypoints(const spline_smoother::SplineTrajectory &spline,
                         trajectory_msgs::JointTrajectory &joint_trajectory)
{
  std::vector<double> waypoint_times_vector;
  double waypoint_time = 0.0;
  waypoint_times_vector.push_back(waypoint_time);
  for(unsigned int i=0; i < spline.segments.size(); i++)
  {
    waypoint_time = waypoint_time + spline.segments[i].duration.toSec();
    waypoint_times_vector.push_back(waypoint_time);
    ROS_DEBUG("Spline segment time: %f",spline.segments[i].duration.toSec());
  }
  if(!spline_smoother::sampleSplineTrajectory(spline,waypoint_times_vector,joint_trajectory))
    return false;
  return true;
}

inline double maxLInfDistance(const trajectory_msgs::JointTrajectoryPoint &start,
                              const trajectory_msgs::JointTrajectoryPoint &end)
{
  double max_diff = 0.0;
  for(unsigned int i=0; i< start.positions.size(); i++)
  {
    double diff = fabs(end.positions[i]-start.positions[i]);
    if(diff > max_diff)
      max_diff = diff;
  }
  return max_diff;
}

inline void discretizeAndAppendSegment(const spline_smoother::SplineTrajectorySegment &spline_segment,
                                       const double &discretization,
                                       trajectory_msgs::JointTrajectory &joint_trajectory,
                                       const ros::Duration &segment_start_time,
                                       const bool &include_segment_end)
{
  ros::Duration time_from_start = segment_start_time;
  double total_time = spline_segment.duration.toSec();
  double sample_time = 0.0;
  trajectory_msgs::JointTrajectoryPoint start,end;
  spline_smoother::sampleSplineTrajectory(spline_segment,0.0,start);
  if(joint_trajectory.points.empty())
  {
    start.time_from_start = ros::Duration(0.0);
    joint_trajectory.points.push_back(start);
    sample_time += 0.01;
  }
  start = joint_trajectory.points.back();
  while(sample_time < total_time)
  {
     ROS_DEBUG("Sample time is %f",sample_time);
     spline_smoother::sampleSplineTrajectory(spline_segment,sample_time,end);
     double max_diff = maxLInfDistance(start,end);
     if(sample_time > 0 && max_diff < discretization)
     {
       ROS_DEBUG("Max diff is %f. Skipping",max_diff);
       sample_time += 0.01;
       continue;
     }
     end.time_from_start = time_from_start + ros::Duration(sample_time);
     joint_trajectory.points.push_back(end);
     ROS_DEBUG("Pushing back point with time: %f",end.time_from_start.toSec());
     sample_time += 0.01;
     start = end;
  }
  if(include_segment_end)
  {
    spline_smoother::sampleSplineTrajectory(spline_segment,total_time,end);
    end.time_from_start = time_from_start + ros::Duration(total_time);
    joint_trajectory.points.push_back(end);
  }
}


inline void discretizeTrajectory(const spline_smoother::SplineTrajectory &spline,
                                 const double &discretization,
                                 trajectory_msgs::JointTrajectory &joint_trajectory)
{
  if(spline.segments.empty())
    return;
  joint_trajectory.points.clear();
  ros::Duration segment_start_time(0.0);
  for(unsigned int i=0; i < spline.segments.size(); i++)
  {
    if(i == spline.segments.size()-1)
      discretizeAndAppendSegment(spline.segments[i],discretization,joint_trajectory,segment_start_time,true);
    else
      discretizeAndAppendSegment(spline.segments[i],discretization,joint_trajectory,segment_start_time,false);
    segment_start_time += spline.segments[i].duration;
    ROS_DEBUG("Discretizing and appending segment %d",i);
  }
}


inline void refineTrajectory(trajectory_msgs::JointTrajectory &trajectory,
                             const std::vector<arm_navigation_msgs::JointLimits>& limits)
{
  if(trajectory.points.size() < 3)
    return;

  for(unsigned int i=1; i < trajectory.points.size()-1; i++)
  {
    for(unsigned int j=0; j < trajectory.points[i].positions.size(); j++)
    {
      double dq_first = trajectory.points[i].positions[j] - trajectory.points[i-1].positions[j];
      double dq_second = trajectory.points[i+1].positions[j] - trajectory.points[i].positions[j];
      double dt_first = (trajectory.points[i].time_from_start - trajectory.points[i-1].time_from_start).toSec();
      double dt_second = (trajectory.points[i+1].time_from_start - trajectory.points[i].time_from_start).toSec();
      if( (dq_first > 0 && dq_second > 0) || (dq_first < 0 && dq_second < 0))
      {
        trajectory.points[i].velocities[j] = 0.5*(dq_first/dt_first + dq_second/dt_second);
        trajectory.points[i].velocities[j] = std::max(std::min(trajectory.points[i].velocities[j], limits[j].max_velocity), -limits[j].max_velocity);
      }
    }
  }
}


inline bool cubicSmoothWaypointsTrajectory(const trajectory_msgs::JointTrajectory& in,
                                           const std::vector<arm_navigation_msgs::JointLimits>& limits,
                                           trajectory_msgs::JointTrajectory& out,
                                           const double& discretization = 0.01)
{
  out = in;
  if (!checkTrajectoryConsistency(out))
    return false;

  spline_smoother::SplineTrajectory spline;
  spline_smoother::CubicTrajectory trajectory_solver;

  if(!trajectory_solver.parameterize(out, limits, spline))
  {
    return false;
  }

  if(!getWaypoints(spline, out))
  {
    return false;
  }

  refineTrajectory(out, limits);
  //ROS_INFO("======Input=====");
  //ROS_INFO_STREAM(in);
  //ROS_INFO("======Before=====");
  //ROS_INFO_STREAM(out);
  //ROS_INFO("======After=====");
  //ROS_INFO_STREAM(out);
  discretizeTrajectory(spline, discretization, out);



  for(size_t i = 0; i < out.points.size(); i++)
  {
    out.points[i].accelerations.clear();
  }

  if (!trajectory_solver.parameterize(out, limits, spline))
  {
    return false;
  }

  if(!getWaypoints(spline, out))
  {
    return false;
  }

  return true;
}



}


#endif /* HG_TARJECTORY_H_ */
