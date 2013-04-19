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

#ifndef HG_CUBIC_PATH_H_
#define HG_CUBIC_PATH_H_

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace hg_cubic_path
{

template <typename T>
void tridiagonalSolve(std::vector<T>& a,
                      std::vector<T>& b,
                      std::vector<T>& c,
                      std::vector<T>& d,
                      std::vector<T>& x)
{
  int n = (int)d.size();

  x.resize(n);

  // forward elimination
  for (int i=1; i<n; i++)
  {
    double m = a[i] / b[i-1];
    b[i] -= m*c[i-1];
    d[i] -= m*d[i-1];
  }

  // backward substitution
  x[n-1] = d[n-1]/b[n-1];
  for (int i=n-2; i>=0; i--)
  {
    x[i] = (d[i] - c[i]*x[i+1])/b[i];
  }
}

//Introduction to Robotics 3rd Edition Ch7 P208
inline bool clamp_path_velocity_with_tanget(const std::vector<double>& positions,
                                            const std::vector<double>& time_from_start,
                                            std::vector<double>& velocities)
{
  size_t length = positions.size();
  if(length < 3)
  {
    ROS_DEBUG("positions.size() < 3");
    return true;
  }

  // generate time intervals:
  std::vector<double> intervals(length - 1);
  for (size_t i = 0; i < length - 1; i++)
  {
    intervals[i] = time_from_start[i+1] - time_from_start[i];
    if(intervals[i] == 0)
    {
      ROS_ERROR("found a zero segment time!");
      return true;
    }
  }

  //prepare velocity
  velocities.resize(length);
  velocities[0] = 0.0;
  velocities[length-1] = 0.0;
  std::vector<double> slope(length-1);
  std::vector<int> sign(length-1);
  int sign0, sign1;

  slope[0] = (positions[1] - positions[0]) / intervals[0];
  sign[0] = (slope[0] > 0) ? 1 : ((slope[0] < 0) ? -1 : 0);
  for(size_t i = 0; i < length-2; i++)
  {
    slope[i+1] = (positions[i+2] - positions[i+1]) / intervals[i+1];
    sign[i+1] = (slope[i+1] > 0) ? 1 : ((slope[i+1]) ? -1 : 0);
    if(sign[i] != sign[i+1])
    {
      velocities[i+1] = 0;
    }
    else
    {
      velocities[i+1] = (slope[i] + slope[i+1]) / 2.0;
    }
  }
  return true;
}


inline bool clamp_path_velocity_with_tanget(trajectory_msgs::JointTrajectory& trajectory)
{
  size_t length = trajectory.points.size();
  if(length < 3)
  {
    ROS_DEBUG("trajectory.points < 3");
    return true;
  }

  // generate time intervals:
  std::vector<double> intervals(length - 1);
  for (int i = 0; i < length - 1; i++)
  {
    intervals[i] = (trajectory.points[i+1].time_from_start - trajectory.points[i].time_from_start).toSec();
    if(intervals[i] == 0)
    {
      ROS_ERROR("found a zero segment time!");
      return true;
    }
  }

  //prepare velocity
  std::vector<double> slope(length-1);
  std::vector<int> sign(length-1);
  int sign0, sign1;

  int num_joint = trajectory.joint_names.size();

  for(int j = 0; j < num_joint; j++)
  {
    slope[0] = (trajectory.points[1].positions[j] - trajectory.points[0].positions[j]) / intervals[0];
    sign[0] = (slope[0] > 0) ? 1 : ((slope[0] < 0) ? -1 : 0);
    for(size_t i = 1; i < length-1; i++)
    {
      slope[i] = (trajectory.points[i+1].positions[j] - trajectory.points[i].positions[j]) / intervals[i];
      sign[i] = (slope[i] > 0) ? 1 : ((slope[i]) ? -1 : 0);
      if(sign[i] != sign[i-1])
      {
        trajectory.points[i].velocities[j] = 0;
      }
      else
      {
        trajectory.points[i].velocities[j] = (slope[i] + slope[i-1]) / 2.0;
      }
      ROS_INFO("%d %d %f", j, i, trajectory.points[i].velocities[j]);
    }
  }
  return true;
}

inline bool quadSolve(const double &a, const double &b, const double &c, std::vector<double> &solution)
{
  double t1(0.0), t2(0.0);
  double eps = 2.2e-16;
  if (fabs(a) > eps)
  {
    double discriminant = b * b - 4 * a * c;
    if (discriminant >= 0)
    {
      t1 = (-b + sqrt(discriminant)) / (2 * a);
      t2 = (-b - sqrt(discriminant)) / (2 * a);
      ROS_DEBUG("t1:%f t2:%f",t1,t2);
      solution.push_back(t1);
      solution.push_back(t2);
      return true;
    }
    else
    {
      ROS_DEBUG("Discriminant: %f",discriminant);
      return false;
    }
  }
  else
  {
    if (fabs(b) == 0.0)
      return false;
    t1 = -c / b;
    t2 = t1;
    solution.push_back(t1);
    solution.push_back(t2);
    return true;
  }
}

inline bool validSolution(const double &q0,
                          const double &q1,
                          const double &v0,
                          const double &v1,
                          const double &dT,
                          const double &vmax,
                          const double &amax)
{
  if (dT == 0.0)
    return false;
  //  double a0 = q0;
  double a1 = v0;
  double a2 = (3*(q1-q0)-(2*v0+v1)*dT)/(dT*dT);
  double a3 = (2*(q0-q1)+(v0+v1)*dT)/(dT*dT*dT);

  double max_accn = fabs(2*a2);
  if(fabs(2*a2+6*a3*dT) > max_accn)
    max_accn = fabs(2*a2+6*a3*dT);

  bool max_vel_exists = false;
  double max_vel = 0.0;

  if(fabs(a3) > 0.0)
  {
    double max_vel_time = (-2*a2)/(6*a3);
    if (max_vel_time >= 0 && max_vel_time < dT)
    {
      max_vel_exists = true;
      max_vel = a1-(a2*a2)/(a3*3.0);
    }
  }

  if(amax > 0 && max_accn-amax > 1e-2)
  {
    ROS_DEBUG("amax allowed: %f, max_accn: %f",amax,max_accn);
    return false;
  }
  if(max_vel_exists)
    if(fabs(max_vel)-vmax > 1e-2)
    {
      ROS_DEBUG("vmax allowed: %f, max_vel: %f",vmax,max_vel);
      return false;
    }
  return true;
}


inline double minSegmentTime(const double &q0,
                             const double &q1,
                             const double &v0,
                             const double &v1,
                             const double &v_max,
                             const double &a_max = 0.0)
{
  double dq = q1-q0;
  double vmax = v_max;
  if( q0 == q1 && fabs(v0-v1) == 0.0)
  {
   return 0.0;
  }

  dq = q1 - q0;
  double v = vmax;
  double solution;
  std::vector<double> solution_vec;

  double a1 = 3 * (v0 + v1) * v - 3 * (v0 + v1) * v0 + (2 * v0 + v1) * (2 * v0 + v1);
  double b1 = -6 * dq * v + 6 * v0 * dq - 6 * dq * (2 * v0 + v1);
  double c1 = 9 * dq * dq;

  double a2 = 3 * (v0 + v1) * v + 3 * (v0 + v1) * v0 - (2 * v0 + v1) * (2 * v0 + v1);
  double b2 = -6 * dq * v - 6 * v0 * dq + 6 * dq * (2 * v0 + v1);
  double c2 = -9 * dq * dq;

  std::vector<double> t1,t2,t3,t4,t5,t6;

  if (quadSolve(a1, b1, c1, t1))
    for (unsigned int i = 0; i < t1.size(); i++)
      solution_vec.push_back(t1[i]);

  if (quadSolve(a2, b2, c2, t2))
    for (unsigned int i = 0; i < t2.size(); i++)
      solution_vec.push_back(t2[i]);
  double amax = -1.0;

  if (a_max > 0)
  {
    amax = a_max;
    double a3 = amax / 2.0;
    double b3 = 2 * v0 + v1;
    double c3 = -3 * dq;
    if (quadSolve(a3, b3, c3, t3))
      for (unsigned int i = 0; i < t3.size(); i++)
        solution_vec.push_back(t3[i]);

    double a4 = amax / 2.0;
    double b4 = -(2 * v0 + v1);
    double c4 = 3 * dq;
    if (quadSolve(a4, b4, c4, t4))
      for (unsigned int i = 0; i < t4.size(); i++)
        solution_vec.push_back(t4[i]);

    double a5 = amax;
    double b5 = (-2 * v0 - 4 * v1);
    double c5 = 6 * dq;
    if (quadSolve(a5, b5, c5, t5))
      for (unsigned int i = 0; i < t5.size(); i++)
        solution_vec.push_back(t5[i]);

    double a6 = amax;
    double b6 = (2 * v0 + 4 * v1);
    double c6 = -6 * dq;
    if (quadSolve(a6, b6, c6, t6))
      for (unsigned int i = 0; i < t6.size(); i++)
        solution_vec.push_back(t6[i]);
  }
  std::vector<double> positive_durations, valid_durations;
  for (unsigned int i = 0; i < solution_vec.size(); i++)
  {
    if (solution_vec[i] > 0)
      positive_durations.push_back(solution_vec[i]);
  }

  for (unsigned int i = 0; i < positive_durations.size(); i++)
  {
    ROS_DEBUG("Positive duration: %f", positive_durations[i]);
    if (validSolution(q0, q1, v0, v1, positive_durations[i], vmax, amax))
      valid_durations.push_back(positive_durations[i]);
  }

  ROS_DEBUG("valid size: %d", (int)valid_durations.size());
  std::sort(valid_durations.begin(), valid_durations.end());
  if (!valid_durations.empty())
    solution = valid_durations.front();
  else
    solution = 0.025;

  ROS_DEBUG(" ");
  ROS_DEBUG(" ");
  ROS_DEBUG(" ");
  return solution;
}

inline bool parameterize_cubic_path_with_limit(std::vector<double>& positions,
                                               std::vector<double>& velocities,
                                               std::vector<double>& accelerations,
                                               const std::vector<double>& v_maxs,
                                               const std::vector<double>& a_maxs)
{
  size_t num_positions = positions.size();


  return true;
}



}


#endif /* HG_CUBIC_PATH_H_ */
