/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Imai Laboratory, Keio University.
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
 *      * Neither the name of the Imai Laboratory, nor the name of its
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
 */

#include <iostream>
#include <boost/thread.hpp>

namespace hg
{

class PID
{
public:
  double p_err, d_err, i_err;
  double kp, ki, kd, il;
  boost::mutex mutex;

  PID() :
      p_err(0), d_err(0), i_err(0), kp(0), ki(0), kd(0), il(0)
  {
  }

  PID(double p, double i, double d, double l) :
      p_err(0), d_err(0), i_err(0), kp(p), ki(i), kd(d), il(l)
  {
  }

  PID(const PID& pid) :
      kp(pid.kp), ki(pid.ki), kd(pid.kd), il(pid.il)
  {
  }

  inline void updateGain(double p, double i, double d, double l)
  {
    kp = p;
    ki = i;
    kd = d;
    il = l;
    resetError();
  }

  inline void updateP(double p)
  {
    boost::mutex::scoped_lock(mutex);
    kp = p;
    resetError();
  }

  inline void updateI(double i)
  {
    boost::mutex::scoped_lock(mutex);
    ki = i;
    resetError();
  }

  inline void updateD(double d)
  {
    boost::mutex::scoped_lock(mutex);
    kd = d;
    resetError();
  }

  inline void updateIL(double l)
  {
    boost::mutex::scoped_lock(mutex);
    il = l;
    resetError();
  }

  inline void resetError()
  {
    p_err = 0;
    d_err = 0;
    i_err = 0;
  }

  inline double update(double err)
  {
    boost::mutex::scoped_lock(mutex);
    d_err = err - p_err;
    p_err = err;
    i_err += d_err;

    if (fabs(i_err) > il)
    {
      if (i_err > 0)
        i_err = il;
      else
        i_err = -il;
    }
    return (kp * p_err) + (kd * d_err) + (ki * i_err);
  }

  ///Support for output stream operator
  friend std::ostream& operator <<(std::ostream& os, const PID& p)
  {
    return os << p.kp << " " << p.ki << " " << p.kd << " " << p.il << " " << p.p_err << " " << p.i_err << " " << p.d_err;
  }
};

}
