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

namespace hg
{

class TrapezoidalVelocityControl
{
public:
  double target, current;
  double max_vel;
  double max_acc;

  TrapezoidalVelocityControl() :
      target(0), current(0), max_vel(0), max_acc(0.0)
  {
  }

  TrapezoidalVelocityControl(double v, double a) :
      target(0), current(0), max_vel(v), max_acc(a)
  {
  }

  void updateControlLimit(double a, double v)
  {
    max_acc = a;
    max_vel = v;
  }

  void updateMaxAcceleration(double a)
  {
    max_acc = a;
  }

  void updateMaxVelopcity(double a)
  {
    max_acc = a;
  }

  void updateTargetVelocity(double v)
  {
    target = v;
  }

  void reset()
  {
    target = 0;
    current = 0;
  }

  double getNextVelocity()
  {
    double tmp;
    if (target > current)
      tmp = current + max_acc;
    else if (target < current)
      tmp = current - max_acc;
    else
      tmp = current;

    if (fabs(tmp) > max_vel)
    {
      if (tmp > 0)
        tmp = max_vel;
      else
        tmp = -max_vel;
    }
    current = tmp;
    return current;
  }

  double getVelocityError(double real)
  {
    return current - real;
  }

  ///Support for output stream operator
  friend std::ostream& operator <<(std::ostream& os, const TrapezoidalVelocityControl& v)
  {
    return os << v.max_vel << " " << v.max_acc << " " << v.target << " " << v.current;
  }
};

}
