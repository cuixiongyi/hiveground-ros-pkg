///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF, INC and Willow Garage, Inc
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Willow Garage Inc, hiDOF Inc, nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

//Modify by Mahisorn Wongphati HiveGround Co., Ltd. 2013-04

#ifndef HG_JOINT_COMMAND_INTERFACE_H_
#define HG_JOINT_COMMAND_INTERFACE_H_

#include <hg_controller_manager/hg_hardware_interface.h>
#include <hg_controller_manager/hg_joint_state_interface.h>

namespace hg_controller_manager
{

class JointHandle : public JointStateHandle
{
public:
  JointHandle()
    : cmd_(0)
  {
  }

  JointHandle(const JointStateHandle& js, double* cmd) :
      JointStateHandle(js), cmd_(cmd)
  {
  }
  void setCommand(double command)
  {
    *cmd_ = command;
  }

private:
  double* cmd_;
};

class JointCommandInterface : public HardwareInterface
{
public:
  virtual ~JointCommandInterface() { }

  std::vector<std::string> getJointNames() const
  {
    std::vector<std::string> out;
    out.reserve(handle_map_.size());
    for (HandleMap::const_iterator it = handle_map_.begin(); it != handle_map_.end(); ++it)
    {
      out.push_back(it->first);
    }
    return out;
  }

  void registerJoint(const JointStateHandle& js, double* cmd)
  {
    JointHandle handle(js, cmd);
    HandleMap::iterator it = handle_map_.find(js.getName());
    if (it == handle_map_.end())
      handle_map_.insert(std::make_pair(js.getName(), handle));
    else
      it->second = handle;
  }

  JointHandle getJointHandle(const std::string& name)
  {
    HandleMap::const_iterator it = handle_map_.find(name);

    if (it == handle_map_.end())
      throw ControllerManagerException("Could not find joint [" + name + "] in JointCommandInterface");

    HardwareInterface::claim(name);
    return it->second;
  }

protected:
  typedef std::map<std::string, JointHandle> HandleMap;
  HandleMap handle_map_;
};

class EffortJointInterface : public JointCommandInterface
{

};

class VelocityJointInterface : public JointCommandInterface
{

};

class PositionJointInterface : public JointCommandInterface
{

};

}

#endif /* HG_JOINT_COMMAND_INTERFACE_H_ */
