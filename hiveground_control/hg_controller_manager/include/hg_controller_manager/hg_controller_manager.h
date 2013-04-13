///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF, INC & Willow Garage, Inc
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc., Willow Garage, Inc., nor the names of its
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

/*
 * Author: Wim Meeussen
 */

#ifndef HG_CONTROLLER_MANAGER_H
#define HG_CONTROLLER_MANAGER_H

#include <hg_controller_manager/hg_controller_spec.h>
#include <pthread.h>
#include <cstdio>
#include <map>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <tinyxml.h>
#include <hg_controller_manager/hg_robot_hardware.h>
#include <hg_realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <pluginlib/class_loader.h>
#include <hg_controller_manager/ListControllerTypes.h>
#include <hg_controller_manager/ListControllers.h>
#include <hg_controller_manager/ReloadControllerLibraries.h>
#include <hg_controller_manager/LoadController.h>
#include <hg_controller_manager/UnloadController.h>
#include <hg_controller_manager/SwitchController.h>
#include <boost/thread/condition.hpp>
#include <hg_controller_manager/hg_controller_loader_interface.h>

namespace hg_controller_manager
{

class ControllerManager
{

public:
  ControllerManager(hg_controller_manager::RobotHardware *robot_hw, const ros::NodeHandle& nh = ros::NodeHandle());
  virtual ~ControllerManager();

  // Real-time functions
  void update(const ros::Time& time, const ros::Duration& period, bool reset_controllers = false);

  // Non real-time functions
  bool loadController(const std::string& name);
  bool unloadController(const std::string &name);
  bool switchController(const std::vector<std::string>& start_controllers,
                        const std::vector<std::string>& stop_controllers, const int strictness);

  // controllers_lock_ must be locked before calling
  virtual ControllerBase* getControllerByName(const std::string& name);

  void registerControllerLoader(boost::shared_ptr<ControllerLoaderInterface> controller_loader);

private:
  void getControllerNames(std::vector<std::string> &v);
  void getControllerSchedule(std::vector<size_t> &schedule);

  hg_controller_manager::RobotHardware* robot_hw_;

  ros::NodeHandle controller_node_, cm_node_;

  typedef boost::shared_ptr<ControllerLoaderInterface> LoaderPtr;
  std::list<LoaderPtr> controller_loaders_;

  // for controller switching
  std::vector<ControllerBase*> start_request_, stop_request_;
  bool please_switch_;
  int switch_strictness_;

  // controller lists
  boost::mutex controllers_lock_;
  std::vector<ControllerSpec> controllers_lists_[2];
  int current_controllers_list_, used_by_realtime_;

  // services to work with controllers
  bool listControllerTypesSrv(hg_controller_manager::ListControllerTypes::Request &req,
                              hg_controller_manager::ListControllerTypes::Response &resp);
  bool listControllersSrv(hg_controller_manager::ListControllers::Request &req,
                          hg_controller_manager::ListControllers::Response &resp);
  bool switchControllerSrv(hg_controller_manager::SwitchController::Request &req,
                           hg_controller_manager::SwitchController::Response &resp);
  bool loadControllerSrv(hg_controller_manager::LoadController::Request &req,
                         hg_controller_manager::LoadController::Response &resp);
  bool unloadControllerSrv(hg_controller_manager::UnloadController::Request &req,
                           hg_controller_manager::UnloadController::Response &resp);
  bool reloadControllerLibrariesSrv(hg_controller_manager::ReloadControllerLibraries::Request &req,
                                    hg_controller_manager::ReloadControllerLibraries::Response &resp);
  boost::mutex services_lock_;
  ros::ServiceServer srv_list_controllers_, srv_list_controller_types_, srv_load_controller_;
  ros::ServiceServer srv_unload_controller_, srv_switch_controller_, srv_reload_libraries_;
};

}
#endif
