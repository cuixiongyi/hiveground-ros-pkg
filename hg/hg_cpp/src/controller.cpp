#include <hg_cpp/controller.h>
#include <hg_cpp/hg_ros.h>
using namespace hg;

Controller::Controller(hg::HgROS* hg_ros, const std::string& name)
	: name_(name),
	  simulate_(hg_ros->simulate_),
	  pause_(false),
	  hg_ros_(hg_ros),
	  node_handle_(hg_ros->node_handle_)
{ }
