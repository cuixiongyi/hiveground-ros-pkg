#include <hg_cpp/denso/denso_ve026a_controller.h>

using namespace hg;

DensoVe026a_BCapController::DensoVe026a_BCapController(hg::HgROS* hg_ros, const std::string& name)
		: Controller(hg_ros, name)
{
	node_handle_.param<std::string>("SerialPort", serial_port_, "/dev/ttyUSB0");
	node_handle_.param<int>("BaudRate", baud_rate_, 115200);
	node_handle_.param<int>("DataBits", data_bits_, 8);
	node_handle_.param<int>("StopBits", stop_bits_, 1);
	node_handle_.param<int>("Parity", parity_, SerialPort::PARITY_NONE);

	ROS_INFO("SerialPort=%s, BaudRate=%d, DataBits=%d, StopBits=%d, Parity=%d",
		serial_port_.c_str(), baud_rate_, data_bits_, stop_bits_, parity_);

}

DensoVe026a_BCapController::~DensoVe026a_BCapController()
{

}

void DensoVe026a_BCapController::startup()
{

}

void DensoVe026a_BCapController::update()
{

}

void DensoVe026a_BCapController::shutdown()
{

}

bool DensoVe026a_BCapController::active()
{
	return false;
}

