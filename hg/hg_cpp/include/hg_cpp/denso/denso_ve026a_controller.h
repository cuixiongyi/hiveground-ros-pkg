#ifndef _hg_denso_ve026a_controller_h_
#define _hg_denso_ve026a_controller_h_

#include <hg_cpp/controller.h>
#include <hg_cpp/denso/denso_bcap_serial.h>

namespace hg
{

class HgROS;

class DensoVe026a_BCapController : public Controller
{
public:
	DensoVe026a_BCapController(hg::HgROS* hg_ros, const std::string& name);
	~DensoVe026a_BCapController();

	void startup();
	void update();
	void shutdown();
	bool active();


	ros::NodeHandle n_;

	uint32_t b_cap_controller_;
	uint32_t b_cap_robot_;

	std::string serial_port_;
	int baud_rate_;
	int data_bits_;
	int stop_bits_;
	int parity_;


	BCapSerial bcap_serial_;
};


}



#endif

