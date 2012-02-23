#ifndef _hg_denso_ve026a_controller_h_
#define _hg_denso_ve026a_controller_h_

#include <hg_cpp/controller.h>
#include <hg_cpp/joint.h>
#include <hg_cpp/denso/denso_bcap_serial.h>


#include <boost/thread.hpp>
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

	//thread function
	void control_loop();

private:
	void initialize_ve026a();
	bool set_motor(bool on_off);

	/**
	 * Set joints position of VE026a.
	 * @param joint positions in degree (float [7])
	 * @param current joint positions in degree (float [8])
	 */
	bool set_joints(float* positions, float* results);
public:

	bool is_initialized_;
	uint32_t b_cap_controller_;
	uint32_t b_cap_robot_;

	std::string port_name_;
	int baud_rate_;
	int data_bits_;
	int stop_bits_;
	int parity_;

	BCapSerial bcap_serial_;

	double control_rate_;
	bool is_running_;
	boost::thread control_thread_;
};


}



#endif

