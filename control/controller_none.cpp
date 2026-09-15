
#include "controller_none.h"

None_Controller::None_Controller(MCInstance& mc, MCState& mcs) : MC_Controller(mc, mcs)
{

}

void None_Controller::reset(void)
{

}


void None_Controller::run_none_controller(void)
{
	// Write min PWM signals to all motors
	mincopter.hal.rcout->write(0, 1000);
	mincopter.hal.rcout->write(1, 1000);
	mincopter.hal.rcout->write(2, 1000);
	mincopter.hal.rcout->write(3, 1000);

	return;
}
