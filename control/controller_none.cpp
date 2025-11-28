
#include "controller_none.h"

#include "mcinstance.h"
extern MCInstance mincopter;

// Instance
None_Controller controller;

void None_Controller::run()
{
	// Write min PWM signals to all motors
	mincopter.hal.rcout->write(0, 1000);
	mincopter.hal.rcout->write(1, 1000);
	mincopter.hal.rcout->write(2, 1000);
	mincopter.hal.rcout->write(3, 1000);

	return;
}
