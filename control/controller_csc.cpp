
#include "controller_csc.h"

#define CSC_PID_IMAX 100

// Instance
CSC_Controller controller;

CSC_Controller::CSC_Controller()
	: MC_Controller(),
	rate_roll(0.5, 0.1, 0, CSC_PID_IMAX),
	rate_pitch(0.5, 0.1, 0, CSC_PID_IMAX),
	rate_yaw(0.5, 0.1, 0, CSC_PID_IMAX),

	error_roll(5, 0.1, 0, CSC_PID_IMAX),
	error_pitch(5, 0.1, 0, CSC_PID_IMAX),
	error_yaw(5, 0.1, 0, CSC_PID_IMAX)
{

}

void CSC_Controller::run(void)
{


}


