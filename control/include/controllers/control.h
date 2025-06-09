
/* This header is included by each translation unit that needs to access the controller. 
 *
 * This allows the desired controller to be defined in the CMakeLists or other configuration file
 */

#pragma once

#ifdef CONTROLLER_MPC
	#include "controller_mpc.h"
	extern MPC_Controller controller;
#elif CONTROLLER_PID
	#include "controller_pid.h"
	extern PID_Controller controller;
/* 
 * Add remaining controller implementations here..
 */
#else
	#error No CONTROLLER implementation selected 
#endif
