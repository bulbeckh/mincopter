
/* This header is included by each translation unit that needs to access the controller. 
 *
 * This allows the desired controller to be defined in the CMakeLists or other configuration file
 */

#pragma once

#ifdef CONTROLLER_MPC
	#include "controller_mpc.h"
#elif CONTROLLER_PID
	#include "controller_pid.h"
#elif CONTROLLER_LQR
	#include "controller_lqr.h"
#elif CONTROLLER_CSC
	#include "controller_csc.h"
#elif CONTROLLER_NONE
	#include "controller_none.h"
/* 
 * Add remaining controller implementations here..
 */
#else
	#error No CONTROLLER implementation selected 
#endif
