
#pragma once

/* For the other classes like mcstate, mcinstance, and controller/planner, the header files (like control.h) will
 * add an extern reference to the correct object. Here however, we need to instantiate these objects in MC_State
 * and so will instead return a macro that expands to the desired AHRS and INAV classes. */

#ifdef MC_AHRS_DCM
	#include "ahrs_dcm.h"
	#define MC_AHRS_CLASS AP_AHRS_DCM
#elif MC_AHRS_SIM
	#include "ahrs_sim.h"
	#define MC_AHRS_CLASS AHRS_sim
/* Add additional AHRS here
 *
 */
#else
	#error No AHRS implementation selected
#endif




