
#pragma once

#ifdef MC_INAV_DEFAULT
	#include "inav_default.h"
	#define MC_INAV_CLASS AP_InertialNav
#elif MC_INAV_SIM
	#include "inav_sim.h"
	#define MC_INAV_CLASS MC_InertialNav_Sim
/* Add remaining implementations here */
#else
	#error No INAV implementation selected
#endif



