
#pragma once

/* For the other classes like mcstate, mcinstance, and controller/planner, the header files (like control.h) will
 * add an extern reference to the correct object. Here however, we need to instantiate these objects in MC_State
 * and so will instead return a macro that expands to the desired AHRS and INAV classes. */

#ifdef MC_AHRS_DCM
	#error "AHRS DCM no longer complies to interface. Do not use"
	// HASH include "ahrs_dcm.h"
	// HASH define MC_AHRS_CLASS AP_AHRS_DCM
#elif MC_AHRS_SIM
	#include "ahrs_sim.h"
	#define MC_AHRS_CLASS AHRS_sim
	extern AHRS_sim ahrs_obj;
#elif MC_AHRS_EKF
	#include "ekf.h"
	#define MC_AHRS_CLASS EKF
	extern EKF ahrs_obj;
#elif MC_AHRS_NONE
	#include "ahrs_none.h"
	#define MC_AHRS_CLASS AHRS_None
	extern AHRS_None ahrs_obj;
#elif MC_AHRS_COMPLEMENTARY
	#include "ahrs_complementary.h"
	#define MC_AHRS_CLASS AHRS_Complementary
	extern AHRS_Complementary ahrs_obj;
/* Add additional AHRS here
 *
 */
#else
	#error "No AHRS implementation selected"
#endif




