

#ifdef MC_STATE_COMPLEMENTARY
	#include "complementary/state_complementary.h"
	extern StateComplementary mcstate;
#elif MC_STATE_SIM
	#include "sim/state_sim.h"
	extern StateSim mcstate;
#elif MC_STATE_NONE
	#include "none/state_none.h"
	extern StateNone mcstate;
#elif MC_STATE_EKF
	#include "ekf/state_ekf.h"
	extern StateEKF mcstate;
#elif MC_STATE_MADGWICK
	#include "madgwick/state_madgwick.h"
	extern StateMadgwick mcstate;
#endif


