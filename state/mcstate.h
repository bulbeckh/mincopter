

#ifdef MC_STATE_COMPLEMENTARY
	#include "complementary/state_complemenetary.h"
#elif MC_STATE_SIM
	// TODO
	#include "sim/state_sim.h"
	extern StateSim mcstate;
#elif MC_STATE_NONE
	#include "none/state_none.h"
	extern StateNone mcstate;
#elif MC_STATE_EKF
	// TODO
#elif MC_STATE_MADGWICK
	// TODO
#endif


