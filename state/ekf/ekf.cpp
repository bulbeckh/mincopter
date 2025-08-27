

#include "ekf.h"

#include "ekf_predict.h"
#include "ekf_correct.h"


void EKF::update(void)
{
	// This is called from the main loop at ~100Hz
	
	// Run prediction step (last three args are real/int workspace sizes and memory index, which are all 0)
	int _result = ekf_predict(&ekf_predict_arg, &ekf_predict_res, 0, 0, 0);

	// Run correction step
	int _result = ekf_correct(&ekf_correct_arg, &ekf_correct_res, 0, 0, 0);

	return;
}


/* IMPLEMENT REMAINING FUNCTIONS HERE */



