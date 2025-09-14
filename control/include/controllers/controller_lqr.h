
#pragma once

#include <stdint.h>

#include "controller_interface.h"

#include "mcinstance.h"
#include "mcstate.h"


/* LQR Controller as per state-space model in control/design/src/lqr
 *
 * Needs to take the current state and calculate the output vector [Total Force, Roll Torque, Pitch Torque, Yaw Torque]
 * and account for the target.
 *
 *   U = -k*(state - ref)
 *
 * xx
 *
 */
class LQR_Controller : public MC_Controller
{
	public:

		LQR_Controller() : MC_Controller() { }


	public:
		void run(void) override;
		

};

