
#pragma once

#include "mcinstance.h"
#include "mcstate.h"

class MC_Controller {

	public:
		/* @brief Top-level abstraction for a MinCopter controller
		 * @param mcstate
		 * @param mcinstance
		 */
		MC_Controller()
		{
		}

		// TODO Add destructor

	public:

		/* @brief Runs controller taking in the state and instance objects and updating control output
		 * @param
		 */
		virtual void run() = 0;

		// Controller State Variables

		/* @brief Desired roll/pitch/yaw values for determination by planner and usage by controller
		 */
		int16_t control_roll;
		int16_t control_pitch;
		int32_t control_yaw;

		// TODO Add in the remaining 9 controller state variables

};

