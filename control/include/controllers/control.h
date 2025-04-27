
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
};

