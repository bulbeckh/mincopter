
#ifndef __CONTROL__H
#define __CONTROL__H

#include "mcinstance.h"
#include "mcstate.h"

class MC_Controller {

	public:
		/* @brief Top-level abstraction for a MinCopter controller
		 * @param mcstate
		 * @param mcinstance
		 */
		MC_Controller(
				MC_State* mcstate,
				MC_Instance* mcinstance
		) :
			state(mcstate),
			mincopter(mcinstance);

		// TODO Add destructor

	public:
			MC_State* state;
			MC_Instance* mincopter;


			/* @brief Runs controller taking in the state and instance objects and updating control output
			 * @param
			 */
			virtual void run() = 0;
}


#endif  // __CONTROL__H

