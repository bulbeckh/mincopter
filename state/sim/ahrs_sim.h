
#pragma once

#include "ahrs_interface.h"

/* Simulated AHRS with state obtained directly from the simulation */

class AHRS_sim : public AP_AHRS
{
	public:
		AHRS_sim(void) : AP_AHRS() { }

		/* @brief Sim AHRS update method */
		void ahrs_update(void) override;

		/* @brief Sim internal init method */
		void _ahrs_init_internal(void) override;

		// NOTE We can keep the default ahrs reset methods as the sim basically
		// just retrieves the orientation directly from gazebo 

};


