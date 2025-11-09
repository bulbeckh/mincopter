
#pragma once

#include "inav_interface.h"

class MC_InertialNav_Sim : public MC_InertialNav
{
	public:
    	MC_InertialNav_Sim(void) : MC_InertialNav() { }

		/* @brief Initialise the inertial navigation */
		void _inav_init_internal(void) override;

		/* @brief Update the inertial navigation */
		void inav_update(void) override;

};


