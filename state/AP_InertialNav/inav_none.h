
#pragma once

#include "inav_interface.h"

class InertialNav_None : public MC_InertialNav
{
	public:
    	InertialNav_None(void) : MC_InertialNav() { }

		/* @brief Initialise the inertial navigation */
		void _inav_init_internal(void) override;

		/* @brief Update the inertial navigation */
		void inav_update(void) override;

};


