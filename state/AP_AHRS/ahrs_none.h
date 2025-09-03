
#pragma once

#include "ahrs_interface.h"

/* AHRS_None class for unit testing of other modules and minimising size when testing memory layouts */

class AHRS_None : public AP_AHRS
{
	public:
		AHRS_None(void) : AP_AHRS() { }

		/* @brief AHRS update method */
		void ahrs_update(void) override;

		/* @brief internal init method */
		void _ahrs_init_internal(void) override;

};


