
#pragma once

#include <stdint.h>

#include "controller_interface.h"

#include <AC_PID.h>

#include "mcinstance.h"
#include "mcstate.h"

// TODO Eventually this mixer will follow the standard "mixer.h" and "mixer_interface.h" pattern we use for the other libraries
#include "mixer.h"

// Cascaded PID controller implementation

class CSC_Controller : public MC_Controller
{
	public:
		CSC_Controller();

	public:
		/* @brief Run controller (including call to mixer) */
		void run(void) override;

	private:
		/* @brief Controller mixer algorithm */
		Mixer mixer;

		/* @brief PID Rate controllers */
		AC_PID rate_roll;
		AC_PID rate_pitch;
		AC_PID rate_yaw;

		AC_PID error_roll;
		AC_PID error_pitch;
		AC_PID error_yaw;

};

