
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
		// Implemented controller run methods
		void run_position(void) override;
		void run_xy_position_z_velocity(void) override;
		void run_roll_pitch_z_velocity(void) override;

		void reset(void) override;

	private:

		/* @brief Counter to ensure outer runs at 20Hz */
		uint32_t csc_counter;

		/* @brief PID Rate controllers */
		AC_PID rate_roll;
		AC_PID rate_pitch;
		AC_PID rate_yaw;

		AC_PID error_roll;
		AC_PID error_pitch;
		AC_PID error_yaw;

		AC_PID pos_throttle;
		AC_PID vel_throttle;

		AC_PID nav_x_pos;
		AC_PID nav_y_pos;

		AC_PID nav_x_vel;
		AC_PID nav_y_vel;


	private:
		void csc_run_roll_pitch(void);
		void csc_run_yaw(void);

		void csc_run_rp_velocity(void);
		void csc_run_yaw_velocity(void);

		void csc_run_xy_position(void);
		void csc_run_z_position(void);

		void csc_run_xy_velocity(void);
		void csc_run_z_velocity(void);

};

