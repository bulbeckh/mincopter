
#pragma once

#include "controller_interface.h"

class None_Controller : public MC_Controller
{
	public:
		None_Controller();

		void run_position(void) override { run_none_controller(); }

		void run_xy_position_z_velocity(void) override { run_none_controller(); }

		void run_roll_pitch_z_velocity(void) override { run_none_controller(); }

		void reset(void) override;

	private:
		void run_none_controller(void);
};
