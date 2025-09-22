
#pragma once

#include <stdint.h>

#include "controller_interface.h"

#include "mcinstance.h"
#include "mcstate.h"

// TODO Eventually this mixer will follow the standard "mixer.h" and "mixer_interface.h" pattern we use for the other libraries
#include "mixer.h"

/* LQR Controller as per state-space model in control/design/src/lqr
 *
 * Needs to take the current state and calculate the output vector [Total Force, Roll Torque, Pitch Torque, Yaw Torque]
 * and account for the target.
 *
 *   U = -k*(state - ref)
 *
 * xx
 *
 */
class LQR_Controller : public MC_Controller
{
	public:
		LQR_Controller() : MC_Controller() { }

	public:
		/* @brief Run controller (including call to mixer) */
		void run(void) override;

	private:
		/* @brief Calculate control output and call mixer */
		void control_output(float*);

	private:
		/* @brief Controller mixer algorithm */
		Mixer mixer;

		/* @brief LQR Gain matrix (row-major) */
		float lqr_k[48] = {
			// Row 0 (total force)
			0.0f,
			0.0f,
			-0.2236,
			0.0f,
			0.0f,
			0.0f,
			0.0f,
			0.0f,
			-1.3736,
			0.f,
			0.f,
			0.f,
			// Row 1 (roll torque)
			0.0f,
			0.4472,
			0.0f,
			2.6419,
			0.0f,
			0.0f,
			0.0f,
			1.0202,
			0.0f,
			0.3512,
			0.0f,
			0.0f,
			// Row 2 (pitch torque)
			-0.4472,
			0.0f,
			0.0f,
			0.0f,
			2.8652,
			0.0f,
			-1.0302,
			0.0f,
			0.0f,
			0.0f,
			0.4087,
			0.0f,
			// Row 3 (yaw torque)
			0.0f,
			0.0f,
			0.0f,
			0.0f,
			0.0f,
			0.2487,
			0.0f,
			0.0f,
			0.0f,
			0.0f,
			0.0f,
			0.5778
		};


};

