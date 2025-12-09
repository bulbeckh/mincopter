
#pragma once

#include "mcinstance.h"
#include "mcstate.h"

#include "mixer.h"

// TODO This should be moved to the sub-class
#define CONTROLLER_N_HORIZON 1

class MC_Controller {

	public:
		/* @brief Top-level abstraction for a MinCopter controller
		 * @param mcstate
		 * @param mcinstance
		 */
		MC_Controller() :
			angle_rate_max(18000),
			max_climb_rate(10000),
			min_climb_rate(10000)
			/* NOTE Hardcoded here but should probably pull from config file */
		{
		}

		// TODO Add destructor

	public:

		/* @brief Mixer Instance */
		Mixer mixer;

		// Controller run methods
		// NOTE By default, our controllers don't implement a lot of these methods

		/* @brief Runs the controller, specifying only the x-y-z position in the inertial frame */
		virtual void run_position(void) { }

		/* @brief Uses a specified x/y position but and a specified z-velocity instead of position */
		virtual void run_xy_position_z_velocity(void) { }

		/* @brief Uses a specified roll and pitch (typically both zero) and a specified z-velocity */
		virtual void run_roll_pitch_z_velocity(void) { }

		/* @brief All controllers need to implement a reset method that resets any state that the controller uses (e.g. PID I-term error) */
		virtual void reset(void) = 0;

		// TODO Potentially more methods depending on mode

		// TODO This will become an (12,N) matrix for an N-horizon controller like MPC 
		struct state_reference_t {
			// Position (inertial frame, metres)
			float x;
			float y;
			float z;

			// Orientation (inertial frame, rad)
			float roll;
			float pitch;
			float yaw;

			// Velocitiy (inertial frame, m/s)
			float dx;
			float dy;
			float dz;

			// Angular velocity (inertial frame, rad/s)
			float droll;
			float dpitch;
			float dyaw;
		};

		/* @brief Reference state set by planner */
		state_reference_t reference[CONTROLLER_N_HORIZON];


	protected:
		/* @brief Control actions */
		float u_force;
		float u_rt;
		float u_pt;
		float u_yt;

	public:

		// Control Limits

		// TODO split into separate max and min roll/pitch rates + add yaw_rate + rename these in the other files
		// TODO is int32_t really required here or will int16_t suffice

		/* @brief Maximum rotation rate in roll/pitch axis */
		int32_t angle_rate_max;

		/* @brief Parameters to control vertical ascent/descent speed. Set by planner */
		int16_t max_climb_rate;
		int16_t min_climb_rate;

		/* @brief The current climb rate. Used during call to get_throttle_rate (high-level) */
		// TODO Where is climb rate actually set??
		int16_t climb_rate;

		// NOTE TODO Duplicate with state vector
		/* @brief The desired altitude in cm. Setup by planner */
		float controller_desired_alt;

};

