#pragma once

/* Implementation of a complementary filter for state estimation */

#include "mcstate_interface.h"

class StateComplementary : public MCState
{
	public:
		StateComplementary(void) : MCState() { }

		void update(void) override;

		void init_derived(void) override;

	private:
		/* @brief Complementary filter computes euler angles so we maintain the state in euler angles */
		Vector3f euler_internal;

		/* @brief Whether we have completed first update */
		uint8_t _first_update;

		// TODO Alpha should be passed in as an initialised value somehow
		/* @brief The alpha value of the complementary filter */
		//float alpha = 0.998;
		float alpha = 0.98;

		float alpha_yaw = 0.8;

		/* @brief For fusion of vertical barometer data with integrated IMU data, we use this weighting */
		float z_axis_baro_fuse_alpha = 0.3;

		/* @brief Weighting for how much we fuse accelerometer integration (position) over GPS position */
		float x_axis_gps_fuse_alpha = 0.1;
		float y_axis_gps_fuse_alpha = 0.1;
		float z_axis_gps_fuse_alpha = 0.1;

		/* @brief Weighting for how much we fuse accelerometer integration (velocity) over GPS velocities */
		float x_axis_gpsvel_fuse_alpha = 0.05;
		float y_axis_gpsvel_fuse_alpha = 0.05;
		float z_axis_gpsvel_fuse_alpha = 0.05;

		/* @brief Estimates of 3D position from GPS to be fused with position state. Kept as state variable
		 * for logging purposes */
		float x_position_est;
		float y_position_est;
		float z_position_est;

};


