
#pragma once

#include "ahrs_interface.h"

/* Complementary filter for fusing gyrometer, accelerometer, and magnetometer data */

class AHRS_Complementary : public AP_AHRS
{
	public:
		AHRS_Complementary(void) : AP_AHRS() { }

		/* @brief AHRS update method */
		void ahrs_update(void) override;

		/* @brief internal init method */
		void _ahrs_init_internal(void) override;

	private:
		/* @brief Complementary filter computes euler angles so we maintain the state in euler angles */
		Vector3f euler_internal;

		/* @brief Whether we have completed first update */
		uint8_t _first_update;

		// TODO Alpha should be passed in as an initialised value somehow
		/* @brief The alpha value of the complementary filter */
		float alpha = 0.0;




};


