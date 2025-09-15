
#pragma once

#include <AP_HAL/AP_HAL_Namespace.h>

// TODO Fix this as soon as possible - we should not be including AP_Math here and 
// we should also not have specific methods for retrieving readings as below.
//
// Rather, use a generic ::read method that takes a reading type enum or something 
// and then implementing the 'reading' functionality in the HAL subclasses (like Generic)

class AP_HAL::Sim
{
	public:
		Sim() {}

    public:
		/* @brief Set up UDP socket between this and GZ server process */
		virtual bool setup_sim_socket(void) = 0;

		/* @brief Send a motor control output PWM */
		virtual bool send_control_output(void) = 0;

		/* @brief Receive, parse, and store a GZ simulation state packet */
		virtual bool recv_state_input(void) = 0;

		/* @brief Steps the simulation by the desired microseconds */
		virtual void tick(uint32_t tick_us) = 0;

	public:
		/*
		virtual void get_barometer_pressure(float& pressure) = 0;

		virtual void get_compass_field(Vector3f& field) = 0;

		virtual void get_imu_gyro_readings(Vector3f& gyro_rate) = 0;

		virtual void get_imu_accel_readings(Vector3f& accel) = 0;

		virtual void update_gps_position(int32_t& latitude, int32_t& longitude, int32_t& altitude) = 0;
		
		virtual void update_gps_velocities(int32_t& vel_north, int32_t& vel_east, int32_t& vel_down) = 0;

		*/

};


