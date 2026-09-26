
#pragma once

#include <AP_HAL/AP_HAL_Namespace.h>

#include <cstdint>

#include "SimulationMessage.h"

// TODO Fix this as soon as possible - we should not be including AP_Math here and 
// we should also not have specific methods for retrieving readings as below.
//
// Rather, use a generic ::read method that takes a reading type enum or something 
// and then implementing the 'reading' functionality in the HAL subclasses (like Generic)

#define GZ_INTERFACE_STATE_BUFFER_LENGTH 10

class AP_HAL::Sim
{
	public:
		Sim() {}

	public:
		enum class LogSource {
			PIPE,
			LOGFILE
		};

    public:
		/* @brief Set up UDP socket between this and GZ server process */
		virtual bool setup_sim_socket(void) = 0;

		/* @brief Set up the pipe to log current state to other processes */
		virtual bool setup_log_source(const char* addr, LogSource source) = 0;

		/* @brief Send a motor control output PWM */
		virtual bool send_control_output(bool) = 0;

		/* @brief Receive, parse, and store a GZ simulation state packet */
		virtual bool recv_state_input(void) = 0;

		/* @brief Steps the simulation by the desired microseconds */
		virtual void tick(uint32_t tick_us) = 0;

	public:

		/* @brief The struct containing all sensor information. This is accessed by each of the sim_* 
		 * simulated sensor classes */
		mc::StateMessage sensor_states[GZ_INTERFACE_STATE_BUFFER_LENGTH];

		mc::StateMessage last_sensor_state;

		/* @brief The index in the buffer that we will next read sensor states to */
		uint8_t state_buffer_index=0;

		// TODO This is a very bad quick hack to get the motor output - should really be taking this from hal.rcout
		int16_t motor_out[4];
		float control_input[4];

		/* @brief Flag for whether we can consider or data valid yet (i.e. a first read has been done) */
		bool valid{false};

	public:
		
		/* **Simulation Control Methods**
		 *
		 * During testing, we need to be able to reset the simulation and arbitrarily modify the state of the quadcopter
		 * including position, velocity, and attitude.
		 *
		 * We supply methods below to set state which is communicated to the ArduPilot gazebo driver at the next call to
		 * **send_control_output**. Note, setting any state during a simulation step will cause the gazebo driver to ignore
		 * the supplied x4 control output vector for that step.
		 *
		 * Resetting the simulation also updates the internal mincopter millis/micros count. Simulated sensor drivers should
		 * check for jumps in time due to a reset.
		 */

		/* @brief Reset the simulation back to default configuration including all model poses and simulation time */
		virtual void reset(void) = 0;

		/* @brief Update the position of the copter in the Gazebo simulation. Pose is specified in the MinCopter frame (NED,
		 * extrinsic X-Y-Z orientation) with position in metres and orientation in radians */
		virtual void set_mincopter_position(float x_ned_m, float y_ned_m, float z_ned_m) = 0;

		/* @brief Update the attitude of the copter in the Gazebo simulation. Pose is specified in the MinCopter frame (NED,
		 * extrinsic X-Y-Z orientation) with position in metres and orientation in radians */
		virtual void set_mincopter_attitude(float roll_rad, float pitch_rad, float yaw_rad) = 0;

		/* @brief Update the linear velocity of the copter in the Gazebo simulation. Pose is specified in the MinCopter frame (NED,
		 * extrinsic X-Y-Z orientation) with position in metres and orientation in radians */
		virtual void set_mincopter_linvelocity(float dx_ned_ms, float dy_ned_ms, float dz_ned_ms) = 0;

		/* @brief Update the angular velocity of the copter in the Gazebo simulation. Pose is specified in the MinCopter frame (NED,
		 * extrinsic X-Y-Z orientation) with position in metres and orientation in radians */
		virtual void set_mincopter_angvelocity(float droll_rads, float dpitch_rads, float dyaw_rads) = 0;

	protected:
		/* @brief Flag that we have lost connection to the gazebo simulation plugin */
		bool connection_lost{false};

	public:
		/* @brief Return true if we are still connected */
		bool connected(void) { return !connection_lost; }

	public:

		/* @brief Log state data to the pipe */
		virtual void log_state(uint8_t* data, uint8_t len, uint8_t type) = 0;


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


