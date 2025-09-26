
#pragma once

#include <AP_HAL/AP_HAL_Namespace.h>

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
		virtual bool send_control_output(void) = 0;

		/* @brief Receive, parse, and store a GZ simulation state packet */
		virtual bool recv_state_input(void) = 0;

		/* @brief Steps the simulation by the desired microseconds */
		virtual void tick(uint32_t tick_us) = 0;

	public:
		/* @brief Simulation state struct */
		struct mc_sim_state_packet {
			double timestamp;

			/* IMU */
			double imu_gyro_x;
			double imu_gyro_y;
			double imu_gyro_z;
			double imu_accel_x;
			double imu_accel_y;
			double imu_accel_z;

			double pos_x;
			double pos_y;
			double pos_z;

			// Is this now intrisic or extrinsic rotation?? In which order?
			double wldAbdyA_eul_x; // Roll
			double wldAbdyA_eul_y; // Pitch
			double wldAbdyA_eul_z; // Yaw
			
			double euler_rate_x;
			double euler_rate_y;
			double euler_rate_z;

			double vel_x;
			double vel_y;
			double vel_z;

			/* Magnetometer */
			double field_x;
			double field_y;
			double field_z;

			/* Barometer */
			double pressure;

			/* NavSat (GPS) */
			double lat_deg;
			double lng_deg;
			double alt_met;
			double vel_east;
			double vel_north;
			double vel_up;

		};

		/* @brief The struct containing all sensor information. This is accessed by each of the sim_* 
		 * simulated sensor classes */
		mc_sim_state_packet sensor_states[GZ_INTERFACE_STATE_BUFFER_LENGTH];

		mc_sim_state_packet last_sensor_state;

		/* @brief The index in the buffer that we will next read sensor states to */
		uint8_t state_buffer_index=0;

		// TODO This is a very bad quick hack to get the motor output - should really be taking this from hal.rcout
		int16_t motor_out[4];
		float control_input[4];

	public:

		/* @brief Reset the simulation back to default configuration including all model poses and simulation time */
		virtual void reset(void) = 0;

		/* @brief Update the pose of the copter in the Gazebo simulation. This should zero all velocities/accelerations/momentum.
		 * Pose is specified in the MinCopter frame (NED, extrinsic X-Y-Z orientation) with position in metres and orientation in radians */
		virtual void set_mincopter_pose(float x_ned_m, float y_ned_m, float z_ned_m, float roll_rad, float pitch_rad, float yaw_rad) = 0;

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


