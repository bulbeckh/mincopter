
#pragma once

#include <arch/linux/generic/AP_HAL_Generic.h>

#include <stdint.h>
#include <netinet/in.h>

#include <AP_Math.h>

/* @brief The buffer length used to buffer readings from the simulation */

class generic::GenericGZInterface : public AP_HAL::Sim {

	public:
		/* @brief Defines a socket connection between the Gazebo simulation and the mincopter runtime */
		GenericGZInterface() : frame_counter(0) { }

    private:
		/* @brief Struct to hold a control input (motor speeds) packet */
		struct servo_packet_16 {
			uint16_t magic = 18458; // constant magic value
			uint16_t frame_rate;
			uint32_t frame_count;
			uint16_t pwm[4]; // See below for structure
			uint8_t  update_flag;
			float    update_position[3];
			float    update_velocity[3];
			float    update_attitude[3];
			float    update_angvel[3];
		};

		/* servo_packet_16 should contain
		 *
		 * 8b for x4 uint16_t pwm signals       : 4
		 * 2b for update flags					: 1
		 * 12,24,36,48b dependent on flags set  : 16
		 *
		 */

		struct sockaddr_in servaddr;

		/* @brief Number of iterations in the simulation */
		uint32_t frame_counter;

		/* @brief File descriptor for socket */
		int sockfd;

		/* @brief Holds the raw memory stream from a UDP packet */
		char buffer[1024];

		/* @brief File descriptor for log pipe */
		int logfd;

	public:
		/* @brief Holds the control PWM signals when using direct updates rather than via AP_Motors */
		uint16_t control_pwm[4];

	private:
		/* @brief Array of update flags for each state */
		bool position_update;
		bool velocity_update;
		bool attitude_update;
		bool angvel_update;

		/* @brief Vectors of new simulation states to be communicated to gazebo */
		Vector3f sim_new_position;
		Vector3f sim_new_velocity;
		Vector3f sim_new_attitude;
		Vector3f sim_new_angvel;

    public:
		/* @brief Set up UDP socket between this and GZ server process */
		bool setup_sim_socket(void) override;

		bool setup_log_source(const char*, LogSource source) override;

		void log_state(uint8_t* data, uint8_t len, uint8_t type) override;

		/* @brief Send a motor control output PWM */
		bool send_control_output(void) override;

		/* @brief Receive, parse, and store a GZ simulation state packet */
		bool recv_state_input(void) override;

		/* @brief Iterate the simulation by the specified microseconds */
		void tick(uint32_t tick_us) override;

		/* @brief Reset the simulation back to default configuration including all model poses and simulation time */
		void reset(void) override;

		void set_mincopter_position(float x_ned_m, float y_ned_m, float z_ned_m) override;
		void set_mincopter_attitude(float roll_rad, float pitch_rad, float yaw_rad) override;
		void set_mincopter_linvelocity(float dx_ned_ms, float dy_ned_ms, float dz_ned_ms) override;
		void set_mincopter_angvelocity(float droll_rads, float dpitch_rads, float dyaw_rads) override;


};


