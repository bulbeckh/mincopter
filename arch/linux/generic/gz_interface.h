
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
			uint16_t pwm[16];
		};

		struct sockaddr_in servaddr;

		/* @brief Number of iterations in the simulation */
		uint32_t frame_counter;

		/* @brief File descriptor for socket */
		int sockfd;

		/* @brief Holds the raw memory stream from a UDP packet */
		char buffer[1024];

	public:
		/* @brief Holds the control PWM signals when using direct updates rather than via AP_Motors */
		uint16_t control_pwm[4];

    public:
		/* @brief Set up UDP socket between this and GZ server process */
		bool setup_sim_socket(void) override;

		/* @brief Send a motor control output PWM */
		bool send_control_output(void) override;

		/* @brief Receive, parse, and store a GZ simulation state packet */
		bool recv_state_input(void) override;

		/* @brief Iterate the simulation by the specified microseconds */
		void tick(uint32_t tick_us) override;

		/* @brief Reset the simulation back to default configuration including all model poses and simulation time */
		void reset(void) override;

		/* @brief Update the pose of the copter in the Gazebo simulation. This should zero all velocities/accelerations/momentum */
		void set_mincopter_pose(float x_ned_m, float y_ned_m, float z_ned_m, float roll_rad, float pitch_rad, float yaw_rad) override;



};


