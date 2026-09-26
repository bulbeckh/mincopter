
#pragma once

#include <arch/linux/generic/AP_HAL_Generic_Namespace.h>
#include <AP_HAL/Sim.h>

#include "SocketUnix.hh"

#include <netinet/in.h>

#include <AP_Math.h>

#include <cstdint>

/* @brief The buffer length used to buffer readings from the simulation */

class generic::GenericGZInterface : public AP_HAL::Sim {

	public:
		/* @brief Defines a socket connection between the Gazebo simulation and the mincopter runtime */
		GenericGZInterface() { }

    private:
		/* @brief UNIX Socket for communication with simulator */
		SocketUnix usocket;

		/* @brief File descriptor for log pipe */
		int logfd;

	public:
		/* @brief Holds the control PWM signals when using direct updates rather than via AP_Motors */
		uint16_t control_pwm[4];

	private:
		/* @brief Counter for how many times we have failed to receive a state packet */
		uint8_t receive_packet_retries{0};


    public:
		/* @brief Set up UDP socket between this and GZ server process */
		bool setup_sim_socket(void) override;

		/* @brief Send a motor control output PWM */
		bool send_control_output(bool retry) override;

		/* @brief Receive, parse, and store a GZ simulation state packet */
		bool recv_state_input(void) override;

		/* @brief Iterate the simulation by the specified microseconds */
		void tick(uint32_t tick_us) override;

		/* @brief Reset the simulation back to default configuration including all model poses and simulation time */
		//void reset(void) override;

		// TODO Move these logging functions into a unified logger library

		bool setup_log_source(const char*, LogSource source) override;
		void log_state(uint8_t* data, uint8_t len, uint8_t type) override;

		void reset(void) override;
		void set_mincopter_position(float, float, float) override;
		void set_mincopter_attitude(float, float, float) override;
		void set_mincopter_linvelocity(float, float, float) override;
		void set_mincopter_angvelocity(float, float, float) override;


};


