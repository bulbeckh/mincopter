
#pragma once

#include <stdint.h>
#include <netinet/in.h>

class GZ_Interface {
    public:
	
	/* @brief Defines a socket connection between the Gazebo simulation and the mincopter runtime
	 */
	GZ_Interface() : frame_counter(0)
	{
	}


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

	char buffer[1024];

    public:

	bool setup_sim_socket();

	bool send_control_output();

	bool recv_state_input();

};


