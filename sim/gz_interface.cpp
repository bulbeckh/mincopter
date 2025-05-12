/* Functions for communication between gazebo and this software simulation
 *
 * Structs taken from <add link to ardupilot_gazebo>
 *
 *
 */

#include "gz_interface.h"

#include <iostream>
#include <cstring>
#include <string.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "mcinstance.h"
extern MCInstance mincopter;

bool GZ_Interface::setup_sim_socket()
{
    // Initialisation
    //

    // Create a UDP socket with arbitrary port
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0 ) {
	std::cout << "Error: creating UDP socket\n";
	return false;
    }

    memset(&servaddr, 0, sizeof(servaddr));

    servaddr.sin_family = AF_INET;
    //servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    inet_pton(AF_INET, "127.0.0.1", &servaddr.sin_addr);
    // Use port zero to auto configure
    servaddr.sin_port = htons(0);

    if (bind(sockfd, (const struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
	std::cout << "Error: binding to UDP port\n";
	close(sockfd);
	return false;
    }

    std::cout << "Socket created successfully at 127.0.0.1\n";

    return true;
}

bool GZ_Interface::send_control_output()
{
    /* These hold the roll, pitch, and yaw outputs but these are interpreted by the
     * motors class into actual motor values.
     *
     * mincopter.rc_1.servo_out;
     * mincopter.rc_2.servo_out;
     * mincopter.rc_4.servo_out;
     *
     */
    
    servo_packet_16 control_pkt;

    control_pkt.frame_count = frame_counter;
    control_pkt.frame_rate  = 1001;

    static int send_counter=0;
    for (int16_t i=0;i<4;i++) {
	int16_t m_out = mincopter.motors.get_raw_motor_out(i);

	// NOTE The pkt entry is int16_t whereas m_out uint16_t
	//control_pkt.pwm[i] = m_out;
	control_pkt.pwm[i] = 2000;
    }

    // Send packet
    struct sockaddr_in cliaddr;
    memset(&cliaddr, 0, sizeof(cliaddr));

    cliaddr.sin_family = AF_INET;
    inet_pton(AF_INET, "127.0.0.1", &cliaddr.sin_addr);
    cliaddr.sin_port = htons(9002);

    socklen_t len = sizeof(cliaddr);
    sendto(sockfd, &control_pkt, sizeof(servo_packet_16), 0,
	    (const struct sockaddr*)&cliaddr, len);
    
    send_counter++;

    if (send_counter%100==0) {
	for (int i=0;i<4;i++) {
	    std::cout << "Motor " << i << ": " << mincopter.motors.get_raw_motor_out(i) << ", ";
	}
	std::cout << "\n";
	send_counter=0;
    }

    return false;
}

bool GZ_Interface::recv_state_input()
{

    socklen_t len = sizeof(servaddr);

    ssize_t n_received = recvfrom(sockfd, buffer, 1024, 0,
	    (struct sockaddr*)&servaddr, &len);

    // Iterate frame counter
    frame_counter += 1;

    // Unpack received data

    /* Example JSON pack
     *
     * {
     *  'timestamp': xx,
     * 	'imu' : {
     * 		'gyro' : [x,y,z],
     * 		'accel_body : [x,y,z],
     * 		},
     * 	'position' : [x,y,z],
     * 	'quaternion' : [a,b,c,d],
     * 	'velocity' : [x,y,z]
     * }
     * 		
     * TODO Add the following sensors to the mincopter sdf model:
     * - Barometer
     * - Compass
     * - GPS
     *
     */

    mc_sim_state_packet* pkt = (mc_sim_state_packet*)buffer;

    // For now, just create a copy of the structure but maybe in future can have a more elegant solution
    // like separate structs for each sensor type
    sensor_states = *pkt;

    //std::cout << "PKT Data: " << pkt->timestamp << ", pos-x: " << pkt->pos_x << "\n";
    std::cout << "PKT Data: " << pkt->timestamp << ", field-x: " << pkt->field_x << " baro pressure " << pkt->pressure << "\n";

    return true;
}

