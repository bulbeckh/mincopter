/* Functions for communication between gazebo and this software simulation
 *
 * Structs taken from <add link to ardupilot_gazebo>
 *
 *
 */

#include "gz_interface.h"

#include <iostream>
#include <cstring>
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

    int n = recvfrom(sockfd, buffer, 1024, 0,
	    (struct sockaddr*)&servaddr, &len);

    frame_counter += 1;

    return true;
}

/*
void JSON::output_servos(const struct sitl_input &input)
{
    size_t pkt_size = 0;
    ssize_t send_ret = -1;
    if (SRV_Channels::have_32_channels()) {
      servo_packet_32 pkt;
      pkt.frame_rate = rate_hz;
      pkt.frame_count = frame_counter;
      for (uint8_t i=0; i<32; i++) {
          pkt.pwm[i] = input.servos[i];
      }
      pkt_size = sizeof(pkt);
      send_ret = sock.sendto(&pkt, pkt_size, target_ip, control_port);
    } else {
      servo_packet_16 pkt;
      pkt.frame_rate = rate_hz;
      pkt.frame_count = frame_counter;
      for (uint8_t i=0; i<16; i++) {
          pkt.pwm[i] = input.servos[i];
      }
      pkt_size = sizeof(pkt);
      send_ret = sock.sendto(&pkt, pkt_size, target_ip, control_port);
    }

    if ((size_t)send_ret != pkt_size) {
        if (send_ret <= 0) {
            printf("Unable to send servo output to %s:%u - Error: %s, Return value: %ld\n",
                   target_ip, control_port, strerror(errno), (long)send_ret);
        } else {
            printf("Sent %ld bytes instead of %lu bytes\n", (long)send_ret, (unsigned long)pkt_size);
        }
    }
}

void JSON::recv_fdm(const struct sitl_input &input)
{
    // Receive sensor packet
    ssize_t ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, UDP_TIMEOUT_MS);
    uint32_t wait_ms = UDP_TIMEOUT_MS;
    while (ret <= 0) {
        //printf("No JSON sensor message received - %s\n", strerror(errno));
        ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, UDP_TIMEOUT_MS);
        wait_ms += UDP_TIMEOUT_MS;
        // if no sensor message is received after 10 second resend servos, this help cope with SITL and the physics getting out of sync
        if (wait_ms > 1000) {
            wait_ms = 0;
            printf("No JSON sensor message received, resending servos\n");
            output_servos(input);
        }
    }

    // convert '\n' into nul
    while (uint8_t *p = (uint8_t *)memchr(&sensor_buffer[sensor_buffer_len], '\n', ret)) {
        *p = 0;
    }
    sensor_buffer_len += ret;

    const uint8_t *p2 = (const uint8_t *)memrchr(sensor_buffer, 0, sensor_buffer_len);
    if (p2 == nullptr || p2 == sensor_buffer) {
        return;
    }

    const uint8_t *p1 = (const uint8_t *)memrchr(sensor_buffer, 0, p2 - sensor_buffer);
    if (p1 == nullptr) {
        return;
    }

    const uint32_t received_bitmask = parse_sensors((const char *)(p1+1));
    if (received_bitmask == 0) {
        // did not receive one of the mandatory fields
        printf("Did not contain all mandatory fields\n");
        return;
    }

    // Must get either attitude or quaternion fields
    if ((received_bitmask & (EULER_ATT | QUAT_ATT)) == 0) {
        printf("Did not receive attitude or quaternion\n");
        return;
    }

    if (received_bitmask != last_received_bitmask) {
        // some change in the message we have received, print what we got
        printf("\nJSON received:\n");
        for (uint16_t i=0; i<ARRAY_SIZE(keytable); i++) {
            struct keytable &key = keytable[i];
            if ((received_bitmask &  1U << i) == 0) {
                continue;
            }
            if (strcmp(key.section, "") == 0) {
                printf("\t%s\n",key.key);
            } else {
                printf("\t%s: %s\n",key.section,key.key);
            }
        }
        printf("\n");
    }
    last_received_bitmask = received_bitmask;
*/
