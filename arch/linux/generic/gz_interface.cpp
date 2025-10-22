/* Functions for communication between gazebo and this software simulation
 *
 * Structs taken from <add link to ardupilot_gazebo>
 *
 *
 */

#include <arch/linux/generic/gz_interface.h>

#include <iostream>
#include <fstream>
#include <cstring>
#include <string.h>

#include <sys/stat.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <AP_Math.h>

using namespace generic;

/* TODO A far better way is to re-cast this hal object to the HAL_Generic class and then 
 * call methods from the .sim object, rather than have a base AP_HAL::Sim class that will only
 * ever really be implemented by the HAL Generic. */
extern const AP_HAL::HAL& hal;

void GenericGZInterface::tick(uint32_t tick_us)
{
	// TODO This is where we all **send_control_output** and **recv_state_input**
	// TODO Use the tick_us param to drive the simulation step
	
	hal.console->printf("Sending control output...\r\n");
	send_control_output();
	hal.console->printf("Sent control output...\r\n");

	// After we (possibly) send a state to the gazebo driver, we clear the flags for each state update
	position_update = false;
	velocity_update = false;
	attitude_update = false;
	angvel_update = false;

	reset_requested = false;
	
	// Receive next state and update internal simulation state
	hal.console->printf("Receiving state input...\r\n");
	recv_state_input();
	hal.console->printf("Received state input...\r\n");

	return;
}

bool GenericGZInterface::setup_sim_socket(void)
{
	hal.console->printf("Initialising connection to Gazebo Simulator...\n");

    // Create a UDP socket with arbitrary port
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    if (sockfd < 0 ) {
		hal.console->printf("Error: creating UDP socket\n");
		return false;
    }

    memset(&servaddr, 0, sizeof(servaddr));

    servaddr.sin_family = AF_INET;
    //servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    inet_pton(AF_INET, "127.0.0.1", &servaddr.sin_addr);
    // Use port zero to auto configure
    servaddr.sin_port = htons(0);

    if (bind(sockfd, (const struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
		hal.console->printf("Error: binding to UDP port\n");
		close(sockfd);
		return false;
    }

	hal.console->printf("Socket created successfully at 127.0.0.1\n");

	// Setup a 1s timeout for the receive function
	struct timeval _tv_timeout;
	_tv_timeout.tv_sec=1;
	_tv_timeout.tv_usec=0;
	setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &_tv_timeout, sizeof(_tv_timeout));

    return true;
}

bool GenericGZInterface::send_control_output(void)
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

	/* TODO NOTE We are now going to get the PWM signals directly from hal.rcout instead of mincopter.motors.get_raw_motor_out
	 * as this makes it easy to work across multiple backends and also eliminate the need for AP_Motors */
	
	// TODO Update this

    for (int16_t i=0;i<16;i++) {
		// TODO Change how motor PWM signal is retrived - should be the output of mixer but unclear whether to use RCOutput PWM or elsewhere
		//int16_t m_out = mincopter.motors.get_raw_motor_out(i);
		int16_t m_out = 0;

		// NOTE The pkt entry is int16_t whereas m_out uint16_t
		//control_pkt.pwm[i] = m_out;
		// Send minimum
		control_pkt.pwm[i] = 0;
    }

	// TODO PWM should be retrieved from the RCOutput HAL object and not the temporary hal.sim->motor_out array
	// Read PWM signals to be sent
	control_pkt.pwm[0] = hal.sim->motor_out[0];
	control_pkt.pwm[1] = hal.sim->motor_out[1]; 
	control_pkt.pwm[2] = hal.sim->motor_out[3];
	control_pkt.pwm[3] = hal.sim->motor_out[2];

	// Uncomment this to send a uniform constant PWM output
	
	/*
	static uint32_t send_pwm=1100;
	control_pkt.pwm[0] = send_pwm;
	control_pkt.pwm[1] = send_pwm;
	control_pkt.pwm[2] = send_pwm;
	control_pkt.pwm[3] = send_pwm;
	*/

	// If we have set the state directly during this step, we add to the packet
	uint8_t update_flag = 0x00;

	if (position_update) {
		update_flag |= (0x01 << 0);
		control_pkt.update_position[0] = sim_new_position.x;
		control_pkt.update_position[1] = sim_new_position.y;
		control_pkt.update_position[2] = sim_new_position.z;
	}

	if (velocity_update) {
		update_flag |= (0x01 << 1);
		control_pkt.update_velocity[0] = sim_new_velocity.x;
		control_pkt.update_velocity[1] = sim_new_velocity.y;
		control_pkt.update_velocity[2] = sim_new_velocity.z;
	}

	if (attitude_update) {
		update_flag |= (0x01 << 2);
		control_pkt.update_attitude[0] = sim_new_attitude.x;
		control_pkt.update_attitude[1] = sim_new_attitude.y;
		control_pkt.update_attitude[2] = sim_new_attitude.z;
	}

	if (angvel_update) {
		update_flag |= (0x01 << 3);
		control_pkt.update_angvel[0] = sim_new_angvel.x;
		control_pkt.update_angvel[1] = sim_new_angvel.y;
		control_pkt.update_angvel[2] = sim_new_angvel.z;
	}

	// Add the update flag bitfield to the control packet
	control_pkt.update_flag = update_flag;

    // Send packet
    struct sockaddr_in cliaddr;
    memset(&cliaddr, 0, sizeof(cliaddr));

    cliaddr.sin_family = AF_INET;
    inet_pton(AF_INET, "127.0.0.1", &cliaddr.sin_addr);
    cliaddr.sin_port = htons(9002);

    socklen_t len = sizeof(cliaddr);
    sendto(sockfd, &control_pkt, sizeof(servo_packet_16), 0, (const struct sockaddr*)&cliaddr, len);
    
    return false;
}

bool GenericGZInterface::recv_state_input(void)
{

    socklen_t len = sizeof(servaddr);

    ssize_t n_received = recvfrom(sockfd, buffer, 1024, 0,
	    (struct sockaddr*)&servaddr, &len);

	// NOTE TODO This works but may skip calls to update state during the loop as a call to ::tick
	// won't increment the frame_counter but will reset the direct state update flags
	
	// If we timeout (1second) then we just continue the tick loop and send the control packet again
	if (n_received<0) {
		return false;
	}

    // Iterate frame counter
    frame_counter += 1;

    // Unpack received data
    mc_sim_state_packet* pkt = (mc_sim_state_packet*)buffer;

    // For now, just create a copy of the structure but maybe in future can have a more elegant solution
    // like separate structs for each sensor type
    sensor_states[state_buffer_index] = *pkt;
	
	// TODO We should really just be stopping here and exposing state retrieval functions for each of the sim driver in dev/
	
	// For the IMU sensor, we need to simulate the DLPF by updating a proportion of the previous filter reading
	uint8_t temp_idx=0;
	if (state_buffer_index==0) {
		temp_idx=GZ_INTERFACE_STATE_BUFFER_LENGTH-1;
	} else {
		temp_idx = state_buffer_index-1;
	}

	// **alpha** is a number between 0 and 1 that controls how much of the previous value we use. Essentially a digital low pass filter
	//float alpha=0.6;
	float alpha=0.0;
	sensor_states[state_buffer_index].imu_gyro_x = sensor_states[temp_idx].imu_gyro_x*alpha + sensor_states[state_buffer_index].imu_gyro_x*(1.0f-alpha);
	sensor_states[state_buffer_index].imu_gyro_y = sensor_states[temp_idx].imu_gyro_y*alpha + sensor_states[state_buffer_index].imu_gyro_y*(1.0f-alpha);
	sensor_states[state_buffer_index].imu_gyro_z = sensor_states[temp_idx].imu_gyro_z*alpha + sensor_states[state_buffer_index].imu_gyro_z*(1.0f-alpha);

	sensor_states[state_buffer_index].imu_accel_x = sensor_states[temp_idx].imu_accel_x*alpha + sensor_states[state_buffer_index].imu_accel_x*(1.0f-alpha);
	sensor_states[state_buffer_index].imu_accel_y = sensor_states[temp_idx].imu_accel_y*alpha + sensor_states[state_buffer_index].imu_accel_y*(1.0f-alpha);
	sensor_states[state_buffer_index].imu_accel_z = sensor_states[temp_idx].imu_accel_z*alpha + sensor_states[state_buffer_index].imu_accel_z*(1.0f-alpha);

	state_buffer_index += 1;
	state_buffer_index %= GZ_INTERFACE_STATE_BUFFER_LENGTH;

	last_sensor_state = *pkt;

    return true;
}


bool GenericGZInterface::setup_log_source(const char* addr, LogSource source)
{
	if (source==LogSource::PIPE) {
		// Create pipe if it doesn't already exist
		mkfifo(addr, 0666);

		// NOTE This will block until we open the pipe on the other side
		logfd = open(addr, O_WRONLY);
	} else if (source==LogSource::LOGFILE) {
		// Create file in current directory
		logfd = open(addr, O_WRONLY | O_CREAT | O_TRUNC, 0644);
	}

	if (logfd < 0 ) {
		hal.console->printf("bad fd for logging\n");
		hal.scheduler->panic("Could not open pipe for logging\n");
	}

	return true;
}

void GenericGZInterface::log_state(uint8_t* data, uint8_t len, uint8_t type)
{
	if (len==0) {
		hal.console->printf("Log state called with len=0. Ignoring\n");
		return;
	}

	uint8_t packet[len+4];

	// Sync bytes
	packet[0] = 0x2A;
	packet[1] = 0x4E;

	// TODO Change this to some sort of shared enum that represents the packet type;
	packet[2] = type;
	packet[3] = len;

	// TODO Change to memcpy?
	for (uint8_t i=0;i<len;i++) {
		packet[i+4] = data[i];
	}

	// Log to pipe
	write(logfd, packet, len+4);

	return;
}

void GenericGZInterface::reset(void)
{
	// Flag that a reset has been requested
	
	return;
}

void GenericGZInterface::set_mincopter_position(float x_ned_m, float y_ned_m, float z_ned_m)
{
	position_update = true;

	sim_new_position.x = x_ned_m;
	sim_new_position.y = y_ned_m;
	sim_new_position.z = z_ned_m;

	return;
}

void GenericGZInterface::set_mincopter_attitude(float roll_rad, float pitch_rad, float yaw_rad)
{
	attitude_update = true;

	sim_new_attitude.x = roll_rad;
	sim_new_attitude.y = pitch_rad;
	sim_new_attitude.z = yaw_rad;

	return;
}

void GenericGZInterface::set_mincopter_linvelocity(float dx_ned_ms, float dy_ned_ms, float dz_ned_ms)
{
	velocity_update = true;

	sim_new_velocity.x = dx_ned_ms;
	sim_new_velocity.y = dy_ned_ms;
	sim_new_velocity.z = dz_ned_ms;

	return;
}

void GenericGZInterface::set_mincopter_angvelocity(float droll_rads, float dpitch_rads, float dyaw_rads)
{
	angvel_update = true;

	sim_new_angvel.x = droll_rads;
	sim_new_angvel.y = dpitch_rads;
	sim_new_angvel.z = dyaw_rads;

	return;
}


