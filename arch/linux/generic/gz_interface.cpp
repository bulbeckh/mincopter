/* Functions for communication between gazebo and this software simulation
 *
 * Structs taken from <add link to ardupilot_gazebo>
 *
 *
 */

#include <arch/linux/generic/gz_interface.h>

// TODO yet another hack to get access to hal object from with a hal component implementation
#include <AP_HAL/AP_HAL.h>

#include <iostream>
#include <fstream>
#include <cstring>
#include <string.h>

#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>

// TODO Maybe remove these two?
#include <netinet/in.h>
#include <arpa/inet.h>

#include <unistd.h>

#include <AP_Math.h>

// Included from ap-gz/
#include "SimulationMessage.h"

using namespace generic;

/* TODO A far better way is to re-cast this hal object to the HAL_Generic class and then 
 * call methods from the .sim object, rather than have a base AP_HAL::Sim class that will only
 * ever really be implemented by the HAL Generic. */
extern const AP_HAL::HAL& hal;

void GenericGZInterface::tick(uint32_t /* unused */)
{
	static uint16_t iterations{0};
	if (iterations>=1000) {
		std::cout << "Tick 1000" << std::endl;
		iterations = 0;
	}

	// TODO This is where we all **send_control_output** and **recv_state_input**
	// TODO Use the tick_us param to drive the simulation step
	
	send_control_output(false);

	// Receive next state and update internal simulation state
	recv_state_input();

	return;
}

bool GenericGZInterface::setup_sim_socket(void)
{
	hal.console->printf("[HAL ] Initialising connection to Gazebo Simulator...\n");

	if (!usocket.init_client()) {
		hal.console->printf("[HAL ] Failed to initialise unix socket...\n");
		return false;
	}

	hal.console->printf("[HAL ] UNIX socket created successfully under %s\n", usocket.get_socket_path());

    	return true;
}

bool GenericGZInterface::send_control_output(bool /* retry */)
{
	// In the new formulation of the lightweight messaging system, this will instead create
	// and serialize a control message and then send via the socket

	mc::ControlMessage message;

	message.pwm[0] = hal.sim->motor_out[0];
	message.pwm[1] = hal.sim->motor_out[1]; 
	message.pwm[2] = hal.sim->motor_out[3];
	message.pwm[3] = hal.sim->motor_out[2];

	// Send message to socket
	if (!usocket.send_message<mc::ControlMessage>(message)) {
		std::cout << "Error sending control output packet\n";
		return false;
	}

	std::cout << "[HAL ] Sent control output packet\n";

	return true;
}

// TODO I still think this interface needs work, perhaps even an asynchronous callback,
// checks that iteration count/timing is in sync, checks for missed packets, and re-try functionality
bool GenericGZInterface::recv_state_input(void)
{

	auto deserialized_msg = usocket.receive_message<mc::StateMessage>();

	if (!deserialized_msg) {
		std::cout << "in recv_state_input, failed to deserialize or receive the StateMessage\n";
		return false;
	}

    // For now, just create a copy of the structure but maybe in future can have a more elegant solution
    // like separate structs for each sensor type
	// TODO Inefficient
    sensor_states[state_buffer_index] = *deserialized_msg;
	
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

	last_sensor_state = *deserialized_msg;

	// Set our valid flag, indicating that we have received data
	valid = true;

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

// TODO Either remove the following from the interface or implement them
void GenericGZInterface::reset(void) { }
void GenericGZInterface::set_mincopter_position(float, float, float) { }
void GenericGZInterface::set_mincopter_attitude(float, float, float) { }
void GenericGZInterface::set_mincopter_linvelocity(float, float, float) { }
void GenericGZInterface::set_mincopter_angvelocity(float, float, float) { }

