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

#include "mcstate.h"
extern MCState mcstate;

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

    for (int16_t i=0;i<16;i++) {
		int16_t m_out = mincopter.motors.get_raw_motor_out(i);

		// NOTE The pkt entry is int16_t whereas m_out uint16_t
		//control_pkt.pwm[i] = m_out;
		// Send minimum
		control_pkt.pwm[i] = 0;
    }

	control_pkt.pwm[0] = mincopter.motors.get_raw_motor_out(0);
	control_pkt.pwm[1] = mincopter.motors.get_raw_motor_out(1);
	control_pkt.pwm[2] = mincopter.motors.get_raw_motor_out(2);
	control_pkt.pwm[3] = mincopter.motors.get_raw_motor_out(3);

	/*
	control_pkt.pwm[0] = 1200;
	control_pkt.pwm[1] = 0;
	control_pkt.pwm[2] = 0;
	control_pkt.pwm[3] = 0;
	*/

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
	    //std::cout << "Motor " << i << ": " << mincopter.motors.get_raw_motor_out(i) << ", ";
	}
	//std::cout << "\n";
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
    mc_sim_state_packet* pkt = (mc_sim_state_packet*)buffer;

    // For now, just create a copy of the structure but maybe in future can have a more elegant solution
    // like separate structs for each sensor type
    sensor_states[state_buffer_index] = *pkt;
	
	// For the IMU sensor, we need to simulate the DLPF by updating a proportion of the previous filter reading
	uint8_t temp_idx=0;
	if (state_buffer_index==0) {
		temp_idx=GZ_INTERFACE_STATE_BUFFER_LENGTH-1;
	} else {
		temp_idx = state_buffer_index-1;
	}
	float alpha=0.5;
	sensor_states[state_buffer_index].imu_gyro_x = sensor_states[temp_idx].imu_gyro_x*alpha + sensor_states[state_buffer_index].imu_gyro_x*(1.0f-alpha);
	sensor_states[state_buffer_index].imu_gyro_y = sensor_states[temp_idx].imu_gyro_y*alpha + sensor_states[state_buffer_index].imu_gyro_y*(1.0f-alpha);
	sensor_states[state_buffer_index].imu_gyro_z = sensor_states[temp_idx].imu_gyro_z*alpha + sensor_states[state_buffer_index].imu_gyro_z*(1.0f-alpha);

	sensor_states[state_buffer_index].imu_accel_x = sensor_states[temp_idx].imu_accel_x*alpha + sensor_states[state_buffer_index].imu_accel_x*(1.0f-alpha);
	sensor_states[state_buffer_index].imu_accel_y = sensor_states[temp_idx].imu_accel_y*alpha + sensor_states[state_buffer_index].imu_accel_y*(1.0f-alpha);
	sensor_states[state_buffer_index].imu_accel_z = sensor_states[temp_idx].imu_accel_z*alpha + sensor_states[state_buffer_index].imu_accel_z*(1.0f-alpha);

	state_buffer_index += 1;
	state_buffer_index %= GZ_INTERFACE_STATE_BUFFER_LENGTH;

	last_sensor_state = *pkt;


	// sense check readings
	if (false && frame_counter%1000==0) {
		std::cout << " Timestamp (s): " << pkt->timestamp << " ********************************\n";

		std::cout << "IMU Gyro x/y/z : "
			<< pkt->imu_gyro_x << " "
			<< pkt->imu_gyro_y << " "
			<< pkt->imu_gyro_z << "\n";
		std::cout << "IMU Accel x/y/z/ : "
			<< pkt->imu_accel_x << " "
			<< pkt->imu_accel_y << " "
			<< pkt->imu_accel_z << "\n";
	}

	if (false && frame_counter%1000==0) {
		std::cout << "COMP Field x/y/z : "
			<< pkt->field_x << " "
			<< pkt->field_y << " "
			<< pkt->field_z << "\n";
	}

	if (false && frame_counter%1000==0) {
		// TODO Add Temperature sensor to SDF
		std::cout << "BARO Pressure (Pa) : " << pkt->pressure << "\n";
	}

	if (false && frame_counter%1000==0) {
		std::cout << "GPS Vel North / Vel East / Vel Up: " << pkt->vel_north << " "
			<< pkt->vel_east << " "
			<< pkt->vel_up << "\n";
	
		std::cout << "GPS Lat / Lng / Alt (m) : " 
			<< pkt->lat_deg << " "
			<< pkt->lng_deg << " "
			<< pkt->alt_met << "\n";
	}

	if (true && frame_counter%1000==0) {
		/* pkt->pos_<x,y,z> is the simulated position in metres
		 *
		 *
		 *
		 */
		
		/* The inertial nav retrieves position in cm so we multiply by 0.01 to get in m. This
		 * is a position **estimate** by the inav relative to the home location. The home location
		 * is set during a call to mcstate.inertial_nav.set_home_position during the init_home function
		 * which is called during arming. In the simulation, this should be the (0,0,0) position. */
		Vector3f inav_pos = mcstate.inertial_nav.get_position();
		inav_pos *= 0.01;

		int32_t inav_lat = mcstate.inertial_nav.get_latitude();
		int32_t inav_lng = mcstate.inertial_nav.get_longitude();
		int32_t inav_alt = mcstate.inertial_nav.get_altitude();

		std::cout << "---------- ROUND " << frame_counter << " ----\n";
		std::cout << "(sim/inav) Position X (m): " << pkt->pos_x << " " << inav_pos.x << "\n";
		std::cout << "(sim/inav) Position Y (m): " << pkt->pos_y << " " << inav_pos.y << "\n";
		std::cout << "(sim/inav) Position Z (m): " << pkt->pos_z << " " << inav_pos.z << "\n";
		std::cout << "(sim/inav) Latitude (deg*1e7): " << (int32_t)((1e7)*pkt->lat_deg) << " " << inav_lat << "\n";
		std::cout << "(sim/inav) Longitude(deg*1e7): " << (int32_t)((1e7)*pkt->lng_deg) << " " << inav_lng << "\n";
		std::cout << "(sim/inav) Altitude (cm): " << (int32_t)((100)*pkt->alt_met) << " " <<  inav_alt << "\n";

		/*
		std::cout << "INAV   POS: " << inav_pos.x << " " << inav_pos.y << " " << inav_pos.z << "\n";
		std::cout << "ERROR     : " << pkt->pos_x - inav_pos.x << " " << pkt->pos_y - inav_pos.y << " " << pkt->pos_z - inav_pos.z << "\n";
		std::cout << "ACTUAL GPS (deg*1e7, ded*1e7, cm): " << (1e7)*pkt->lat_deg << " " << (1e7)*pkt->lng_deg << " " << (100)*pkt->alt_met << "\n";
		std::cout << "INAV   GPS: 						 " << inav_lat << " " << inav_lng << " " << mcstate.inertial_nav.get_altitude() << "\n";
		*/

	}

    return true;
}

void GZ_Interface::get_barometer_pressure(float& pressure)
{
	/* Both in Pascals so no need for unit conversion */

	// NOTE During barometer calibration, it checks for a non-zero pressure reading from the sensor
	// and will trigger a HAL panic if it doesn't get one. Sometimes in the early stage of the simulation
	// the pressure will be 0 before it has started so we need to catch this and fake a ground pressure reading.
	if (last_sensor_state.pressure==0.0f) { 
		pressure = 101322.6f;
	} else {
		pressure = (float)(last_sensor_state.pressure);
	}
}

void GZ_Interface::get_compass_field(Vector3f& field)
{
	// We average the compass reads
	
	/* Both fields are in Tesla */
	field.x = (float)last_sensor_state.field_x;
	field.y = (float)last_sensor_state.field_y;
	field.z = (float)last_sensor_state.field_z;
}

void GZ_Interface::get_imu_gyro_readings(Vector3f& gyro_rate)
{
	// We average the IMU and gyro readings
	
	double avg_imu_gyro_x=0;
	double avg_imu_gyro_y=0;
	double avg_imu_gyro_z=0;
	for (int i=0;i<GZ_INTERFACE_STATE_BUFFER_LENGTH;i++) {
		avg_imu_gyro_x += sensor_states[i].imu_gyro_x;
		avg_imu_gyro_y += sensor_states[i].imu_gyro_y;
		avg_imu_gyro_z += sensor_states[i].imu_gyro_z;
	}

	gyro_rate.x = (float)(avg_imu_gyro_x / GZ_INTERFACE_STATE_BUFFER_LENGTH);
	gyro_rate.y = (float)(avg_imu_gyro_y / GZ_INTERFACE_STATE_BUFFER_LENGTH);
	gyro_rate.z = (float)(avg_imu_gyro_z / GZ_INTERFACE_STATE_BUFFER_LENGTH);
}

void GZ_Interface::get_imu_accel_readings(Vector3f& accel)
{
	double avg_imu_accel_x=0;
	double avg_imu_accel_y=0;
	double avg_imu_accel_z=0;
	for (int i=0;i<GZ_INTERFACE_STATE_BUFFER_LENGTH;i++) {
		avg_imu_accel_x += sensor_states[i].imu_accel_x;
		avg_imu_accel_y += sensor_states[i].imu_accel_y;
		avg_imu_accel_z += sensor_states[i].imu_accel_z;
	}

	accel.x = (float)(avg_imu_accel_x / GZ_INTERFACE_STATE_BUFFER_LENGTH);
	accel.y = (float)(avg_imu_accel_y / GZ_INTERFACE_STATE_BUFFER_LENGTH);
	accel.z = (float)(avg_imu_accel_z / GZ_INTERFACE_STATE_BUFFER_LENGTH);
}

void GZ_Interface::update_gps_position(int32_t& latitude, int32_t& longitude, int32_t& altitude)
{
	double intermediate_lat = 1e7*last_sensor_state.lat_deg;
	double intermediate_lng = 1e7*last_sensor_state.lng_deg;

	latitude = (int32_t)(intermediate_lat);
	longitude = (int32_t)(intermediate_lng);

	/* Simulation altitude is in m but we store in cm */
	altitude = (int32_t)(100*last_sensor_state.alt_met);

}

void GZ_Interface::update_gps_velocities(int32_t& vel_north, int32_t& vel_east, int32_t& vel_down)
{
	vel_east = (int32_t)(last_sensor_state.vel_east*100.0f);
	vel_north = (int32_t)(last_sensor_state.vel_north*100.0f);
	vel_down = (int32_t)(-100.0f*last_sensor_state.vel_up);

}
