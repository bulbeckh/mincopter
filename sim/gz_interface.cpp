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
    for (int16_t i=0;i<4;i++) {
		int16_t m_out = mincopter.motors.get_raw_motor_out(i);

		// NOTE The pkt entry is int16_t whereas m_out uint16_t
		control_pkt.pwm[i] = m_out;
		//control_pkt.pwm[i] = 2000;
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
    mc_sim_state_packet* pkt = (mc_sim_state_packet*)buffer;

    // For now, just create a copy of the structure but maybe in future can have a more elegant solution
    // like separate structs for each sensor type
    sensor_states = *pkt;


	// sense check readings
	if (false && frame_counter%100==0) {
		Vector3f gyros = mincopter.ins.get_gyro();
		Vector3f accel = mincopter.ins.get_accel();

		std::cout << "sensor vals: " << mincopter.barometer.get_pressure()
			<< " " << gyros.x << " " << gyros.y << " " << gyros.z
			<< " " << accel.x << " " << accel.y << " " << accel.z << "\n";
	}

	if (false && frame_counter%100==0) {
		std::cout << "baro: " << pkt->pressure << "\n";
	}

	if (false && frame_counter%100==0) {
		std::cout << "packet vel: " << pkt->vel_north << " " << pkt->vel_east << "\n";
		
		//Vector3f vel_vec = mincopter.g_gps->velocity_vector();
		float v_north = mincopter.g_gps->velocity_north();
		std::cout << "lat long " << v_north << " " << mincopter.g_gps->latitude << "\n";
	}

	if (true && frame_counter%100==0) {
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
		std::cout << "(sim/inav) Latitude (deg*1e7): " << (1e7)*pkt->lat_deg << " " << inav_lat << "\n";
		std::cout << "(sim/inav) Longitude(deg*1e7): " << (1e7)*pkt->lng_deg << " " << inav_lng << "\n";
		std::cout << "(sim/inav) Altitude (cm): " << (100)*pkt->alt_met << " " <<  inav_alt << "\n";

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
	if (sensor_states.pressure==0.0f) { 
		pressure = 101322.6f;
	} else {
		pressure = (float)sensor_states.pressure;
	}
}

void GZ_Interface::get_compass_field(Vector3f& field)
{
	/* Both fields are in Tesla */
	field.x = (float)sensor_states.field_x;
	field.y = (float)sensor_states.field_y;
	field.z = (float)sensor_states.field_z;
}

void GZ_Interface::get_imu_gyro_readings(Vector3f& gyro_rate)
{
	gyro_rate.x = (float)sensor_states.imu_gyro_x;
	gyro_rate.y = (float)sensor_states.imu_gyro_y;
	gyro_rate.z = (float)sensor_states.imu_gyro_z;
}

void GZ_Interface::get_imu_accel_readings(Vector3f& accel)
{
	accel.x = (float)sensor_states.imu_accel_x;
	accel.y = (float)sensor_states.imu_accel_y;
	accel.z = (float)sensor_states.imu_accel_z;
}

void GZ_Interface::update_gps_position(int32_t& latitude, int32_t& longitude, int32_t& altitude)
{
	double intermediate_lat = 1e7*sensor_states.lat_deg;
	double intermediate_lng = 1e7*sensor_states.lng_deg;

	latitude = (int32_t)(intermediate_lat);
	longitude = (int32_t)(intermediate_lng);

	/* Simulation altitude is in m but we store in cm */
	altitude = (int32_t)(100*sensor_states.alt_met);

}

void GZ_Interface::update_gps_velocities(int32_t& vel_north, int32_t& vel_east, int32_t& vel_down)
{
	vel_east = (int32_t)(sensor_states.vel_east*100.0f);
	vel_north = (int32_t)(sensor_states.vel_north*100.0f);
	vel_down = (int32_t)(-100.0f*sensor_states.vel_up);

	static int i=0;
	if (i%100==0) {
		std::cout << "VEL " << vel_east << " " << vel_north << " " << vel_down << " - " << sensor_states.vel_east << "  " << sensor_states.vel_north << " " << sensor_states.vel_up << "\n";
		i==0;
	}
	i++;
}
