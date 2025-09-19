/* Functions for communication between gazebo and this software simulation
 *
 * Structs taken from <add link to ardupilot_gazebo>
 *
 *
 */

#include <arch/linux/generic/gz_interface.h>

#include <iostream>
#include <cstring>
#include <string.h>

#include <sys/socket.h>
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
	
	send_control_output();

	recv_state_input();

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

	if (true /* Use assigned PWM signals directly from MPC mixer*/ ) {

		/*
		control_pkt.pwm[0] = control_pwm[0];
		control_pkt.pwm[1] = control_pwm[1];
		// NOTE Deliberately switched
		control_pkt.pwm[2] = control_pwm[3];
		control_pkt.pwm[3] = control_pwm[2];
		*/

		control_pkt.pwm[0] = hal.sim->motor_out[0];
		control_pkt.pwm[1] = hal.sim->motor_out[1]; 
		control_pkt.pwm[2] = hal.sim->motor_out[3];
		control_pkt.pwm[3] = hal.sim->motor_out[2];

		/*
		control_pkt.pwm[0] = 0;
		control_pkt.pwm[1] = 0;
		control_pkt.pwm[2] = 2000;
		control_pkt.pwm[3] = 2000;
		*/

	} else {
		/*
		control_pkt.pwm[0] = mincopter.motors.get_raw_motor_out(0);
		control_pkt.pwm[1] = mincopter.motors.get_raw_motor_out(1);
		// NOTE This has been changed in order to align with the new motor orientation as per AP_MotorsQuad
		control_pkt.pwm[2] = mincopter.motors.get_raw_motor_out(3);
		control_pkt.pwm[3] = mincopter.motors.get_raw_motor_out(2);
		*/
		// NOTE Temporarily sending max servo to determine the acceleration rate
		
		static uint32_t send_pwm=1100;
		control_pkt.pwm[0] = send_pwm;
		control_pkt.pwm[1] = send_pwm;
		control_pkt.pwm[2] = send_pwm;
		control_pkt.pwm[3] = send_pwm;
	}


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


	// sense check readings
	/*
	if (false && frame_counter%100==0) {
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
	if (true && frame_counter%100==0) {
		Vector3f inav_pos = mcstate.get_position();
		std::cout << "POS x,y,z r/p/y: " << pkt->pos_x << "," << pkt->pos_y << "," << pkt->pos_z << " " << pkt->wldAbdyA_eul_x << "/" << pkt->wldAbdyA_eul_y << "/" << pkt->wldAbdyA_eul_z << "\n";
		//std::cout << "INAV x-y () : " << inav_pos.x << " " << inav_pos.y << "\n";
	}
		double wldAbdyA_eul_x; // Roll
		double wldAbdyA_eul_y; // Pitch
		double wldAbdyA_eul_z; // Yaw

	if (false && frame_counter%1000==0) {
		std::cout << "GPS Vel North / Vel East / Vel Up: " << pkt->vel_north << " "
			<< pkt->vel_east << " "
			<< pkt->vel_up << "\n";
	
		std::cout << "GPS Lat / Lng / Alt (m) : " 
			<< pkt->lat_deg << " "
			<< pkt->lng_deg << " "
			<< pkt->alt_met << "\n";
	}
	*/

	//if (false && frame_counter%1000==0) {
		/* pkt->pos_<x,y,z> is the simulated position in metres
		 *
		 *
		 *
		 */
		
		/* The inertial nav retrieves position in cm so we multiply by 0.01 to get in m. This
		 * is a position **estimate** by the inav relative to the home location. The home location
		 * is set during a call to mcstate.inertial_nav.set_home_position during the init_home function
		 * which is called during arming. In the simulation, this should be the (0,0,0) position. */

		/*
		Vector3f inav_pos = mcstate.get_position();
		inav_pos *= 0.01;

		int32_t inav_lat = mcstate.get_latitude();
		int32_t inav_lng = mcstate.get_longitude();
		int32_t inav_alt = mcstate.get_altitude();

		// TODO Get heading from mcstate instead
		//float c_heading = degrees(mincopter.compass.calculate_heading(mcstate.get_dcm()));
		float c_heading = 0.0f; 

		std::cout << "---------- ITERATION " << frame_counter << " ----\n";
		std::cout << "(sim/inav) Position X (m): " << pkt->pos_x << " " << inav_pos.x << "\n";
		std::cout << "(sim/inav) Position Y (m): " << pkt->pos_y << " " << inav_pos.y << "\n";
		std::cout << "(sim/inav) Position Z (m): " << pkt->pos_z << " " << inav_pos.z << "\n";
		std::cout << "(sim/ahrs) Rotation (Roll) (deg): " << degrees(pkt->wldAbdyA_eul_x) << " " << 0.01f*mcstate.roll_sensor << "\n";
		std::cout << "(sim/ahrs) Rotation (Pitch) (deg): " << degrees(pkt->wldAbdyA_eul_y) << " " << 0.01f*mcstate.pitch_sensor << "\n";
		std::cout << "(sim/ahrs) Rotation (Yaw) (deg): " << degrees(pkt->wldAbdyA_eul_z) << " " << 0.01f*wrap_360_cd(mcstate.yaw_sensor) << "\n";
		std::cout << "(compass)  Heading (deg)       : " << c_heading << "\n";
		std::cout << "(sim/inav) Latitude (deg*1e7): " << (int32_t)((1e7)*pkt->lat_deg) << " " << inav_lat << "\n";
		std::cout << "(sim/inav) Longitude(deg*1e7): " << (int32_t)((1e7)*pkt->lng_deg) << " " << inav_lng << "\n";
		std::cout << "(sim/inav) Altitude (cm): " << (int32_t)((100)*pkt->alt_met) << " " <<  inav_alt << "\n";
		std::cout << "timestamp " << pkt->timestamp << "\n";

		std::cout << "INAV   POS: " << inav_pos.x << " " << inav_pos.y << " " << inav_pos.z << "\n";
		std::cout << "ERROR     : " << pkt->pos_x - inav_pos.x << " " << pkt->pos_y - inav_pos.y << " " << pkt->pos_z - inav_pos.z << "\n";
		std::cout << "ACTUAL GPS (deg*1e7, ded*1e7, cm): " << (1e7)*pkt->lat_deg << " " << (1e7)*pkt->lng_deg << " " << (100)*pkt->alt_met << "\n";
		std::cout << "INAV   GPS: 						 " << inav_lat << " " << inav_lng << " " << mcstate.inertial_nav.get_altitude() << "\n";
		*/


    return true;
}

void GenericGZInterface::reset(void)
{
	// TODO Implement
	
	return;
}

void GenericGZInterface::set_mincopter_pose(float x_ned_m, float y_ned_m, float z_ned_m, float roll_rad, float pitch_rad, float yaw_rad)
{
	// TODO Implement
	
	return;
}

// TODO Move all of this conversion/rotation code into the dev/ specific sim drivers. This class should just retrieve raw simulation state from Gazebo

/*
void GenericGZInterface::get_barometer_pressure(float& pressure)
{
	// Both in Pascals so no need for unit conversion

	// NOTE During barometer calibration, it checks for a non-zero pressure reading from the sensor
	// and will trigger a HAL panic if it doesn't get one. Sometimes in the early stage of the simulation
	// the pressure will be 0 before it has started so we need to catch this and fake a ground pressure reading.
	if (last_sensor_state.pressure==0.0f) { 
		pressure = 101322.6f;
	} else {
		pressure = (float)(last_sensor_state.pressure);
	}
}

void GenericGZInterface::get_compass_field(Vector3f& field)
{
	// We average the compass reads
	
	// Both fields are in Tesla
	field.x = (float)last_sensor_state.field_x;
	field.y = (float)last_sensor_state.field_y;
	field.z = (float)last_sensor_state.field_z;
}

void GenericGZInterface::get_imu_gyro_readings(Vector3f& gyro_rate)
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

void GenericGZInterface::get_imu_accel_readings(Vector3f& accel)
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

void GenericGZInterface::update_gps_position(int32_t& latitude, int32_t& longitude, int32_t& altitude)
{
	double intermediate_lat = 1e7*last_sensor_state.lat_deg;
	double intermediate_lng = 1e7*last_sensor_state.lng_deg;

	latitude = (int32_t)(intermediate_lat);
	longitude = (int32_t)(intermediate_lng);

	// Simulation altitude is in m but we store in cm
	altitude = (int32_t)(100*last_sensor_state.alt_met);

}

void GenericGZInterface::update_gps_velocities(int32_t& vel_north, int32_t& vel_east, int32_t& vel_down)
{
	vel_east = (int32_t)(last_sensor_state.vel_east*100.0f);
	vel_north = (int32_t)(last_sensor_state.vel_north*100.0f);
	vel_down = (int32_t)(-100.0f*last_sensor_state.vel_up);

}

*/


