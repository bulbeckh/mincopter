
#pragma once

#include <stdint.h>
#include <netinet/in.h>
#include <AP_Math.h>

/* @brief The buffer length used to buffer readings from the simulation */
#define GZ_INTERFACE_STATE_BUFFER_LENGTH 50

class GZ_Interface {
    public:
	
	/* @brief Defines a socket connection between the Gazebo simulation and the mincopter runtime
	 */
	GZ_Interface() : frame_counter(0)
	{
	}

    public:
	struct mc_sim_state_packet {
	    double timestamp;
	    double imu_gyro_x;
	    double imu_gyro_y;
	    double imu_gyro_z;
	    double imu_accel_x;
	    double imu_accel_y;
	    double imu_accel_z;
	    double pos_x;
	    double pos_y;
	    double pos_z;
	    double vel_x;
	    double vel_y;
	    double vel_z;
	    double field_x;
	    double field_y;
	    double field_z;
	    double pressure;

		/* NavSat (GPS */
		double lat_deg;
		double lng_deg;
		double alt_met;
		double vel_east;
		double vel_north;
		double vel_up;

	};

	/* @brief The struct containing all sensor information. This is accessed by each of the sim_* 
	 * simulated sensor classes */
	mc_sim_state_packet sensor_states[GZ_INTERFACE_STATE_BUFFER_LENGTH];

	mc_sim_state_packet last_sensor_state;

	/* @brief The index in the buffer that we will next read sensor states to */
	uint8_t state_buffer_index=0;


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
		/* @brief Set up UDP socket between this and GZ server process */
		bool setup_sim_socket();

		/* @brief Send a motor control output PWM */
		bool send_control_output();

		/* @brief Receive, parse, and store a GZ simulation state packet */
		bool recv_state_input();

	public:
		/* @brief Get barometer pressure */
		void get_barometer_pressure(float& pressure);

		/* @brief Get magnetic field readings */
		void get_compass_field(Vector3f& field);

		/* @brief Get IMU gyro rates */
		void get_imu_gyro_readings(Vector3f& gyro_rate);

		/* @brief Get IMU accelerometer readings */
		void get_imu_accel_readings(Vector3f& accel);

		/* @brief Update GPS position x3 */
		void update_gps_position(int32_t& latitude, int32_t& longitude, int32_t& altitude);
		
		/* @brief Update GPS velocity x3 */
		void update_gps_velocities(int32_t& vel_north, int32_t& vel_east, int32_t& vel_down);





		

};


