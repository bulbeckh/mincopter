
// Gazebo Simulation <-> MinCopter message definitions and interface

#pragma once

/* Here we document the full message interface between Gazebo and MinCopter
 *
 * For a single iteration of the simulation loop, we send a control packet from MinCopter
 * and then receive a state packet back from Gazebo.
 *
 * MinCopter can send the following messages to Gazebo:
 *
 * 1. ControlMessage
 * This is a standard message and includes four PWM signals to apply to each of the motors in Gazebo.
 * This message indicates that MinCopter will not send more messages this iteration and will block
 * until Gazebo sends back a state packet.
 *
 * 2. StateUpdateMessage
 * This is a request from MinCopter to update some element of the simulation state directly, specifically
 * either position, linear velocity, orientation, or angular velocity. These messages include a bit flag
 * that specifies which of the four we will be requesting be updated, as well as multiple messages containing
 * the values that we want to update.
 *
 * 3. ForceUpdateMessage
 * This is a request from MinCopter to apply a specific force or torque to the simulation state directly.
 * Implementation is similar to the above (StateUpdateMessage).
 *
 * At each simulation iteration, we send at most one of the above messages. As such, the receival of a message
 * by the Gazebo plugin will cause it to iterate the simulation and eventually send a state message.
 *
 * Gazebo sends the following message to MinCopter:
 *
 * 1. StateMessage
 * Sent after the last update and contains various information about the simulation state, including sensor
 * readings, iteration information, timing information.
 *
 * Although not part of the message interface, we may call server.ResetAll() from a test case, which our
 * ArduPilotPlugin needs to be able to respond to and reset its internal state.
 *
 */

#include <cstdint>

namespace mc {

	enum class SimMessageType : uint16_t {
		// Sent by MinCopter
		ControlMessage = 1,
		StateUpdateMessage = 2,
		ForceUpdateMessage = 3,

		// Sent by Gazebo
		StateMessage = 9,
	};

	struct SimMessageHeader {
		uint16_t type;
		uint16_t payload_size;
	};

	struct ControlMessagePayload {
		uint16_t pwm[4];
	};

	struct StateUpdateMessagePayload {
		/* @brief Flag indicating which of the four state types we need to update */
		bool update_flag[4];

		double position[3];
		double velocity[3];
		double orientation[3];
		double angular_velocity[3];
	};

	struct ForceUpdateMessagePayload {
		bool update_flag[2];

		double force[3];
		double torque[3];
	};

	/* @brief Contents of StateMessage that is sent by Gazebo to MinCopter each iteration */
	struct StateMessagePayload {
		// Information
		double timestamp;
		uint64_t iterations;

		// IMU
		double imu_gyro_x;
		double imu_gyro_y;
		double imu_gyro_z;

		double imu_accel_x;
		double imu_accel_y;
		double imu_accel_z;

		// State Information
		double pos_x;
		double pos_y;
		double pos_z;

		double wldAToBdyA_euler_x;
		double wldAToBdyA_euler_y;
		double wldAToBdyA_euler_z;

		double euler_rate_x;
		double euler_rate_y;
		double euler_rate_z;

		double vel_x;
		double vel_y;
		double vel_z;

		// Compass
		double field_x;
		double field_y;
		double field_z;

		// Barometer
		double pressure;

		// NavSat (ENU form)
		double lat_deg;
		double lng_deg;
		double alt_met;
		double vel_east;
		double vel_north;
		double vel_up;
	};

	struct ControlMessage {
		SimMessageHeader header;
		ControlMessagePayload payload;
	};

	struct StateUpdateMessage {
		SimMessageHeader header;
		StateUpdateMessage payload;
	};

	struct ForceUpdateMessage {
		SimMessageHeader header;
		ForceUpdateMessage payload;
	};

	struct StateMessage {
		SimMessageHeader header;
		StateMessagePayload payload;
	}


} // namespace mc

