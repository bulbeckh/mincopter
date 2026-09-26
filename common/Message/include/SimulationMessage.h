
// Gazebo Simulation <-> MinCopter message definitions and interface

#pragma once

// TODO This should probably form part of the AP_HAL::Sim abstraction as the message interface should
// be independent of simulator choice

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

#include "ByteWriter.h"

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

	struct ControlMessage {
		uint16_t pwm[4];
	};

	struct StateUpdateMessage {
		/* @brief Flag indicating which of the four state types we need to update */
		bool update_flag[4];

		double position[3];
		double velocity[3];
		double orientation[3];
		double angular_velocity[3];
	};

	struct ForceUpdateMessage {
		bool update_flag[2];

		double force[3];
		double torque[3];
	};

	// TODO Include flags to mark certain sensor readings as valid/invalid, last read time, etc. - on the
	// simulation side, we have a callback that populates things like IMU readings so there may be a
	// period of time in which certain readings are invalid.
	
	/* @brief Contents of StateMessage that is sent by Gazebo to MinCopter each iteration */
	struct StateMessage {
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

	// Message Serialization
	void write_header(ByteWriter& writer, const SimMessageType& type);
	void serialize(ByteWriter& writer, const ControlMessage& message);
	void serialize(ByteWriter& writer, const StateMessage& message);
	void serialize(ByteWriter& writer, const StateUpdateMessage& message);
	void serialize(ByteWriter& writer, const ForceUpdateMessage& message);

	// TODO Change the way we do this - we do not want to allow deserializations of arbitrary types
	template <typename T>
	const T deserialize(std::vector<std::byte>& bytes);
	
	/* The message creation pipeline should be something like
	 *
	 * ```c++
	 * ControlMessage cmessage;
	 *
	 * cmessage.pwm[0] = 1000;
	 * cmessage.pwm[1] = 1000;
	 * cmessage.pwm[2] = 1000;
	 * cmessage.pwm[3] = 1000;
	 *
	 * socket.send_message(cmessage);
	 * ```
	 *
	 * Then the socket send_message function is responsible for serialization of the message
	 * and then transmission of the sequence of bytes to the (UNIX domain) socket.
	 *
	 * ```c++
	 * template <typename T>
	 * void SocketUnix::send_message(const T& message) {
	 *
	 * 	ByteWriter writer; 
	 * 	std::vector<std::byte> bytes = serialize(writer, message);
	 * 	
	 *	// ... Do send
	 * }
	 * ```
	 *
	 */



} // namespace mc

