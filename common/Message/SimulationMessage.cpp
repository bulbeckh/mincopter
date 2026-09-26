
#include "SimulationMessage.h"
#include "ByteReader.h"

#include <iostream>
#include <stdexcept>

namespace mc {

	void write_header(ByteWriter& writer, const SimMessageType& type) {
		writer.write_u16(static_cast<uint16_t>(type));

		switch(type) {
			case SimMessageType::ControlMessage:
				writer.write_u16(sizeof(ControlMessage));
				break;
			case SimMessageType::StateMessage:
				writer.write_u16(sizeof(StateMessage));
				break;
			case SimMessageType::StateUpdateMessage:
				writer.write_u16(sizeof(StateUpdateMessage));
				break;
			case SimMessageType::ForceUpdateMessage:
				writer.write_u16(sizeof(ForceUpdateMessage));
				break;
			default:
				// TODO Change this
				std::cout << "Bad message type inside write_header\n";
				break;
		}

		return;
	}
	
	/* @brief Serialize a ControlMessage into a sequence of bytes */
	void serialize(ByteWriter& writer, const ControlMessage& message) {
		write_header(writer, SimMessageType::ControlMessage);

		writer.write_u16(message.pwm[0]);
		writer.write_u16(message.pwm[1]);
		writer.write_u16(message.pwm[2]);
		writer.write_u16(message.pwm[3]);

		return;
	}

	/* @brief Serialize a StateMessage into a sequence of bytes */
	void serialize(ByteWriter& writer, const StateMessage& message) {
		write_header(writer, SimMessageType::StateMessage);

		writer.write_d(message.timestamp);
		writer.write_u64(message.iterations);

		writer.write_d(message.imu_gyro_x);
		writer.write_d(message.imu_gyro_y);
		writer.write_d(message.imu_gyro_z);

		writer.write_d(message.imu_accel_x);
		writer.write_d(message.imu_accel_y);
		writer.write_d(message.imu_accel_z);

		writer.write_d(message.pos_x);
		writer.write_d(message.pos_y);
		writer.write_d(message.pos_z);

		writer.write_d(message.wldAToBdyA_euler_x);
		writer.write_d(message.wldAToBdyA_euler_y);
		writer.write_d(message.wldAToBdyA_euler_z);

		writer.write_d(message.euler_rate_x);
		writer.write_d(message.euler_rate_y);
		writer.write_d(message.euler_rate_z);

		writer.write_d(message.vel_x);
		writer.write_d(message.vel_y);
		writer.write_d(message.vel_z);

		writer.write_d(message.field_x);
		writer.write_d(message.field_y);
		writer.write_d(message.field_z);

		writer.write_d(message.pressure);

		writer.write_d(message.lat_deg);
		writer.write_d(message.lng_deg);
		writer.write_d(message.alt_met);
		writer.write_d(message.vel_east);
		writer.write_d(message.vel_north);
		writer.write_d(message.vel_up);

		return;
	}

	/* @brief Serialize a StateUpdateMessage into a sequence of bytes */
	void serialize(ByteWriter& writer, const StateUpdateMessage& message) {
		write_header(writer, SimMessageType::StateUpdateMessage);

		std::uint8_t _update_flag{0};
		
		_update_flag |= message.update_flag[0] & 0x01;
		_update_flag |= message.update_flag[1] & 0x02;
		_update_flag |= message.update_flag[2] & 0x04;
		_update_flag |= message.update_flag[3] & 0x08;

		writer.write_u8(_update_flag);

		writer.write_d(message.position[0]);
		writer.write_d(message.position[1]);
		writer.write_d(message.position[2]);

		writer.write_d(message.velocity[0]);
		writer.write_d(message.velocity[1]);
		writer.write_d(message.velocity[2]);

		writer.write_d(message.orientation[0]);
		writer.write_d(message.orientation[1]);
		writer.write_d(message.orientation[2]);

		writer.write_d(message.angular_velocity[0]);
		writer.write_d(message.angular_velocity[1]);
		writer.write_d(message.angular_velocity[2]);

		return;
	};

	/* @brief Serialize a ForceUpdateMessage into a sequence of bytes */
	void serialize(ByteWriter& writer, const ForceUpdateMessage& message) {
		write_header(writer, SimMessageType::ForceUpdateMessage);

		std::uint8_t _update_flag{0};

		_update_flag |= message.update_flag[0] & 0x01;
		_update_flag |= message.update_flag[1] & 0x02;

		writer.write_d(message.force[0]);
		writer.write_d(message.force[1]);
		writer.write_d(message.force[2]);

		writer.write_d(message.torque[0]);
		writer.write_d(message.torque[1]);
		writer.write_d(message.torque[2]);

		return;
	};

	template <>
	const ControlMessage deserialize<ControlMessage>(std::vector<std::byte>& bytes) {
		return ControlMessage{};
	}

	template <>
	const StateMessage deserialize<StateMessage>(std::vector<std::byte>& bytes) {

		ByteReader reader(bytes);

		StateMessage _msg{};

		// TODO In it's current formulation, we assume the message type sent be recv_state_packet
		// to always be a StateMessage and hence have no usage for the MessageHeader struct. Also,
		// the payload size is static and known by both client and server at compile time.

		// TODO Change this to generic read_header method (like write_header)
		auto msg_type = static_cast<mc::SimMessageType>(reader.read_u16());
		auto msg_payload_size = reader.read_u16();

		// Contents
		_msg.timestamp = reader.read_d();
		_msg.iterations = reader.read_u64();

		_msg.imu_gyro_x = reader.read_d();
		_msg.imu_gyro_y = reader.read_d();
		_msg.imu_gyro_z = reader.read_d();

		_msg.imu_accel_x = reader.read_d();
		_msg.imu_accel_y = reader.read_d();
		_msg.imu_accel_z = reader.read_d();

		_msg.pos_x = reader.read_d();
		_msg.pos_y = reader.read_d();
		_msg.pos_z = reader.read_d();

		_msg.wldAToBdyA_euler_x = reader.read_d();
		_msg.wldAToBdyA_euler_y = reader.read_d();
		_msg.wldAToBdyA_euler_z = reader.read_d();

		_msg.euler_rate_x = reader.read_d();
		_msg.euler_rate_y = reader.read_d();
		_msg.euler_rate_z = reader.read_d();

		_msg.vel_x = reader.read_d();
		_msg.vel_y = reader.read_d();
		_msg.vel_z = reader.read_d();

		_msg.field_x = reader.read_d();
		_msg.field_y = reader.read_d();
		_msg.field_z = reader.read_d();

		_msg.pressure = reader.read_d();

		_msg.lat_deg = reader.read_d();
		_msg.lng_deg = reader.read_d();
		_msg.alt_met = reader.read_d();
		_msg.vel_east = reader.read_d();
		_msg.vel_north = reader.read_d();
		_msg.vel_up = reader.read_d();

		return _msg;
	}

	template <>
	const StateUpdateMessage deserialize<StateUpdateMessage>(std::vector<std::byte>& bytes) {
		return StateUpdateMessage{};
	}

	template <>
	const ForceUpdateMessage deserialize<ForceUpdateMessage>(std::vector<std::byte>& bytes) {
		return ForceUpdateMessage{};
	}

}

