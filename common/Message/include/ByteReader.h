
#pragma once

#include <cstdint>
#include <vector>
#include <cstring>
#include <cstddef>
#include <stdexcept>

#include <iostream>

class ByteReader {
	public:
		explicit ByteReader(std::vector<std::byte>& data) :
			_data{data} { }

	public:
		/* @brief Read a single byte of data from stream */
		std::uint8_t read_u8() {
			check_size(1);

			std::uint8_t val = static_cast<std::uint8_t>(_data[_offset]);

			_offset += 1;
			return val;
		}

		std::uint16_t read_u16() {
			check_size(2);

			std::uint16_t val = 
				(static_cast<std::uint16_t>(_data[_offset]) << 8) |
				 static_cast<std::uint16_t>(_data[_offset+1]);

			_offset += 2;
			return val;
		}

		std::uint32_t read_u32() {
			check_size(4);

			std::uint32_t val =
				(static_cast<std::uint32_t>(_data[_offset]) << 24) |
				(static_cast<std::uint32_t>(_data[_offset+1]) << 16) |
				(static_cast<std::uint32_t>(_data[_offset+2]) << 8) |
				static_cast<std::uint32_t>(_data[_offset+3]);

			_offset += 4;
			return val;
		}

		std::uint64_t read_u64() {
			check_size(8);

			std::uint64_t val =
				(static_cast<std::uint64_t>(_data[_offset]) << 56) |
				(static_cast<std::uint64_t>(_data[_offset+1]) << 48) |
				(static_cast<std::uint64_t>(_data[_offset+2]) << 40) |
				(static_cast<std::uint64_t>(_data[_offset+3]) << 32) |
				(static_cast<std::uint64_t>(_data[_offset+4]) << 24) |
				(static_cast<std::uint64_t>(_data[_offset+5]) << 16) |
				(static_cast<std::uint64_t>(_data[_offset+6]) << 8) |
				static_cast<std::uint64_t>(_data[_offset+7]);

			_offset += 8;
			return val;

		}

		float read_f32() {
			std::uint32_t bits = read_u32();

			float val;
			std::memcpy(&val, &bits, sizeof(float));

			return val;
		}

		double read_d() {
			std::uint64_t bits = read_u64();

			double val;

			std::memcpy(&val, &bits, sizeof(double));

			return val;
		}

	private:
		/* @brief Check that we have enough bytes remaining in our data vector */
		void check_size(std::size_t count) const {
			if (_offset + count > _data.size()) {
				throw std::runtime_error("Not enough bytes remaining in vector");
			}
		}


	private:
		const std::vector<std::byte>& _data;
		std::size_t _offset{0};

};

