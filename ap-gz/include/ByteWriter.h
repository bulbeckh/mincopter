#include <bit>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

class ByteWriter {
public:
    void write_u16(std::uint16_t value)
    {
        buffer_.push_back(static_cast<std::byte>((value >> 8) & 0xff));
        buffer_.push_back(static_cast<std::byte>(value & 0xff));
    }

    void write_u32(std::uint32_t value)
    {
        buffer_.push_back(static_cast<std::byte>((value >> 24) & 0xff));
        buffer_.push_back(static_cast<std::byte>((value >> 16) & 0xff));
        buffer_.push_back(static_cast<std::byte>((value >> 8) & 0xff));
        buffer_.push_back(static_cast<std::byte>(value & 0xff));
    }

    void write_u64(std::uint64_t value)
    {
        buffer_.push_back(static_cast<std::byte>((value >> 56) & 0xff));
        buffer_.push_back(static_cast<std::byte>((value >> 48) & 0xff));
        buffer_.push_back(static_cast<std::byte>((value >> 40) & 0xff));
        buffer_.push_back(static_cast<std::byte>((value >> 32) & 0xff));

        buffer_.push_back(static_cast<std::byte>((value >> 24) & 0xff));
        buffer_.push_back(static_cast<std::byte>((value >> 16) & 0xff));
        buffer_.push_back(static_cast<std::byte>((value >> 8) & 0xff));
        buffer_.push_back(static_cast<std::byte>(value & 0xff));

    }

    void write_f32(float value)
    {
        static_assert(sizeof(float) == sizeof(std::uint32_t));

	std::uint32_t bits;
	std::memcpy(&bits, &value, sizeof(float));

        write_u32(bits);
    }

    void write_d64(double value)
    {
        static_assert(sizeof(double) == sizeof(std::uint64_t));

	std::uint64_t bits;
	std::memcpy(&bits, &value, sizeof(double);

        write_u64(bits);
    }


    const std::vector<std::byte>& data() const
    {
        return buffer_;
    }

private:
    std::vector<std::byte> buffer_;
};

