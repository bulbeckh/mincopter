#pragma once

#include <stddef.h>
#include <stdint.h>

#include <rtos_hal/time.h>
#include <rtos_hal/uart.h>

#include <dev/experimental/gps/GpsDevice.h>

namespace mc_experimental {

struct UbxNeoGpsConfig {
    size_t uart_index;
    uint32_t baud_rate;
    uint32_t expected_fix_rate_hz;
    uint16_t rx_chunk_size;
};

class UbxNeoGpsDevice final : public GpsDevice {
public:
    UbxNeoGpsDevice(mc_rtos_hal::UartPort &uart,
                    mc_rtos_hal::Time &time,
                    const UbxNeoGpsConfig &config);

    bool init() override;
    bool configure() override;
    uint32_t expected_fix_rate_hz() const override;
    bool service(GpsFix &fix, bool &fix_updated) override;

private:
    struct ubx_header {
        uint8_t preamble1;
        uint8_t preamble2;
        uint8_t msg_class;
        uint8_t msg_id;
        uint16_t length;
    };

    struct ubx_nav_posllh {
        uint32_t time;
        int32_t longitude;
        int32_t latitude;
        int32_t altitude_ellipsoid;
        int32_t altitude_msl;
        uint32_t horizontal_accuracy;
        uint32_t vertical_accuracy;
    };

    struct ubx_nav_status {
        uint32_t time;
        uint8_t fix_type;
        uint8_t fix_status;
        uint8_t differential_status;
        uint8_t res;
        uint32_t time_to_first_fix;
        uint32_t uptime;
    };

    struct ubx_nav_velned {
        uint32_t time;
        int32_t ned_north;
        int32_t ned_east;
        int32_t ned_down;
        uint32_t speed_3d;
        uint32_t speed_2d;
        int32_t heading_2d;
        uint32_t speed_accuracy;
        uint32_t heading_accuracy;
    };

    union MessageBuffer {
        ubx_nav_posllh posllh;
        ubx_nav_status status;
        ubx_nav_velned velned;
        uint8_t bytes[128];
    };

    bool read_chunk(uint8_t *buffer, size_t capacity, size_t &read_len);
    bool process_byte(uint8_t byte, GpsFix &fix, bool &fix_updated);
    bool parse_message(GpsFix &fix, bool &fix_updated);
    void update_checksum(uint8_t data);

private:
    enum {
        PREAMBLE1 = 0xB5,
        PREAMBLE2 = 0x62,
        CLASS_NAV = 0x01,
        CLASS_CFG = 0x06,
        MSG_POSLLH = 0x02,
        MSG_STATUS = 0x03,
        MSG_VELNED = 0x12,
    };

    enum {
        NAV_STATUS_FIX_VALID = 1,
        FIX_NONE = 0,
        FIX_2D = 2,
        FIX_3D = 3,
    };

    mc_rtos_hal::UartPort &uart_;
    mc_rtos_hal::Time &time_;
    UbxNeoGpsConfig config_;

    uint8_t step_;
    uint8_t msg_class_;
    uint8_t msg_id_;
    uint16_t payload_length_;
    uint16_t payload_counter_;
    uint8_t ck_a_;
    uint8_t ck_b_;
    MessageBuffer buffer_;

    GpsFix latest_fix_;
    GpsFixType next_fix_;
    bool new_position_;
    bool new_speed_;
    uint32_t sequence_;
};

}  // namespace mc_experimental
