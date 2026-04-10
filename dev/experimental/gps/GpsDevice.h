#pragma once

#include <stdint.h>

#include <rtos_hal/time.h>
#include <rtos_hal/uart.h>

namespace mc_experimental {

enum class GpsFixType : uint8_t {
    None = 0,
    Fix2D = 2,
    Fix3D = 3,
};

struct GpsFix {
    uint32_t timestamp_us;
    uint32_t sequence;
    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t altitude_cm;
    uint32_t ground_speed_cm_s;
    int32_t vel_north_cm_s;
    int32_t vel_east_cm_s;
    int32_t vel_down_cm_s;
    uint16_t heading_cd;
    uint8_t satellites;
    GpsFixType fix_type;
    bool valid;
};

class GpsDevice {
public:
    virtual ~GpsDevice() = default;

    virtual bool init() = 0;
    virtual bool configure() = 0;
    virtual uint32_t expected_fix_rate_hz() const = 0;
    virtual bool service(GpsFix &fix, bool &fix_updated) = 0;
};

}  // namespace mc_experimental
