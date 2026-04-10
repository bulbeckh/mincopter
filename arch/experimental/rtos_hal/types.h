#pragma once

#include <stddef.h>
#include <stdint.h>

namespace mc_rtos_hal {

enum class Status : uint8_t {
    Ok = 0,
    Error,
    Timeout,
    Busy,
    Unsupported,
    InvalidArgument,
};

struct Timeout {
    uint32_t ms;
};

enum class PinMode : uint8_t {
    Input = 0,
    Output,
    Alternate,
    Analog,
};

enum class Edge : uint8_t {
    Rising = 0,
    Falling,
    Both,
};

enum class TaskPriority : uint8_t {
    Lowest = 0,
    Low,
    Medium,
    High,
    Highest,
};

}  // namespace mc_rtos_hal
