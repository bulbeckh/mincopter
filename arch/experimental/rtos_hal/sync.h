#pragma once

#include <stdint.h>

#include <rtos_hal/types.h>

namespace mc_rtos_hal {

class Mutex {
public:
    virtual ~Mutex() = default;
    virtual Status lock(Timeout timeout) = 0;
    virtual void unlock() = 0;
};

class BinarySemaphore {
public:
    virtual ~BinarySemaphore() = default;
    virtual Status take(Timeout timeout) = 0;
    virtual void give() = 0;
    virtual void give_from_isr() = 0;
};

class EventGroup {
public:
    virtual ~EventGroup() = default;
    virtual uint32_t set_bits(uint32_t bits) = 0;
    virtual uint32_t clear_bits(uint32_t bits) = 0;
    virtual uint32_t wait_bits(uint32_t bits, bool clear_on_exit, bool wait_for_all, Timeout timeout) = 0;
};

}  // namespace mc_rtos_hal
