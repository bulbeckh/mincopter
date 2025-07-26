
#pragma once

#include <AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_RPI.h>
#include <pthread.h>

class RPI::LinuxSemaphore : public AP_HAL::Semaphore {
public:
    LinuxSemaphore() {
        pthread_mutex_init(&_lock, NULL);
    }
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();
private:
    pthread_mutex_t _lock;
};
#endif // CONFIG_HAL_BOARD

