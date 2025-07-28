
#ifndef __AP_HAL_LINUX_SEMAPHORE_H__
#define __AP_HAL_LINUX_SEMAPHORE_H__

#include <AP_HAL/AP_HAL_Boards.h>

#include <AP_HAL/AP_HAL.h>
#include <arch/linux/AP_HAL_Linux_Namespace.h>

#include <pthread.h>

class Linux::LinuxSemaphore : public AP_HAL::Semaphore {
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

#endif // __AP_HAL_LINUX_SEMAPHORE_H__
