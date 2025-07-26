
#ifndef __AP_HAL_LINUX_NAMESPACE_H__
#define __AP_HAL_LINUX_NAMESPACE_H__

/* While not strictly required, names inside the Linux namespace are prefixed
 * with Linux for clarity. (Some of our users aren't familiar with all of the
 * C++ namespace rules.)
 */

namespace Linux {
    class LinuxStorage;
    class LinuxSemaphore;
    class LinuxScheduler;
    class LinuxUtil;
}

#endif // __AP_HAL_LINUX_NAMESPACE_H__

