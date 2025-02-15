
#ifndef __COMPAT_H__
#define __COMPAT_H__

#define OUTPUT GPIO_OUTPUT
#define INPUT GPIO_INPUT

#define HIGH 1
#define LOW 0

#include <AP_HAL.h>

/* Forward declarations to avoid broken auto-prototyper (coughs on '::'?) */
void init_cli(AP_HAL::UARTDriver *port);
void run_cli(void);

#endif // __COMPAT_H__

