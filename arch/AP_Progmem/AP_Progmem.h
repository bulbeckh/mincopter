
#ifndef __AP_PROGMEM_H__
#define __AP_PROGMEM_H__

#include <AP_HAL/AP_HAL_Boards.h>

#if defined(TARGET_ARCH_AVR)
	#include <AP_Progmem/AP_Progmem_AVR.h>
#elif defined(TARGET_ARCH_LINUX)
	#include <AP_Progmem/AP_Progmem_Identity.h>
#endif

#define PROGMEM_STRING(_v, _s)  static const char _v[] PROGMEM = _s

#endif // __AP_PROGMEM_H__

