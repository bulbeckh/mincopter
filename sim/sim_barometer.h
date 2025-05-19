/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_SIM_H__
#define __AP_BARO_SIM_H__

#include <AP_HAL.h>
#include "AP_Baro.h"

class AP_Baro_Sim : public AP_Baro
{
public:
    AP_Baro_Sim()
    {
    }

    /* AP_Baro public interface: */
    bool            init();
    uint8_t         read();
    float           get_pressure(); // in mbar*100 units
    float           get_temperature(); // in celsius degrees

    //void            _calculate();

#ifdef TARGET_ARCH_LINUX
	/* Public setter methods for Barometer */
	void set_pressure(double gz_pressure);
#endif

private:

	/* @brief TODO What are units of temperature here?? */
    float                           Temp;

	/* @brief pressure reading in pascals (101,325 Pa is approximately sea-level) */
    float                           Press;

};

#endif //  __AP_BARO_MS5611_H__
