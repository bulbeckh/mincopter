

#pragma once

#include "AP_Baro.h"
#include <AverageFilter.h>

class AP_Baro_BME280 : public AP_Baro
{
public:
    AP_Baro_BME280() {
    };

    /* AP_Baro public interface: */
    bool            init();

    uint8_t         read();

    void 			accumulate(void);

    float           get_pressure();

    float           get_temperature();

private:
	//float Press;
	//float Temp;

};


