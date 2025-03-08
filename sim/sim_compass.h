
#ifndef AP_COMPASS_SIM_H
#define AP_COMPASS_SIM_H

#include <AP_HAL.h>

#include "../AP_Common/AP_Common.h"
#include "../AP_Math/AP_Math.h"

#include "Compass.h"

class AP_Compass_Sim : public Compass
{

public:
    AP_Compass_Sim() : Compass() {
    }
    bool        init(void);
    bool        read(void);
    void        accumulate(void);

private:
    float               calibration[3];
    bool                _initialised;
    uint8_t             _base_config;
		
		/*
    bool                read_register(uint8_t address, uint8_t *value);
    bool                write_register(uint8_t address, uint8_t value);
		*/

    uint32_t            _retry_time; // when unhealthy the millis() value to retry at
    AP_HAL::Semaphore*  _i2c_sem;

		// TODO Remove a bunch of this - not necessary for simulated compass readings
	
		// These contain raw readings
    int16_t			    _mag_x;
    int16_t			    _mag_y;
    int16_t			    _mag_z;

		// These contain accumulated (summed) readings to be averaged later by division by _accum_count
    int16_t             _mag_x_accum;
    int16_t             _mag_y_accum;
    int16_t             _mag_z_accum;
    uint8_t			    _accum_count;

    uint32_t            _last_accum_time;


};
#endif
