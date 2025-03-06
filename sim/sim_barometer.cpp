
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

bool AP_Baro_Sim::init()
{
    healthy = true;
    return true;
}


// Read the sensor. This is a state machine
// We read one time Temperature (state=1) and then 4 times Pressure (states 2-5)
// temperature does not change so quickly...
void AP_Baro_Sim::_update(void)
{
    uint32_t tnow = hal.scheduler->micros();
    // Throttle read rate to 100hz maximum.
    if (tnow - _timer < 10000) {
        return;
    }

    _timer = tnow;

    if (_state == 0) {
        _s_D2 += _serial->read_adc();// On state 0 we read temp
																		 //
        _d2_count++;
        if (_d2_count == 32) {
            // we have summed 32 values. This only happens
            // when we stop reading the barometer for a long time
            // (more than 1.2 seconds)
            _s_D2 >>= 1;
            _d2_count = 16;
        }
        _state++;

    } else {
        _s_D1 += _serial->read_adc();
        _d1_count++;
        if (_d1_count == 128) {
            // we have summed 128 values. This only happens
            // when we stop reading the barometer for a long time
            // (more than 1.2 seconds)
            _s_D1 >>= 1;
            _d1_count = 64;
        }
        _state++;
        // Now a new reading exists
        _updated = true;
        if (_state == 5) {
            _state = 0;
        } else {
        }
    }

}

uint8_t AP_Baro_Sim::read()
{
    bool updated = _updated;

    if (updated) {
        uint32_t sD1, sD2;
        uint8_t d1count, d2count;

        // Suspend timer procs because these variables are written to
        // in "_update".
        sD1 = _s_D1; _s_D1 = 0;
        sD2 = _s_D2; _s_D2 = 0;
        d1count = _d1_count; _d1_count = 0;
        d2count = _d2_count; _d2_count = 0;

        _updated = false;

        if (d1count != 0) {
            D1 = ((float)sD1) / d1count;
        }
        if (d2count != 0) {
            D2 = ((float)sD2) / d2count;
        }
        _pressure_samples = d1count;
        _raw_press = D1;
        _raw_temp = D2;
    }
    _calculate();
    if (updated) {
        _last_update = hal.scheduler->millis();
    }

    return updated ? 1 : 0;
}

float AP_Baro_Sim::get_pressure()
{
    return Press;
}

float AP_Baro_Sim::get_temperature()
{
    // temperature in degrees C units
    return Temp;
}

