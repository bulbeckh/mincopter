
#ifndef __AP_HAL_AVR_RC_INPUT_H__
#define __AP_HAL_AVR_RC_INPUT_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

#define AVR_RC_INPUT_NUM_CHANNELS 8
#define AVR_RC_INPUT_MIN_CHANNELS 5     // for ppm sum we allow less than 8 channels to make up a valid packet

class AP_HAL_AVR::APM2RCInput : public AP_HAL::RCInput {
    /* Pass in a AP_HAL_AVR::ISRRegistry* as void*. */
    void     init(void* isrregistry);
    uint8_t  valid_channels();
    uint16_t read(uint8_t ch);
    uint8_t  read(uint16_t* periods, uint8_t len);
    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();
private:
    /* private callback for input capture ISR */
    static void _timer5_capt_cb(void);
    /* private variables to communicate with input capture isr */
    static volatile uint16_t _pulse_capt[AVR_RC_INPUT_NUM_CHANNELS];
    static volatile uint8_t  _valid_channels;

    /* override state */
    uint16_t _override[AVR_RC_INPUT_NUM_CHANNELS]; 
};

#endif // __AP_HAL_AVR_RC_INPUT_H__

