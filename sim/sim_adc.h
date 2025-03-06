
#ifndef __AP_ADC_SIM_H_
#define __AP_ADC_SIM_H_

#include <inttypes.h>
#include "AP_ADC.h"
#include <AP_HAL.h>

class AP_ADC_Sim : public AP_ADC
{
public:
    AP_ADC_Sim();
    void                Init();

    // Read 1 sensor value
    float               Ch(unsigned char ch_num);

    // Read 6 sensors at once
    uint32_t            Ch6(const uint8_t *channel_numbers, float *result);

    // check if Ch6 would block
    bool                new_data_available(const uint8_t *channel_numbers);

    // Get minimum number of samples read from the sensors
    uint16_t            num_samples_available(const uint8_t *channel_numbers);

private:
    void read(void);

};

#endif
