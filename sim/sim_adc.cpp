
#include <AP_Progmem.h>
#include <AP_Common.h>
#include <AP_HAL.h>

#include "sim_adc.h"

extern const AP_HAL::HAL& hal;

// Constructors ////////////////////////////////////////////////////////////////
AP_ADC_Sim::AP_ADC_Sim() { }

// Public Methods //////////////////////////////////////////////////////////////
void AP_ADC_Sim::Init()
{
	// Do nothing
}

// Read one channel value
float AP_ADC_Sim::Ch(uint8_t ch_num)
{
		// Set adc_val and then return
		float adc_val=0;

		return adc_val;
}

// see if Ch6() can return new data
bool AP_ADC_Sim::new_data_available(const uint8_t *channel_numbers)
{
    return true;
}


// Read 6 channel values
// this assumes that the counts for all of the 6 channels are
// equal. This will only be true if we always consistently access a
// sensor by either Ch6() or Ch() and never mix them. If you mix them
// then you will get very strange results
uint32_t AP_ADC_Sim::Ch6(const uint8_t *channel_numbers, float *result)
{
    for (int i = 0; i < 6; i++) {
				// Update result directly here
        result[i] = 0;
    }

    // return number of microseconds since last call
    return 0;
}

/// Get minimum number of samples read from the sensors
uint16_t AP_ADC_Sim::num_samples_available(const uint8_t *channel_numbers)
{
    return 6;
}


