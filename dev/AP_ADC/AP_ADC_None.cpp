
#include <AP_Progmem.h>
#include <AP_Common.h>
#include <AP_HAL.h>

#include "AP_ADC_None.h"

void AP_ADC_None::read(void)
{
	/* Implement */
}


// Constructors ////////////////////////////////////////////////////////////////
AP_ADC_None::AP_ADC_None() { }

// Public Methods //////////////////////////////////////////////////////////////
void AP_ADC_None::Init()
{
	/* Implement */
}

float AP_ADC_None::Ch(uint8_t ch_num)
{
    return 0;
}

bool AP_ADC_None::new_data_available(const uint8_t *channel_numbers)
{
	/* Implement */
	return false;
}

uint32_t AP_ADC_None::Ch6(const uint8_t *channel_numbers, float *result)
{
    return 0;
}

uint16_t AP_ADC_None::num_samples_available(const uint8_t *channel_numbers)
{
    return 0;
}



