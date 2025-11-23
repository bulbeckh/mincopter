/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include <AP_HAL.h>

#include "avrdx/AnalogIn.h"

using namespace AP_HAL_AVRDx;

extern const AP_HAL::HAL& hal;

/* CHANNEL_READ_REPEAT: how many reads on a channel before using the value.
 * This seems to be determined empirically */
#define CHANNEL_READ_REPEAT 2

AVRDxAnalogIn::AVRDxAnalogIn(void) :
    _vcc(NULL),
	_channels{
		// TODO We define the max number of channels in the header file but we statically declare each on here?? Need to fix
		AVRDxAnalogSource(ANALOG_INPUT_BOARD_VCC),
		AVRDxAnalogSource(ANALOG_INPUT_NONE),
		AVRDxAnalogSource(ANALOG_INPUT_NONE),
		AVRDxAnalogSource(ANALOG_INPUT_NONE),
		AVRDxAnalogSource(ANALOG_INPUT_NONE),
		AVRDxAnalogSource(ANALOG_INPUT_NONE),
		AVRDxAnalogSource(ANALOG_INPUT_NONE),
		AVRDxAnalogSource(ANALOG_INPUT_NONE),
		AVRDxAnalogSource(ANALOG_INPUT_NONE),
	},
	_num_channels{0},
	_active_channel{0},
	_channel_repeat_count{0}
{
}


void AVRDxAnalogIn::init(void*) 
{

    // Register AVRAnalogIn::_timer_event with the scheduler
    hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(AVRDxAnalogIn, &AVRDxAnalogIn::_timer_event));

	// Set VCC to be pointer to the first index in the array
	_vcc = _channels;
	_num_channels = 1;

	// Enable ADC
	/*
    //PRR0 &= ~_BV(PRADC);
    ADCSRA |= _BV(ADEN);
	*/

	// Enable ADC
	ADC0.CTRLA |= ADC_ENABLE_bm;
	
	return;
}

AVRDxAnalogSource* AVRDxAnalogIn::_register_channel(int16_t chnum) {

	// NOTE Update the pin number
	_channels[_num_channels].set_pin(chnum);

    /* Need to lock to increment _num_channels as it is used * by the interrupt to access _channels */
    uint8_t sreg = CPU_SREG;
    cli();
    _num_channels++;
    CPU_SREG = sreg;

	// Return the address of the ADCSource object in the array
	return &(_channels[_num_channels-1]);
}

void AVRDxAnalogIn::_timer_event(void) 
{
	// TODO
	return;

	/*
    if (_channels[_active_channel]._pin == ANALOG_INPUT_NONE) {
        _channels[_active_channel].new_sample(0);
        goto next_channel;
    }

    if (ADCSRA & _BV(ADSC)) {
        // ADC Conversion is still running - this should not happen, as we are called at 1khz
        return;
    }

    if (_num_channels == 0) {
        // No channels are registered - nothing to be done.
        return;
    }

    _channel_repeat_count++;
    if (_channel_repeat_count < CHANNEL_READ_REPEAT ||
        !_channels[_active_channel].reading_settled()) {
        // Start a new conversion, throw away the current conversion
        ADCSRA |= _BV(ADSC);
        return;
    }

    _channel_repeat_count = 0;

    // Read the conversion registers
    {
        uint8_t low = ADCL;
        uint8_t high = ADCH;
        uint16_t sample = low | (((uint16_t)high) << 8);
        // Give the active channel a new sample
        _channels[_active_channel].new_sample( sample );
    }
next_channel:
    // Move to the next channel
    _active_channel = (_active_channel + 1) % _num_channels;
    // Setup the next channel's conversion
    _channels[_active_channel].setup_read();
    // Start conversion
    ADCSRA |= _BV(ADSC);
	*/
}

AP_HAL::AnalogSource* AVRDxAnalogIn::channel(int16_t channel)
{
    if (channel == ANALOG_INPUT_BOARD_VCC) return _vcc;

	if (channel == ANALOG_INPUT_NONE) {
		hal.console->printf("ADC:Requested NONE channel. Returning NULL\r\n");
		return NULL;
	}

	// Check that we are not requesting a channel outside our range
	if (channel > AVR_INPUT_MAX_CHANNELS) {
		hal.console->printf("ADC:Requested invalid channel\r\n");
		return NULL;
    }

	// Check that we have actually set this channel up
	return _register_channel(channel);
}

