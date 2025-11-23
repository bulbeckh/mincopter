/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "avrdx/AnalogIn.h"

using namespace AP_HAL_AVRDx;

extern const AP_HAL::HAL& hal;

AVRDxAnalogSource::AVRDxAnalogSource(uint8_t pin) :
    _sum_count(0),
    _sum(0),
    _last_average(0),
    _pin(pin),
    _settle_time_ms(0)
{
}

float AVRDxAnalogSource::read_average(void) {
    if (_pin == ANALOG_INPUT_BOARD_VCC) {
        uint16_t v = (uint16_t) _read_average();
        return 1126400UL / v;
    } else {
        return _read_average();
    }
}

float AVRDxAnalogSource::read_latest(void) {
	return 0.0f;
	// TODO
	/*
    uint8_t sreg = SREG;
    cli();
    uint16_t latest = _latest;
    SREG = sreg;
    if (_pin == ANALOG_INPUT_BOARD_VCC) {
        return 1126400UL / latest;
    } else {
        return latest;
    }
	*/
}

float AVRDxAnalogSource::voltage_average(void)
{
    float vcc_mV = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC)->read_average();
    float v = read_average();
    // constrain Vcc reading so that a bad Vcc doesn't throw off
    // the reading of other sources too badly
    if (vcc_mV < 4000) {
        vcc_mV = 4000;
    } else if (vcc_mV > 6000) {
        vcc_mV = 6000;
    }
    return v * vcc_mV * 9.765625e-7; // 9.765625e-7 = 1.0/(1024*1000)
}

float AVRDxAnalogSource::voltage_latest(void)
{
    if (_pin == ANALOG_INPUT_BOARD_VCC) {
        return read_latest() * 0.001f;
    }
    float vcc_mV = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC)->read_average();
    float v = read_latest();
    // constrain Vcc reading so that a bad Vcc doesn't throw off
    // the reading of other sources too badly
    if (vcc_mV < 4000) {
        vcc_mV = 4000;
    } else if (vcc_mV > 6000) {
        vcc_mV = 6000;
    }
    return v * vcc_mV * 9.765625e-7; // 9.765625e-7 = 1.0/(1024*1000)
}

float AVRDxAnalogSource::voltage_average_ratiometric(void)
{
    float v = read_average();
    return v * (5.0f / 1023.0f);
}

void AVRDxAnalogSource::set_pin(uint8_t pin) {
	return;
	// TODO
	/*
	// ensure the pin is marked as an INPUT pin
	int8_t dpin = hal.gpio->analogPinToDigitalPin(_pin);
	if (dpin != -1) {
		// enable as input without a pull-up. This gives the
		// best results for our analog sensors
		hal.gpio->pinMode(dpin, GPIO_INPUT);
		hal.gpio->write(dpin, 0);
	}

	uint8_t sreg = SREG;
	cli();
	_sum = 0;
	_sum_count = 0;
	_last_average = 0;
	_latest = 0;
	SREG = sreg;
	*/
}

void AVRDxAnalogSource::set_settle_time(uint16_t settle_time_ms) 
{
    _settle_time_ms = settle_time_ms;
}

float AVRDxAnalogSource::_read_average(void) {
	return 0.0f;
	// TODO
	/*
    uint16_t sum;
    uint8_t sum_count;

    if (_sum_count == 0) {
        // avoid blocking waiting for new samples
        return _last_average;
    }

    // Read and clear in a critical section
    uint8_t sreg = SREG;
    cli();

    sum = _sum;
    sum_count = _sum_count;
    _sum = 0;
    _sum_count = 0;

    SREG = sreg;

    float avg = sum / (float) sum_count;

    _last_average = avg;
    return avg;
	*/
}

void AVRDxAnalogSource::setup_read(void) {
	return;
	// TODO
	/*
    if (_settle_time_ms != 0) {
        _read_start_time_ms = hal.scheduler->millis();
    }
    if (_pin == ANALOG_INPUT_BOARD_VCC) {
        ADCSRB = (ADCSRB & ~(1 << MUX5));
        ADMUX = _BV(REFS0)|_BV(MUX4)|_BV(MUX3)|_BV(MUX2)|_BV(MUX1);
    } else if (_pin == ANALOG_INPUT_NONE) {
		// NOOP
    } else {
        ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((_pin >> 3) & 0x01) << MUX5);
        ADMUX = _BV(REFS0) | (_pin & 0x07);
    }
	*/
}

bool AVRDxAnalogSource::reading_settled(void)
{
    if (_settle_time_ms != 0 && (hal.scheduler->millis() - _read_start_time_ms) < _settle_time_ms) {
        return false;
    }
    return true;
}

/* new_sample is called from an interrupt. It always has access to
 *  _sum and _sum_count. Lock out the interrupts briefly with
 * cli/sei to read these variables from outside an interrupt. */
void AVRDxAnalogSource::new_sample(uint16_t sample) {
    _sum += sample;
    _latest = sample;
    if (_sum_count >= 63) {
        _sum >>= 1;
        _sum_count = 32;
    } else {
        _sum_count++;
    }
}
