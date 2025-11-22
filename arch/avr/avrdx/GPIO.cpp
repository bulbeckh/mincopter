#include <AP_HAL.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include "avrdx/utility/pins_arduino_mega.h"
#include "avrdx/GPIO.h"

using namespace AP_HAL_AVRDx;

AP_HAL::Proc AVRDxGPIO::_interrupt_6 = NULL;

SIGNAL(INT6_vect) {
    if (AVRDxGPIO::_interrupt_6) {
        AVRDxGPIO::_interrupt_6();
    }
}   

// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.

#define analogInPinToBit(P) (P)


// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.
// 
// These perform slightly better as macros compared to inline functions
//
#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )
#define digitalPinToBitMask(P) ( pgm_read_byte( digital_pin_to_bit_mask_PGM + (P) ) )
#define digitalPinToTimer(P) ( pgm_read_byte( digital_pin_to_timer_PGM + (P) ) )

/* Used by DigitalSource to retrieve the registers for reading/writing GPIO pins. The P argument
 * is the index in the register array (defined in the pins_arduino_mega.c file) */
#define portOutputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_output_PGM + (P))) )
#define portInputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_input_PGM + (P))) )
#define portModeRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_mode_PGM + (P))) )

AVRDxGPIO::AVRDxGPIO(void)
	: _sources{
		/* Initialise four digital sources */
		AVRDxDigitalSource(0,0),
		AVRDxDigitalSource(0,0),
		AVRDxDigitalSource(0,0),
		AVRDxDigitalSource(0,0)
	},
	_head(0)
{

}

void AVRDxDigitalSource::set_bit(uint8_t bit)
{
	_bit = bit;
	return;
}

void AVRDxDigitalSource::set_port(uint8_t port)
{
	_port = port;
	return;
}

void AVRDxGPIO::pinMode(uint8_t pin, uint8_t mode) {
	return;
	// TODO
	/*
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *reg;

    if (port == NOT_A_PIN) return;

    // JWS: can I let the optimizer do this?
    reg = portModeRegister(port);

    if (mode == GPIO_INPUT) {
        uint8_t oldSREG = SREG;
                cli();
        *reg &= ~bit;
        SREG = oldSREG;
    } else {
        uint8_t oldSREG = SREG;
                cli();
        *reg |= bit;
        SREG = oldSREG;
    }
	*/
}

int8_t AVRDxGPIO::analogPinToDigitalPin(uint8_t pin)
{
	return analogInputToDigitalPin(pin);
}

uint8_t AVRDxGPIO::read(uint8_t pin) {
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);

    if (port == NOT_A_PIN) return 0;

    if (*portInputRegister(port) & bit) return 1;
    return 0;
}

void AVRDxGPIO::write(uint8_t pin, uint8_t value) {
	return;
	// TODO
	/*
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *out;

    if (port == NOT_A_PIN) return;

    out = portOutputRegister(port);

    uint8_t oldSREG = SREG;
    cli();

    if (value == 0) {
        *out &= ~bit;
    } else {
        *out |= bit;
    }

    SREG = oldSREG;
	*/
}

void AVRDxGPIO::toggle(uint8_t pin) {
	return;
	// TODO
	/*
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *out;

    if (port == NOT_A_PIN) return;

    out = portOutputRegister(port);

    uint8_t oldSREG = SREG;
    cli();

    *out ^= bit;

    SREG = oldSREG;
	*/
}

/* Implement GPIO Interrupt 6, used for MPU6000 data ready on APM2. */
bool AVRDxGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc proc, uint8_t mode) {
	return true;
	// TODO
	/*
    // Mode is to set the ISCn0 and ISCn1 bits. These correspond to the GPIO_INTERRUPT_ defs in AP_HAL.h
    if (!((mode == 0)||(mode == 1)||(mode == 2)||(mode==3))) return false;
    if (interrupt_num == 6) {
	uint8_t oldSREG = SREG;
	cli();	
        _interrupt_6 = proc;
        // Set the ISC60 and ICS61 bits in EICRB according to the value of mode.
        EICRB = (EICRB & ~((1 << ISC60) | (1 << ISC61))) | (mode << ISC60);
        EIMSK |= (1 << INT6);
	SREG = oldSREG;
        return true;
    } else {
        return false;
    }
	*/
}


AP_HAL::DigitalSource* AVRDxGPIO::channel(uint16_t pin) {
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    if (port == NOT_A_PIN) return NULL;

	_sources[_head].set_bit(bit);
	_sources[_head].set_port(port);

	_head++;

    return &(_sources[_head-1]);
}

void AVRDxDigitalSource::mode(uint8_t output) {
	return;
	// TODO
	/*
    const uint8_t bit = _bit;
    const uint8_t port = _port;

    volatile uint8_t* reg;
    reg = portModeRegister(port);

    if (output == GPIO_INPUT) {
        uint8_t oldSREG = SREG;
                cli();
        *reg &= ~bit;
        SREG = oldSREG;
    } else {
        uint8_t oldSREG = SREG;
                cli();
        *reg |= bit;
        SREG = oldSREG;
    }
	*/
}

uint8_t AVRDxDigitalSource::read(void)
{
    const uint8_t bit = _bit;
    const uint8_t port = _port;
    if (*portInputRegister(port) & bit) return 1;
    return 0;
}

void AVRDxDigitalSource::write(uint8_t value) {
	return;
	// TODO
	/*
    const uint8_t bit = _bit;
    const uint8_t port = _port;
    volatile uint8_t* out;
    out = portOutputRegister(port);

    uint8_t oldSREG = SREG;
    cli();

    if (value == 0) {
        *out &= ~bit;
    } else {
        *out |= bit;
    }

    SREG = oldSREG;
	*/
}

void AVRDxDigitalSource::toggle(void) {
	return;
	// TODO
	/*
    const uint8_t bit = _bit;
    const uint8_t port = _port;
    volatile uint8_t* out;
    out = portOutputRegister(port);

    uint8_t oldSREG = SREG;
    cli();

    *out ^= bit;

    SREG = oldSREG;
	*/
}

// return true when USB is connected
bool AVRDxGPIO::usb_connected(void)
{
#if HAL_GPIO_USB_MUX_PIN != -1
    pinMode(HAL_GPIO_USB_MUX_PIN, GPIO_INPUT);
    return !read(HAL_GPIO_USB_MUX_PIN);
#else
    return false;
#endif
}


