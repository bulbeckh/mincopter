// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

#include "radio.h"

#include "mcinstance.h"
#include "mcstate.h"

extern MCInstance mincopter;
extern MCState mcstate;

#include "planner.h"
#include "control.h"

#include "util.h"

void default_dead_zones()
{
    mincopter.rc_1.set_default_dead_zone(30);
    mincopter.rc_2.set_default_dead_zone(30);
#if FRAME_CONFIG == HELI_FRAME
    mincopter.rc_3.set_default_dead_zone(10);
    mincopter.rc_4.set_default_dead_zone(15);
    mincopter.rc_8.set_default_dead_zone(10);
#else
    mincopter.rc_3.set_default_dead_zone(30);
    mincopter.rc_4.set_default_dead_zone(40);
#endif
    mincopter.rc_6.set_default_dead_zone(0);
}

void init_esc()
{
	mincopter.motors.set_update_rate(50);
	mincopter.motors.enable();
	mincopter.motors.armed(true);

	// TODO What was this doing?
	/*
	while(0) {
		read_radio();
		delay(100);
		AP_Notify::flags.esc_calibration = true;
		mincopter.motors.throttle_pass_through();
	}
	*/

}

 // init_rc_out -- initialise motors and check if pilot wants to perform ESC calibration
void init_rc_out()
{
    mincopter.motors.set_update_rate(mincopter.rc_speed);
    //mincopter.motors.set_frame_orientation(mincopter.frame_orientation);
	// NOTE TODO This is hardcoded to 1 which is the X orientation frame
    mincopter.motors.set_frame_orientation(1);
    mincopter.motors.Init();                                              // motor initialisation

#ifdef CONTROLLER_PID
    mincopter.motors.set_min_throttle(controller.throttle_min);
#endif

		/*
    for(uint8_t i = 0; i < 5; i++) {
        delay(20);
        read_radio();
    }
		*/

    // we want the input to be scaled correctly
	mincopter.rc_1.set_angle(ROLL_PITCH_INPUT_MAX);
	mincopter.rc_2.set_angle(ROLL_PITCH_INPUT_MAX);
	// TODO Remove this hardcode
	mincopter.rc_4.set_angle(4500);

	// This is the max and minimum for the output of the throttle accel controller
	// This is currently [130,1000]
	mincopter.rc_3.set_range(130,1000);

	mincopter.rc_1.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
	mincopter.rc_2.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
	mincopter.rc_4.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

	//mincopter.rc_3.set_range_out(0,1000);

    // full throttle means to enter ESC calibration
		// NOTE Skip in autonomous flight - need to perform ESC calibration separately
		/*
    if(mincopter.rc_3.control_in >= (mincopter.throttle_max - 50)) {
        if(mincopter.esc_calibrate == 0) {
            // we will enter esc_calibrate mode on next reboot
            mincopter.esc_calibrate.set_and_save(1);
            // display message on console
            mincopter.cliSerial->printf_P(PSTR("Entering ESC Calibration: please restart APM.\n"));
            // turn on esc calibration notification
            AP_Notify::flags.esc_calibration = true;
            // block until we restart
            while(1) { delay(5); }
        }else{
            mincopter.cliSerial->printf_P(PSTR("ESC Calibration active: passing throttle through to ESCs.\n"));
            // clear esc flag
            mincopter.esc_calibrate.set_and_save(0);
            // pass through user throttle to escs
            init_esc();
        }
    }else{
        // did we abort the calibration?
        if(mincopter.esc_calibrate == 1)
            mincopter.esc_calibrate.set_and_save(0);
    }
		*/

    // enable output to motors
    output_min();
}

// output_min - enable and output lowest possible value to motors
void output_min()
{
    // enable motors
    mincopter.motors.enable();
    mincopter.motors.output_min();
}

#define FS_COUNTER 3        // radio failsafe kicks in after 3 consecutive throttle values below failsafe_throttle_value
void set_throttle_and_failsafe(uint16_t throttle_pwm)
{
    // if failsafe not enabled pass through throttle and exit
    if(planner.failsafe_throttle == FS_THR_DISABLED) {
        mincopter.rc_3.set_pwm(throttle_pwm);
        return;
    }

    //check for low throttle value
    if (throttle_pwm < (uint16_t)planner.failsafe_throttle_value) {

        // if we are already in failsafe or motors not armed pass through throttle and exit
        if (planner.failsafe.radio || !mincopter.motors.armed()) {
            mincopter.rc_3.set_pwm(throttle_pwm);
            return;
        }

        // check for 3 low throttle values
        // Note: we do not pass through the low throttle until 3 low throttle values are recieved
        planner.failsafe.radio_counter++;
        if( planner.failsafe.radio_counter >= FS_COUNTER ) {
            planner.failsafe.radio_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
            set_failsafe_radio(true);
            mincopter.rc_3.set_pwm(throttle_pwm);   // pass through failsafe throttle
        }
    }else{
        // we have a good throttle so reduce failsafe counter
        planner.failsafe.radio_counter--;
        if( planner.failsafe.radio_counter <= 0 ) {
            planner.failsafe.radio_counter = 0;   // check to ensure we don't underflow the counter

            // disengage failsafe after three (nearly) consecutive valid throttle values
            if (planner.failsafe.radio) {
                set_failsafe_radio(false);
            }
        }
        // pass through throttle
        mincopter.rc_3.set_pwm(throttle_pwm);
    }
}

