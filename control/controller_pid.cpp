
#include "controller_pid.h"

#include "util.h"
#include "config.h"

#include <AP_Math.h>

#include "mcinstance.h"
#include "mcstate.h"

// TODO Why is a planner being reference in a controller class??
#include "planner.h"


extern MCInstance mincopter;
extern MCState    mcstate;

#ifdef TARGET_ARCH_LINUX
    #include "simulation_logger.h"
    extern SimulationLogger simlog;
#endif

// Instance
PID_Controller controller;

void PID_Controller::run()
{
	/* 1. Run P controllers to convert desired angle into desired rate
	 *
	 * 2. Run rate controllers
	 *
	 *
	 * 3. Output value to motors(AP_MotorsQuad)
	 */
	
	// (Part of control determination)
	// Run controllers that take body frame rate targets and convert to motor values using PID rate controllers (get_rate_{roll,pitch,yaw})

	/* NOTE We don't need to run yaw controllers yet. ideally they should be kept at 0 */
	update_yaw_mode();
	update_roll_pitch_mode();

	/////////////////////////////////////// THROTTLE control - should run at 50Hz 
		
    // get altitude and climb rate from inertial lib
    read_inertial_altitude();

    // Update the throttle ouput
    // -------------------------
    update_throttle_mode();

		// TODO move this to planner
    // check if we've landed
    //planner.update_land_detector();

    // check auto_armed status
    update_auto_armed();

		///////////////////////////////////////

    update_rate_controller_targets();

		run_rate_controllers();


		// TODO The motors current get their target value from rc_*.servo_out but it should really pull
		// directly from a set of controller parameters
    mincopter.motors.output();

#ifdef TARGET_ARCH_LINUX
	simlog.write_motor_outputs();
#endif

}


void PID_Controller::get_stabilize_roll(int32_t target_angle)
{
    // angle error
    int32_t target_error = wrap_180_cd(target_angle - mcstate.roll_sensor);

    // convert to desired rate
    int32_t target_rate = pi_stabilize_roll.kP() * target_error;

    // constrain the target rate
    target_rate = constrain_int32(target_rate, -angle_rate_max, angle_rate_max);

    // set targets for rate controller
	roll_rate_target_ef = target_rate;

#ifdef TARGET_ARCH_LINUX
	simlog.write_pid_state("stb_roll", target_angle, target_error, target_rate, angle_rate_max, -angle_rate_max);
#endif

}

void PID_Controller::get_stabilize_pitch(int32_t target_angle)
{
    // angle error
    int32_t target_error = wrap_180_cd(target_angle - mcstate.pitch_sensor);

    // convert to desired rate
    int32_t target_rate = pi_stabilize_pitch.kP() * target_error;

	// constrain target
    target_rate = constrain_int32(target_rate, -angle_rate_max, angle_rate_max);

    // set targets for rate controller
	pitch_rate_target_ef = target_rate;

#ifdef TARGET_ARCH_LINUX
	simlog.write_pid_state("stb_pitch", target_angle, target_error, target_rate, angle_rate_max, -angle_rate_max);
#endif

}

void PID_Controller::get_stabilize_yaw(int32_t target_angle)
{
    int32_t target_rate;
    int32_t angle_error;

    // angle error
    angle_error = wrap_180_cd(target_angle - mcstate.yaw_sensor);

    // limit the error we're feeding to the PID
    angle_error = constrain_int32(angle_error, -4500, 4500);

    // convert angle error to desired Rate:
    target_rate = pi_stabilize_yaw.kP() * angle_error;

    // set targets for rate controller
	yaw_rate_target_ef = target_rate;

#ifdef TARGET_ARCH_LINUX
	simlog.write_pid_state("stb_yaw", target_angle, angle_error, target_rate, 0, 0);
#endif
}

void PID_Controller::update_rate_controller_targets()
{
		// convert earth frame rates to body frame rates
    roll_rate_target_bf     = roll_rate_target_ef - mcstate.sin_pitch * yaw_rate_target_ef;
    pitch_rate_target_bf    = mcstate.cos_roll_x  * pitch_rate_target_ef + mcstate.sin_roll * mcstate.cos_pitch_x * yaw_rate_target_ef;
    yaw_rate_target_bf      = mcstate.cos_pitch_x * mcstate.cos_roll_x * yaw_rate_target_ef - mcstate.sin_roll * pitch_rate_target_ef;
}

void PID_Controller::run_rate_controllers()
{
    // call rate controllers
		// TODO This doesn't need to be passed in as a parameter
    mincopter.rc_1.servo_out = get_rate_roll(roll_rate_target_bf);
    mincopter.rc_2.servo_out = get_rate_pitch(pitch_rate_target_bf);
    mincopter.rc_4.servo_out = get_rate_yaw(yaw_rate_target_bf);

    // run throttle controller if accel based throttle controller is enabled and active (active means it has been given a target)
    if( throttle_accel_controller_active ) {
        set_throttle_out(get_throttle_accel(throttle_accel_target_ef), true);
    }
}

int16_t PID_Controller::get_rate_roll(int32_t target_rate)
{
    int32_t p,i,d;                  // used to capture pid values for logging
    int32_t current_rate;           // this iteration's rate
    int32_t rate_error;             // simply target_rate - current_rate
    int32_t output;                 // output from pid controller

    // get current rate
		// TODO Change this to get reading directly from sensors unless perfomance is signficantly degraded
    //current_rate    = (mcstate.omega.x * DEGX100);
	// TODO Change this and the other to compute once during start of ::run and then use there UNLESS the compiler optimises out this call to get_gyro
    current_rate    = (mincopter.ins.get_gyro().x * DEGX100);

    // call pid controller
    rate_error  = target_rate - current_rate;
    p           = pid_rate_roll.get_p(rate_error);

    // get i term
    i = pid_rate_roll.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!mincopter.motors.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = pid_rate_roll.get_i(rate_error, mincopter.G_Dt);
    }

    d = pid_rate_roll.get_d(rate_error, mincopter.G_Dt);
    output = p + i + d;

    // constrain output
    output = constrain_int32(output, -5000, 5000);

#ifdef TARGET_ARCH_LINUX
	simlog.write_pid_state("rate_roll", target_rate, rate_error, output, 5000, -5000);
#endif

    // output control
    return output;
}

int16_t PID_Controller::get_rate_pitch(int32_t target_rate)
{
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t current_rate;                                                       // this iteration's rate
    int32_t rate_error;                                                                 // simply target_rate - current_rate
    int32_t output;                                                                     // output from pid controller

    // get current rate
    //current_rate    = (mcstate.omega.y * DEGX100);
    current_rate    = (mincopter.ins.get_gyro().y * DEGX100);

    // call pid controller
    rate_error      = target_rate - current_rate;
    p               = pid_rate_pitch.get_p(rate_error);

    // get i term
    i = pid_rate_pitch.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!mincopter.motors.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = pid_rate_pitch.get_i(rate_error, mincopter.G_Dt);
    }

    d = pid_rate_pitch.get_d(rate_error, mincopter.G_Dt);
    output = p + i + d;

    // constrain output
    output = constrain_int32(output, -5000, 5000);

#ifdef TARGET_ARCH_LINUX
	simlog.write_pid_state("rate_pitch", target_rate, rate_error, output, 5000, -5000);
#endif

    // output control
    return output;
}

int16_t PID_Controller::get_rate_yaw(int32_t target_rate)
{
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t rate_error;
    int32_t output;

    // rate control
    //rate_error              = target_rate - (mcstate.omega.z * DEGX100);
    rate_error              = target_rate - (mincopter.ins.get_gyro().z * DEGX100);

    // separately calculate p, i, d values for logging
		p = pid_rate_yaw.get_p(rate_error);

    // get i term
    i = pid_rate_yaw.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!mincopter.motors.limit.yaw || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
				i = pid_rate_yaw.get_i(rate_error, mincopter.G_Dt);
    }

    // get d value
    d = pid_rate_yaw.get_d(rate_error, mincopter.G_Dt);

    output  = p+i+d;
    output = constrain_int32(output, -4500, 4500);

#ifdef TARGET_ARCH_LINUX
	simlog.write_pid_state("rate_yaw", target_rate, rate_error, output, 4500, -4500);
#endif

    // constrain output
    return output;
}


/*************************************************************
 *  throttle control
 ****************************************************************/

// update_throttle_cruise - update throttle cruise if necessary
void PID_Controller::update_throttle_cruise(int16_t throttle)
{
    // ensure throttle_avg has been initialised
    if( throttle_avg == 0 ) {
        throttle_avg = throttle_cruise;
    }
    // calc average throttle if we are in a level hover
    if (throttle > throttle_min && abs(climb_rate) < 60 && labs(mcstate.roll_sensor) < 500 && labs(mcstate.pitch_sensor) < 500) {
        throttle_avg = throttle_avg * 0.99f + (float)throttle * 0.01f;
        throttle_cruise = throttle_avg;
    }

	return;
}

// get_angle_boost - returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
int16_t PID_Controller::get_angle_boost(int16_t throttle)
{
    float temp = mcstate.cos_pitch_x * mcstate.cos_roll_x;
    int16_t throttle_out;

    temp = constrain_float(temp, 0.5f, 1.0f);

    // reduce throttle if we go inverted
    temp = constrain_float(9000-ap_max(labs(mcstate.roll_sensor),labs(mcstate.pitch_sensor)), 0, 3000) / (3000 * temp);

    // apply scale and constrain throttle
    throttle_out = constrain_float((float)(throttle-throttle_min) * temp + throttle_min, throttle_min, 1000);

    // to allow logging of angle boost
    angle_boost = throttle_out - throttle;

    return throttle_out;
}

 // set_throttle_out - to be called by upper throttle controllers when they wish to provide throttle output directly to motors
 // provide 0 to cut motors
void PID_Controller::set_throttle_out( int16_t throttle_out, bool apply_angle_boost )
{
    if( apply_angle_boost ) {
        mincopter.rc_3.servo_out = get_angle_boost(throttle_out);
    } else {
        mincopter.rc_3.servo_out = throttle_out;
        // clear angle_boost for logging purposes
        angle_boost = 0;
    }

    // update compass with throttle value
    mincopter.compass.set_throttle((float)mincopter.rc_3.servo_out/1000.0f);
}

// get_throttle_accel - accelerometer based throttle controller
// returns an actual throttle output (0 ~ 1000) to be sent to the motors
int16_t PID_Controller::get_throttle_accel(int16_t z_target_accel)
{
    static float z_accel_error = 0;     // The acceleration error in cm.
    static uint32_t last_call_ms = 0;   // the last time this controller was called
    int32_t p,i,d;                      // used to capture pid values for logging
    int16_t output;
    float z_accel_meas;
    uint32_t now = millis();

    // Calculate Earth Frame Z acceleration
	// TODO Decide how to obtain earth-frame acceleration from MCState
    //z_accel_meas = -(mcstate.get_accel_ef().z + GRAVITY_MSS) * 100;

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 100 ) {
        // Reset Filter
        z_accel_error = 0;
    } else {
        // calculate accel error and Filter with fc = 2 Hz
        z_accel_error = z_accel_error + 0.11164f * (constrain_float(z_target_accel - z_accel_meas, -32000, 32000) - z_accel_error);
    }
    last_call_ms = now;

    // separately calculate p, i, d values for logging
    p = pid_throttle_accel.get_p(z_accel_error);

    // get i term
    i = pid_throttle_accel.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if ((!mincopter.motors.limit.throttle_lower && !mincopter.motors.limit.throttle_upper) || (i>0&&z_accel_error<0) || (i<0&&z_accel_error>0)) {
        i = pid_throttle_accel.get_i(z_accel_error, .01f);
    }

    d = pid_throttle_accel.get_d(z_accel_error, .01f);

    output =  constrain_float(p+i+d+throttle_cruise, throttle_min, throttle_max);

#ifdef TARGET_ARCH_LINUX
	simlog.write_pid_state("throttle_accel", (int32_t)z_target_accel, (int32_t)z_accel_error, (int32_t)output, throttle_max, throttle_min);
#endif

    return output;
}


// get_throttle_rate - calculates desired accel required to achieve desired z_target_speed
// sets accel based throttle controller target
void PID_Controller::get_throttle_rate(float z_target_speed)
{
    static uint32_t last_call_ms = 0;
    static float z_rate_error = 0;   // The velocity error in cm.
    static float z_target_speed_filt = 0;   // The filtered requested speed
    float z_target_speed_delta;   // The change in requested speed
    int32_t p;          // used to capture pid values for logging
    int32_t output;     // the target acceleration if the accel based throttle is enabled, otherwise the output to be sent to the motors
    uint32_t now = millis();

    // reset target altitude if this controller has just been engaged
    if( now - last_call_ms > 100 ) {
        // Reset Filter
        z_rate_error = 0;
        z_target_speed_filt = z_target_speed;
        output = 0;
    } else {
        // calculate rate error and filter with cut off frequency of 2 Hz
        z_rate_error    = z_rate_error + 0.20085f * ((z_target_speed - climb_rate) - z_rate_error);
        // feed forward acceleration based on change in the filtered desired speed.
        z_target_speed_delta = 0.20085f * (z_target_speed - z_target_speed_filt);
        z_target_speed_filt    = z_target_speed_filt + z_target_speed_delta;
        output = z_target_speed_delta * 50.0f;   // To-Do: replace 50 with dt
    }
    last_call_ms = now;

    // calculate p
    p = pid_throttle_rate.kP() * z_rate_error;

    // consolidate and constrain target acceleration
    output += p;
    output = constrain_int32(output, -32000, 32000);

    // set target for accel based throttle controller
    throttle_accel_target_ef = output;
    throttle_accel_controller_active = true;

    // update throttle cruise
    // TO-DO: this may not be correct because g.rc_3.servo_out has not been updated for this iteration
    if( z_target_speed == 0 ) {
        update_throttle_cruise(mincopter.rc_3.servo_out);
    }

#ifdef TARGET_ARCH_LINUX
	simlog.write_pid_state("throttle_rate", (int32_t)z_target_speed, (int32_t)z_rate_error, output, 32000, -32000);
#endif

}

// get_throttle_althold - hold at the desired altitude in cm
// updates accel based throttle controller targets
// Note: max_climb_rate is an optional parameter to allow reuse of this function by landing controller
void PID_Controller::get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate)
{
    int32_t alt_error;
    float desired_rate;
    int32_t linear_distance;      // half the distace we swap between linear and sqrt and the distace we offset sqrt.

    // calculate altitude error
    alt_error    = target_alt - mcstate.current_loc.alt;

    // check kP to avoid division by zero
    if( pi_alt_hold.kP() != 0 ) {
        linear_distance = ALT_HOLD_ACCEL_MAX/(2*pi_alt_hold.kP()*pi_alt_hold.kP());
        if( alt_error > 2*linear_distance ) {
            desired_rate = safe_sqrt(2*ALT_HOLD_ACCEL_MAX*(alt_error-linear_distance));
        }else if( alt_error < -2*linear_distance ) {
            desired_rate = -safe_sqrt(2*ALT_HOLD_ACCEL_MAX*(-alt_error-linear_distance));
        }else{
            desired_rate = pi_alt_hold.get_p(alt_error);
        }
    }else{
        desired_rate = 0;
    }

    desired_rate = constrain_float(desired_rate, min_climb_rate, max_climb_rate);

    // call rate based throttle controller which will update accel based throttle controller targets
    get_throttle_rate(desired_rate);

#ifdef TARGET_ARCH_LINUX
	simlog.write_pid_state("throttle_althold", target_alt, alt_error, (int32_t)desired_rate, (int32_t)max_climb_rate, (int32_t)min_climb_rate);
#endif

}

// reset all I integrators
void PID_Controller::reset_I_all(void)
{
    reset_rate_I();
    reset_throttle_I();
}

void PID_Controller::reset_rate_I()
{
    pid_rate_roll.reset_I();
    pid_rate_pitch.reset_I();
    pid_rate_yaw.reset_I();
}

void PID_Controller::reset_throttle_I(void)
{
    // For Altitude Hold
    pi_alt_hold.reset_I();
    pid_throttle_accel.reset_I();
}



// update_throttle_mode - run high level throttle controllers
// 50 hz update rate
void PID_Controller::update_throttle_mode(void)
{
    // do not run throttle controllers if motors disarmed
    if( !mincopter.motors.armed() ) {
        set_throttle_out(0, false);
    		throttle_accel_controller_active = false;
        return;
    }

		/* TODO Move the slow start call to the planner to initiate slow start when mode changes from land to takeoff
    if (planner.ap.land_complete) {
      	// tell motors to do a slow start.
        mincopter.motors.slow_start(true);
    }
		*/
    get_throttle_althold(controller_desired_alt, min_climb_rate-250, max_climb_rate+250);   // 250 is added to give head room to alt hold controller

}

// update_yaw_mode - run high level yaw controllers
// 100hz update rate
void PID_Controller::update_yaw_mode(void)
{
		// TODO Remove for autonomous flight
    //int16_t pilot_yaw = mincopter.rc_4.control_in;

		// if we are landed reset yaw target to current heading'
		/* TODO Move the LAND checks here to planner. Planner should not even be running output to motors if we are landed.
    if (planner.ap.land_complete) {
			control_yaw = mcstate.ahrs.yaw_sensor;
		}
		*/
    
		// TODO Remove the control_yaw as a parameter as the variable already exists in this class
		get_stabilize_yaw(control_yaw);

    // REMOVED YAW_LOOK_AT_LOCATION:

		// REMOVED get_circle_yaw

    // REMOVED YAW_LOOK_AT_HOME:

    // REMOVED YAW_LOOK_AT_HEADING:

	  // REMOVED YAW_LOOK_AHEAD:

    // REMOVED YAW_RESETTOARMEDYAW:

}

// update_roll_pitch_mode - run high level roll and pitch controllers
// 100hz update rate
void PID_Controller::update_roll_pitch_mode(void)
{

    get_stabilize_roll(control_roll);
    get_stabilize_pitch(control_pitch);

		/* TODO Need to determine when to reset the I terms in each PID controller. Shouldn't be dependent on planner though
    if(mincopter.rc_3.control_in == 0 && planner.control_mode <= ACRO) {
        reset_rate_I();
    }
		*/

}




