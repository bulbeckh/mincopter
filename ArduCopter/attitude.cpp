
#include "attitude.h"

#include "mcinstance.h"
#include "mcstate.h"

#include "util.h"
#include "navigation.h"
#include "motors.h"

extern MCInstance mincopter;
extern MCState mcstate;

void reset_roll_pitch_in_filters(int16_t roll_in, int16_t pitch_in)
{
    mincopter.roll_in_filtered = constrain_int16(roll_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);
    mincopter.pitch_in_filtered = constrain_int16(pitch_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);
}

void get_pilot_desired_lean_angles(int16_t roll_in, int16_t pitch_in, int16_t &roll_out, int16_t &pitch_out)
{
    static float _scaler = 1.0;
    static int16_t _angle_max = 0;

    // range check the input
    roll_in = constrain_int16(roll_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);
    pitch_in = constrain_int16(pitch_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);

    // filter input for feel
    if (mincopter.g.rc_feel_rp >= RC_FEEL_RP_VERY_CRISP) {
        // no filtering required
        mincopter.roll_in_filtered = roll_in;
        mincopter.pitch_in_filtered = pitch_in;
    }else{
        float filter_gain;
        if (mincopter.g.rc_feel_rp >= RC_FEEL_RP_CRISP) {
            filter_gain = 0.5;
        } else if(mincopter.g.rc_feel_rp >= RC_FEEL_RP_MEDIUM) {
            filter_gain = 0.3;
        } else if(mincopter.g.rc_feel_rp >= RC_FEEL_RP_SOFT) {
            filter_gain = 0.05;
        } else {
            // must be RC_FEEL_RP_VERY_SOFT
            filter_gain = 0.02;
        }
        mincopter.roll_in_filtered = mincopter.roll_in_filtered * (1.0 - filter_gain) + (float)roll_in * filter_gain;
        mincopter.pitch_in_filtered = mincopter.pitch_in_filtered * (1.0 - filter_gain) + (float)pitch_in * filter_gain;
    }

    // return filtered roll if no scaling required
    if (mincopter.g.angle_max == ROLL_PITCH_INPUT_MAX) {
        roll_out = (int16_t)mincopter.roll_in_filtered;
        pitch_out = (int16_t)mincopter.pitch_in_filtered;
        return;
    }

    // check if angle_max has been updated and redo scaler
    if (mincopter.g.angle_max != _angle_max) {
        _angle_max = mincopter.g.angle_max;
        _scaler = (float)mincopter.g.angle_max/(float)ROLL_PITCH_INPUT_MAX;
    }

    // convert pilot input to lean angle
    roll_out = (int16_t)(mincopter.roll_in_filtered * _scaler);
    pitch_out = (int16_t)(mincopter.pitch_in_filtered * _scaler);
}

void
get_stabilize_roll(int32_t target_angle)
{
    // angle error
    target_angle = wrap_180_cd(target_angle - mcstate.ahrs.roll_sensor);

    // convert to desired rate
    int32_t target_rate = mincopter.g.pi_stabilize_roll.kP() * target_angle;

    // constrain the target rate
    if (!mincopter.ap.disable_stab_rate_limit) {
        target_rate = constrain_int32(target_rate, -mincopter.g.angle_rate_max, mincopter.g.angle_rate_max);
    }

    // set targets for rate controller
    set_roll_rate_target(target_rate, EARTH_FRAME);
}

void
get_stabilize_pitch(int32_t target_angle)
{
    // angle error
    target_angle            = wrap_180_cd(target_angle - mcstate.ahrs.pitch_sensor);

    // convert to desired rate
    int32_t target_rate = mincopter.g.pi_stabilize_pitch.kP() * target_angle;

    // constrain the target rate
    if (!mincopter.ap.disable_stab_rate_limit) {
        target_rate = constrain_int32(target_rate, -mincopter.g.angle_rate_max, mincopter.g.angle_rate_max);
    }

    // set targets for rate controller
    set_pitch_rate_target(target_rate, EARTH_FRAME);
}

void
get_stabilize_yaw(int32_t target_angle)
{
    int32_t target_rate;
    int32_t angle_error;

    // angle error
    angle_error = wrap_180_cd(target_angle - mcstate.ahrs.yaw_sensor);

    // limit the error we're feeding to the PID
    angle_error = constrain_int32(angle_error, -4500, 4500);

    // convert angle error to desired Rate:
    target_rate = mincopter.g.pi_stabilize_yaw.kP() * angle_error;

    // set targets for rate controller
    set_yaw_rate_target(target_rate, EARTH_FRAME);
}

// TODO can optimise these since all will be EARTH FRAME now I think
// TODO Consolidate these into one function

// set_roll_rate_target - to be called by upper controllers to set roll rate targets in the earth frame
void set_roll_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    mincopter.rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        mincopter.roll_rate_target_bf = desired_rate;
    } else {
        mincopter.roll_rate_target_ef = desired_rate;
    }
}

// set_pitch_rate_target - to be called by upper controllers to set pitch rate targets in the earth frame
void set_pitch_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    mincopter.rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        mincopter.pitch_rate_target_bf = desired_rate;
    }else{
        mincopter.pitch_rate_target_ef = desired_rate;
    }
}

// set_yaw_rate_target - to be called by upper controllers to set yaw rate targets in the earth frame
void set_yaw_rate_target( int32_t desired_rate, uint8_t earth_or_body_frame ) {
    mincopter.rate_targets_frame = earth_or_body_frame;
    if( earth_or_body_frame == BODY_FRAME ) {
        mincopter.yaw_rate_target_bf = desired_rate;
    }else{
        mincopter.yaw_rate_target_ef = desired_rate;
    }
}

void
update_rate_controller_targets()
{
	// NOTE rate_targets_frame should always be EARTH_FRAME now
    if( mincopter.rate_targets_frame == EARTH_FRAME ) {
        // convert earth frame rates to body frame rates
        mincopter.roll_rate_target_bf     = mincopter.roll_rate_target_ef - mcstate.sin_pitch * mincopter.yaw_rate_target_ef;
        mincopter.pitch_rate_target_bf    = mcstate.cos_roll_x  * mincopter.pitch_rate_target_ef + mcstate.sin_roll * mcstate.cos_pitch_x * mincopter.yaw_rate_target_ef;
        mincopter.yaw_rate_target_bf      = mcstate.cos_pitch_x * mcstate.cos_roll_x * mincopter.yaw_rate_target_ef - mcstate.sin_roll * mincopter.pitch_rate_target_ef;
    } 
}

// run roll, pitch and yaw rate controllers and send output to motors
// targets for these controllers comes from stabilize controllers
void
run_rate_controllers()
{
    // call rate controllers
    mincopter.g.rc_1.servo_out = get_rate_roll(mincopter.roll_rate_target_bf);
    mincopter.g.rc_2.servo_out = get_rate_pitch(mincopter.pitch_rate_target_bf);
    mincopter.g.rc_4.servo_out = get_rate_yaw(mincopter.yaw_rate_target_bf);

    // run throttle controller if accel based throttle controller is enabled and active (active means it has been given a target)
    if( mincopter.throttle_accel_controller_active ) {
        set_throttle_out(get_throttle_accel(mincopter.throttle_accel_target_ef), true);
    }
}

int16_t
get_rate_roll(int32_t target_rate)
{
    int32_t p,i,d;                  // used to capture pid values for logging
    int32_t current_rate;           // this iteration's rate
    int32_t rate_error;             // simply target_rate - current_rate
    int32_t output;                 // output from pid controller

    // get current rate
    current_rate    = (mcstate.omega.x * DEGX100);

    // call pid controller
    rate_error  = target_rate - current_rate;
    p           = mincopter.g.pid_rate_roll.get_p(rate_error);

    // get i term
    i = mincopter.g.pid_rate_roll.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!mincopter.motors.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = mincopter.g.pid_rate_roll.get_i(rate_error, mincopter.G_Dt);
    }

    d = mincopter.g.pid_rate_roll.get_d(rate_error, mincopter.G_Dt);
    output = p + i + d;

    // constrain output
    output = constrain_int32(output, -5000, 5000);

    // output control
    return output;
}

int16_t
get_rate_pitch(int32_t target_rate)
{
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t current_rate;                                                       // this iteration's rate
    int32_t rate_error;                                                                 // simply target_rate - current_rate
    int32_t output;                                                                     // output from pid controller

    // get current rate
    current_rate    = (mcstate.omega.y * DEGX100);

    // call pid controller
    rate_error      = target_rate - current_rate;
    p               = mincopter.g.pid_rate_pitch.get_p(rate_error);

    // get i term
    i = mincopter.g.pid_rate_pitch.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!mincopter.motors.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = mincopter.g.pid_rate_pitch.get_i(rate_error, mincopter.G_Dt);
    }

    d = mincopter.g.pid_rate_pitch.get_d(rate_error, mincopter.G_Dt);
    output = p + i + d;

    // constrain output
    output = constrain_int32(output, -5000, 5000);

    // output control
    return output;
}

int16_t
get_rate_yaw(int32_t target_rate)
{
    int32_t p,i,d;                                                                      // used to capture pid values for logging
    int32_t rate_error;
    int32_t output;

    // rate control
    rate_error              = target_rate - (mcstate.omega.z * DEGX100);

    // separately calculate p, i, d values for logging
    p = mincopter.g.pid_rate_yaw.get_p(rate_error);

    // get i term
    i = mincopter.g.pid_rate_yaw.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!mincopter.motors.limit.yaw || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = mincopter.g.pid_rate_yaw.get_i(rate_error, mincopter.G_Dt);
    }

    // get d value
    d = mincopter.g.pid_rate_yaw.get_d(rate_error, mincopter.G_Dt);

    output  = p+i+d;
    output = constrain_int32(output, -4500, 4500);

    // constrain output
    return output;
}


/*************************************************************
 * yaw controllers
 *************************************************************/

// get_look_at_yaw - updates bearing to location held in look_at_yaw_WP and calls stabilize yaw controller
// should be called at 100hz
void get_look_at_yaw()
{
    static uint8_t look_at_yaw_counter = 0;     // used to reduce update rate to 10hz

    look_at_yaw_counter++;
    if( look_at_yaw_counter >= 10 ) {
        look_at_yaw_counter = 0;
        mincopter.yaw_look_at_WP_bearing = pv_get_bearing_cd(mcstate.inertial_nav.get_position(), mincopter.yaw_look_at_WP);
    }

    // slew yaw and call stabilize controller
    mcstate.control_yaw = get_yaw_slew(mcstate.control_yaw, mincopter.yaw_look_at_WP_bearing, AUTO_YAW_SLEW_RATE);
    get_stabilize_yaw(mcstate.control_yaw);
}

void get_look_ahead_yaw(int16_t pilot_yaw)
{
    // Commanded Yaw to automatically look ahead.
    if (mincopter.g_gps->fix && mincopter.g_gps->ground_speed_cm > YAW_LOOK_AHEAD_MIN_SPEED) {
        mcstate.control_yaw = get_yaw_slew(mcstate.control_yaw, mincopter.g_gps->ground_course_cd, AUTO_YAW_SLEW_RATE);
        get_stabilize_yaw(wrap_360_cd(mcstate.control_yaw + pilot_yaw));   // Allow pilot to "skid" around corners up to 45 degrees
    } /* REMOVING ACRO
		else{
        control_yaw += pilot_yaw * g.acro_yaw_p * G_Dt;
        control_yaw = wrap_360_cd(control_yaw);
        get_stabilize_yaw(control_yaw);
    }
		*/
}

/*************************************************************
 *  throttle control
 ****************************************************************/

// update_throttle_cruise - update throttle cruise if necessary
void update_throttle_cruise(int16_t throttle)
{
    // ensure throttle_avg has been initialised
    if( mincopter.throttle_avg == 0 ) {
        mincopter.throttle_avg = mincopter.g.throttle_cruise;
    }
    // calc average throttle if we are in a level hover
    if (throttle > mincopter.g.throttle_min && abs(mincopter.climb_rate) < 60 && labs(mcstate.ahrs.roll_sensor) < 500 && labs(mcstate.ahrs.pitch_sensor) < 500) {
        mincopter.throttle_avg = mincopter.throttle_avg * 0.99f + (float)throttle * 0.01f;
        mincopter.g.throttle_cruise = mincopter.throttle_avg;
    }
}

// get_angle_boost - returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
int16_t get_angle_boost(int16_t throttle)
{
    float temp = mcstate.cos_pitch_x * mcstate.cos_roll_x;
    int16_t throttle_out;

    temp = constrain_float(temp, 0.5f, 1.0f);

    // reduce throttle if we go inverted
    temp = constrain_float(9000-max(labs(mcstate.ahrs.roll_sensor),labs(mcstate.ahrs.pitch_sensor)), 0, 3000) / (3000 * temp);

    // apply scale and constrain throttle
    throttle_out = constrain_float((float)(throttle-mincopter.g.throttle_min) * temp + mincopter.g.throttle_min, mincopter.g.throttle_min, 1000);

    // to allow logging of angle boost
    mincopter.angle_boost = throttle_out - throttle;

    return throttle_out;
}

 // set_throttle_out - to be called by upper throttle controllers when they wish to provide throttle output directly to motors
 // provide 0 to cut motors
void set_throttle_out( int16_t throttle_out, bool apply_angle_boost )
{
    if( apply_angle_boost ) {
        mincopter.g.rc_3.servo_out = get_angle_boost(throttle_out);
    } else {
        mincopter.g.rc_3.servo_out = throttle_out;
        // clear angle_boost for logging purposes
        mincopter.angle_boost = 0;
    }

    // update compass with throttle value
    mincopter.compass.set_throttle((float)mincopter.g.rc_3.servo_out/1000.0f);
}

// set_throttle_accel_target - to be called by upper throttle controllers to set desired vertical acceleration in earth frame
void set_throttle_accel_target( int16_t desired_acceleration )
{
    mincopter.throttle_accel_target_ef = desired_acceleration;
    mincopter.throttle_accel_controller_active = true;
}

// disable_throttle_accel - disables the accel based throttle controller
// it will be re-enasbled on the next set_throttle_accel_target
// required when we wish to set motors to zero when pilot inputs zero throttle
void throttle_accel_deactivate()
{
    mincopter.throttle_accel_controller_active = false;
}

// get_throttle_accel - accelerometer based throttle controller
// returns an actual throttle output (0 ~ 1000) to be sent to the motors
int16_t
get_throttle_accel(int16_t z_target_accel)
{
    static float z_accel_error = 0;     // The acceleration error in cm.
    static uint32_t last_call_ms = 0;   // the last time this controller was called
    int32_t p,i,d;                      // used to capture pid values for logging
    int16_t output;
    float z_accel_meas;
    uint32_t now = millis();

    // Calculate Earth Frame Z acceleration
    z_accel_meas = -(mcstate.ahrs.get_accel_ef().z + GRAVITY_MSS) * 100;

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
    p = mincopter.g.pid_throttle_accel.get_p(z_accel_error);

    // get i term
    i = mincopter.g.pid_throttle_accel.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if ((!mincopter.motors.limit.throttle_lower && !mincopter.motors.limit.throttle_upper) || (i>0&&z_accel_error<0) || (i<0&&z_accel_error>0)) {
        i = mincopter.g.pid_throttle_accel.get_i(z_accel_error, .01f);
    }

    d = mincopter.g.pid_throttle_accel.get_d(z_accel_error, .01f);

    output =  constrain_float(p+i+d+mincopter.g.throttle_cruise, mincopter.g.throttle_min, mincopter.g.throttle_max);

    return output;
}

// get_pilot_desired_throttle - transform pilot's throttle input to make cruise throttle mid stick
// used only for manual throttle modes
// returns throttle output 0 to 1000
#define THROTTLE_IN_MIDDLE 500          // the throttle mid point
int16_t get_pilot_desired_throttle(int16_t throttle_control)
{
    int16_t throttle_out;

    // exit immediately in the simple cases
    if( throttle_control == 0 || mincopter.g.throttle_mid == 500) {
        return throttle_control;
    }

    // ensure reasonable throttle values
    throttle_control = constrain_int16(throttle_control,0,1000);
    mincopter.g.throttle_mid = constrain_int16(mincopter.g.throttle_mid,300,700);

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_MIDDLE) {
        // below the deadband
        throttle_out = mincopter.g.throttle_min + ((float)(throttle_control-mincopter.g.throttle_min))*((float)(mincopter.g.throttle_mid - mincopter.g.throttle_min))/((float)(500-mincopter.g.throttle_min));
    }else if(throttle_control > THROTTLE_IN_MIDDLE) {
        // above the deadband
        throttle_out = mincopter.g.throttle_mid + ((float)(throttle_control-500))*(float)(1000-mincopter.g.throttle_mid)/500.0f;
    }else{
        // must be in the deadband
        throttle_out = mincopter.g.throttle_mid;
    }

    return throttle_out;
}

// get_initial_alt_hold - get new target altitude based on current altitude and climb rate
int32_t
get_initial_alt_hold( int32_t alt_cm, int16_t climb_rate_cms)
{
    int32_t target_alt;
    int32_t linear_distance;      // half the distace we swap between linear and sqrt and the distace we offset sqrt.
    int32_t linear_velocity;      // the velocity we swap between linear and sqrt.

    linear_velocity = ALT_HOLD_ACCEL_MAX/mincopter.g.pi_alt_hold.kP();

    if (abs(climb_rate_cms) < linear_velocity) {
        target_alt = alt_cm + climb_rate_cms/mincopter.g.pi_alt_hold.kP();
    } else {
        linear_distance = ALT_HOLD_ACCEL_MAX/(2*mincopter.g.pi_alt_hold.kP()*mincopter.g.pi_alt_hold.kP());
        if (climb_rate_cms > 0){
            target_alt = alt_cm + linear_distance + (int32_t)climb_rate_cms*(int32_t)climb_rate_cms/(2*ALT_HOLD_ACCEL_MAX);
        } else {
            target_alt = alt_cm - ( linear_distance + (int32_t)climb_rate_cms*(int32_t)climb_rate_cms/(2*ALT_HOLD_ACCEL_MAX) );
        }
    }
    return constrain_int32(target_alt, alt_cm - ALT_HOLD_INIT_MAX_OVERSHOOT, alt_cm + ALT_HOLD_INIT_MAX_OVERSHOOT);
}

// get_throttle_rate - calculates desired accel required to achieve desired z_target_speed
// sets accel based throttle controller target
void
get_throttle_rate(float z_target_speed)
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
        z_rate_error    = z_rate_error + 0.20085f * ((z_target_speed - mincopter.climb_rate) - z_rate_error);
        // feed forward acceleration based on change in the filtered desired speed.
        z_target_speed_delta = 0.20085f * (z_target_speed - z_target_speed_filt);
        z_target_speed_filt    = z_target_speed_filt + z_target_speed_delta;
        output = z_target_speed_delta * 50.0f;   // To-Do: replace 50 with dt
    }
    last_call_ms = now;

    // calculate p
    p = mincopter.g.pid_throttle_rate.kP() * z_rate_error;

    // consolidate and constrain target acceleration
    output += p;
    output = constrain_int32(output, -32000, 32000);

    // set target for accel based throttle controller
    set_throttle_accel_target(output);

    // update throttle cruise
    // TO-DO: this may not be correct because g.rc_3.servo_out has not been updated for this iteration
    if( z_target_speed == 0 ) {
        update_throttle_cruise(mincopter.g.rc_3.servo_out);
    }
}

// get_throttle_althold - hold at the desired altitude in cm
// updates accel based throttle controller targets
// Note: max_climb_rate is an optional parameter to allow reuse of this function by landing controller
void
get_throttle_althold(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate)
{
    int32_t alt_error;
    float desired_rate;
    int32_t linear_distance;      // half the distace we swap between linear and sqrt and the distace we offset sqrt.

    // calculate altitude error
    alt_error    = target_alt - mcstate.current_loc.alt;

    // check kP to avoid division by zero
    if( mincopter.g.pi_alt_hold.kP() != 0 ) {
        linear_distance = ALT_HOLD_ACCEL_MAX/(2*mincopter.g.pi_alt_hold.kP()*mincopter.g.pi_alt_hold.kP());
        if( alt_error > 2*linear_distance ) {
            desired_rate = safe_sqrt(2*ALT_HOLD_ACCEL_MAX*(alt_error-linear_distance));
        }else if( alt_error < -2*linear_distance ) {
            desired_rate = -safe_sqrt(2*ALT_HOLD_ACCEL_MAX*(-alt_error-linear_distance));
        }else{
            desired_rate = mincopter.g.pi_alt_hold.get_p(alt_error);
        }
    }else{
        desired_rate = 0;
    }

    desired_rate = constrain_float(desired_rate, min_climb_rate, max_climb_rate);

    // call rate based throttle controller which will update accel based throttle controller targets
    get_throttle_rate(desired_rate);

    // update altitude error reported to GCS
		// TODO Does this actually need to be reported
    mincopter.altitude_error = alt_error;

    // TO-DO: enabled PID logging for this controller
}

// get_throttle_althold_with_slew - altitude controller with slew to avoid step changes in altitude target
// calls normal althold controller which updates accel based throttle controller targets
void
get_throttle_althold_with_slew(int32_t target_alt, int16_t min_climb_rate, int16_t max_climb_rate)
{
    float alt_change = target_alt-mincopter.controller_desired_alt;
    // adjust desired alt if motors have not hit their limits
    if ((alt_change<0 && !mincopter.motors.limit.throttle_lower) || (alt_change>0 && !mincopter.motors.limit.throttle_upper)) {
        mincopter.controller_desired_alt += constrain_float(alt_change, min_climb_rate*0.02f, max_climb_rate*0.02f);
    }

    // do not let target altitude get too far from current altitude
    mincopter.controller_desired_alt = constrain_float(mincopter.controller_desired_alt,mcstate.current_loc.alt-750,mcstate.current_loc.alt+750);

    get_throttle_althold(mincopter.controller_desired_alt, min_climb_rate-250, max_climb_rate+250);   // 250 is added to give head room to alt hold controller
}

// get_throttle_rate_stabilized - rate controller with additional 'stabilizer'
// 'stabilizer' ensure desired rate is being met
// calls normal throttle rate controller which updates accel based throttle controller targets
void
get_throttle_rate_stabilized(int16_t target_rate)
{
    // adjust desired alt if motors have not hit their limits
    if ((target_rate<0 && !mincopter.motors.limit.throttle_lower) || (target_rate>0 && !mincopter.motors.limit.throttle_upper)) {
        mincopter.controller_desired_alt += target_rate * 0.02f;
    }

    // do not let target altitude get too far from current altitude
    mincopter.controller_desired_alt = constrain_float(mincopter.controller_desired_alt,mcstate.current_loc.alt-750,mcstate.current_loc.alt+750);

#if AC_FENCE == ENABLED
    // do not let target altitude be too close to the fence
    // To-Do: add this to other altitude controllers
    if((mcstate.fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        float alt_limit = mcstate.fence.get_safe_alt() * 100.0f;
        if (mincopter.controller_desired_alt > alt_limit) {
            mincopter.controller_desired_alt = alt_limit;
        }
    }
#endif

    get_throttle_althold(mincopter.controller_desired_alt, -mincopter.g.pilot_velocity_z_max-250, mincopter.g.pilot_velocity_z_max+250);   // 250 is added to give head room to alt hold controller
}

// get_throttle_land - high level landing logic
// sends the desired acceleration in the accel based throttle controller
// called at 50hz
void
get_throttle_land()
{
    // if we are above 10m
    if (mcstate.current_loc.alt >= LAND_START_ALT) {
        get_throttle_althold_with_slew(LAND_START_ALT, -mcstate.wp_nav.get_descent_velocity(), -abs(mincopter.g.land_speed));
    }else{
        get_throttle_rate_stabilized(-abs(mincopter.g.land_speed));

        // disarm when the landing detector says we've landed and throttle is at min (or we're in failsafe so we have no pilot thorottle input)
#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        if( mincopter.ap.land_complete && (mincopter.g.rc_3.control_in == 0 || mcstate.failsafe.radio) ) {
#else
        if (mincopter.ap.land_complete) {
#endif
            init_disarm_motors();
        }
    }
}

// reset_land_detector - initialises land detector
void reset_land_detector()
{
    set_land_complete(false);
    mincopter.land_detector = 0;
}

// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// returns true if we have landed
bool update_land_detector()
{
    // detect whether we have landed by watching for low climb rate and minimum throttle
    if (abs(mincopter.climb_rate) < 20 && mincopter.motors.limit.throttle_lower) {
        if (!mincopter.ap.land_complete) {
            // run throttle controller if accel based throttle controller is enabled and active (active means it has been given a target)
            if( mincopter.land_detector < LAND_DETECTOR_TRIGGER) {
                mincopter.land_detector++;
            }else{
                set_land_complete(true);
                mincopter.land_detector = 0;
            }
        }
    }else if (mincopter.g.rc_3.control_in != 0 || mcstate.failsafe.radio){    // zero throttle locks land_complete as true
        // we've sensed movement up or down so reset land_detector
        mincopter.land_detector = 0;
        if(mincopter.ap.land_complete) {
            set_land_complete(false);
        }
    }

    // return current state of landing
    return mincopter.ap.land_complete;
}

/*
 *  reset all I integrators
 */
void reset_I_all(void)
{
    reset_rate_I();
    reset_throttle_I();
    //reset_optflow_I();
}

void reset_rate_I()
{
    mincopter.g.pid_rate_roll.reset_I();
    mincopter.g.pid_rate_pitch.reset_I();
    mincopter.g.pid_rate_yaw.reset_I();
}

void reset_throttle_I(void)
{
    // For Altitude Hold
    mincopter.g.pi_alt_hold.reset_I();
    mincopter.g.pid_throttle_accel.reset_I();
}

void set_accel_throttle_I_from_pilot_throttle(int16_t pilot_throttle)
{
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    mincopter.g.pid_throttle_accel.set_integrator(pilot_throttle-mincopter.g.throttle_cruise);
}


