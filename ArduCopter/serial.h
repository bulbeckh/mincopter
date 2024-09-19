// For printing serial information to console

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include "system.h"

#include <AP_Common.h>

extern AP_HAL::BetterStream* cliSerial;

extern struct Location current_loc;
extern float cos_roll_x;
extern float cos_pitch_x;
extern float cos_yaw;


/* @brief Prints information about current location
*/
void print_GPS();

/* @brief Prints the current roll/pitch/yaw values
*/
void print_RPY();


