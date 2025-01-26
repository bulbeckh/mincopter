


/* Behaviour tree implementation of ArduCopter navigation and control algorithm
*
*
*
*/

#include <bt.h>

#include "mcinstance.h"

extern MCInstance* mci;


// Create tree

/* @brief Runs state updates and controllers to navigate between waypoints
*/
NodeExecutionResult BT_MC_navigate_to_waypoint()
{

}

/* @brief Copter startup and initialisation code
*/
NodeExecutionResult BT_MC_boot()
{
	// add the init_ardupilot code here
}

/* @brief Wait for GPS lock
*/
NodeExecutionResult BT_MC_wait_for_lock()
{
	// NOTE Needs to have a decorator that repeats until GPS lock
}




