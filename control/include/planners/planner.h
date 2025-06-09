#pragma once

#ifdef PLANNER_WAYPOINT
	#include "planner_waypoint.h"
	extern WP_Planner planner;
/*
 * Add additional planner implementations here
 */
#else
	#error No PLANNER implementation selected 
#endif


