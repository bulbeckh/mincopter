#pragma once


// TODO Change the names of the planner classes

#ifdef PLANNER_WAYPOINT
	#include "planner_waypoint.h"
	extern WP_Planner planner;
#elif PLANNER_NONE
	#include "planner_none.h"
	extern None_Planner planner;
#else
/*
 * Add additional planner implementations here
 */
	#error No PLANNER implementation selected 
#endif


