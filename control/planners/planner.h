// Planner Class
//
//


#include <AC_Fence.h>
#include <AC_WPNav.h>

#include "failsafe.h"

class MC_Planner
{

	public:

		MC_Planner() : 
			// add assignmnets
		{
		}


		/* @brief Entry point for planner function
		 */
		virtual void run() = 0;



	public:
		AP_FAILSAFE_T failsafe;

		AC_Fence fence;


		AC_WPNav wp_nav;


		/* @brief Desired roll/pitch/yaw values for determination by planner and usage by controller
		 */
		int16_t control_roll;
		int16_t control_pitch;
		int32_t control_yaw;

}

