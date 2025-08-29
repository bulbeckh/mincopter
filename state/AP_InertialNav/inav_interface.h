
#pragma once

class MC_InertialNav
{
	public:
		MC_InertialNav(void)
	{
	}
	
	public:

		/* @brief Initialise the inertial navigation */
		virtual void inav_init(MCStateData* state) = 0;

		/* @brief Update the inertial navigation. Called via xx TODO */
		virtual void inav_update(void) = 0;

		/* @brief Reset the current attitude representation to zero */
		virtual void inav_reset(void) = 0;

		/* @brief Reset the current position and velocity to the provided values */
		virtual void inav_reset_pos_vel(Vector3f pos, Vector3f vel) = 0;
	
	private:
		/* @brief Pointer to the state object to be updated on each call to inav_update */
		MCStateData* _state;
};



