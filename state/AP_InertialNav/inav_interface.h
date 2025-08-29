
#pragma once

class MC_InertialNav
{
	public:
		MC_InertialNav(void)
	{
	}
	
	public:

		/* @brief Initialise the inertial navigation */
		virtual void inav_init(MCStateData* state) {
			_state = state;
			_inav_init_internal();
		}

		/* @brief Update the inertial navigation. Called via xx TODO */
		virtual void inav_update(void) = 0;

		/* @brief Reset the current attitude representation to zero */
		virtual void inav_reset(void) {
			for (uint8_t i=0;i<3;i++) {
				// TODO If _position and _velocity change to Vector3f types, then zero it the typical way
				_state._position[i] = 0.0f;
				_state._velocity[i] = 0.0f;
			}
			return;
		}

		/* @brief Reset the current position and velocity to the provided values */
		virtual void inav_reset_pos_vel(Vector3f pos, Vector3f vel) {
			_state._position[0] = pos.x;
			_state._position[1] = pos.y;
			_state._position[2] = pos.z;

			_state._velocity[0] = vel.x;
			_state._velocity[1] = vel.y;
			_state._velocity[2] = vel.z;

			return;
		}
	
	private:
		/* @brief Pointer to the state object to be updated on each call to inav_update */
		MCStateData* _state;

		/* @brief Class-specific initialisation method */
		virtual void _inav_init_internal(void) = 0;
};



