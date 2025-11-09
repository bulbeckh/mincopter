
#pragma once

/* Implementation of a complementary filter for state estimation */

class StateComplementary : public MCState
{
	public:
		StateComplementary(void) : MCState() { }

		void update(void) override;

		void init_derived(void) override;

	private:
		/* @brief Complementary filter computes euler angles so we maintain the state in euler angles */
		Vector3f euler_internal;

		/* @brief Whether we have completed first update */
		uint8_t _first_update;

		// TODO Alpha should be passed in as an initialised value somehow
		/* @brief The alpha value of the complementary filter */
		float alpha = 0.998;

		float alpha_yaw = 0.8;




};


