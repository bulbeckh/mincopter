
#pragma once

/* Implementation of a complementary filter for state estimation */

#include "mcstate_interface.h"

class StateSim : public MCState
{
	public:
		StateSim(void) : MCState() { }

		void update(void) override;

		void init_derived(void) override;
};


