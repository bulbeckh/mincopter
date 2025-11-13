
#pragma once

#include "mcstate_interface.h"

class StateNone : public MCState
{
	public:
		StateNone(void) : MCState() { }

		void update(void) override;

		void init_derived(void) override;

};


