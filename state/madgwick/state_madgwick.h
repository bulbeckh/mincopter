
#pragma once

#include "mcstate_interface.h"

class StateMadgwick : public MCState
{
	public:
		StateMadgwick(void) : MCState() { }

		void update(void) override;

		void init_derived(void) override;

	private:
		// TODO

};


