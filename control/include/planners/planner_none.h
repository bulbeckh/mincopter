
#pragma once

#include "planner_interface.h"

class None_Planner : public MC_Planner
{
	public:
		None_Planner() :
			MC_Planner()
		{

		}

	public:

		void run(void) override;

};

