
#pragma once

#include "controller_interface.h"

class None_Controller : public MC_Controller
{
	public:
		None_Controller() :
			MC_Controller()
		{
		}

	public:
		void run() override;

};
