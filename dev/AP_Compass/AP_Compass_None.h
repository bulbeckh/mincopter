
#pragma once

/* Stub class for AP_Compass */

class AP_Compass_None : public Compass
{
	public:
		AP_Compass_None(void) { }

		bool read(void) override { return true; }

		void accumulate(void) override { return; }

};


