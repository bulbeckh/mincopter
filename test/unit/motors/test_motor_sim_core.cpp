
// Motor (simulation) test


#include <iostream>
#include "gz_interface.h"
GZ_Interface gz_interface;

int main(void)
{
	// NO INIT
	
	// Requires a running version of Gazebo MinCopter plugin

	// Run for 2 seconds with no motors
	for (int i=0;i<4;i++) gz_interface.control_pwm[i] = 1100;
	for (int i=0;i<200;i++) {
		gz_interface.send_control_output();
		gz_interface.recv_state_input();
	}

	// Run two rounds of motor cycling
	for (int i=0;i<4;i++) {
		gz_interface.control_pwm[i] = 1800;
		for (int j=0;j<100;j++) {
			// 10ms loop
			gz_interface.send_control_output();
			gz_interface.recv_state_input();
		}
		// Zero out all motors
		for (int i=0;i<4;i++) gz_interface.control_pwm[i] = 1100;

		// Run for 1 second
		for (int j=0;j<100;j++) {
			// 10ms loop
			gz_interface.send_control_output();
			gz_interface.recv_state_input();
		}
	}

	return;

}


