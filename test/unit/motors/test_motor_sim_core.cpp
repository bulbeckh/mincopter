
// Motor (simulation) test

#include <iostream>

#include "gz_interface.h"
GZ_Interface gz_interface;

#include "mcinstance.h"
#include "mcstate.h"
#include <AP_Scheduler.h>       // main loop scheduler

/* NOTE TODO Definitely need to fix this by defining these elsewhere */

/* @brief Interface to the object storing each sensor and other hardware abstraction (DataFlash, Battery, ..) */
MCInstance mincopter;

/* @brief Interface to the scheduler which runs sensor updates and other non-HAL, non-interrupt functions */
AP_Scheduler scheduler;

/* @brief Interface to the state estimation module */
MCState mcstate;

#ifdef TARGET_ARCH_LINUX
    #include "simulation_logger.h"
    SimulationLogger simlog(true);
#endif

const AP_HAL::HAL& hal = mincopter.hal;


/* This test will spin each motors in succession for 1 second.
 *
 * Intended to be a visual 'test' to confirm correct order of motors */

int main(void)
{
	// NO INIT
	
	// Requires a running version of Gazebo MinCopter plugin
	
	gz_interface.setup_sim_socket();

	// Run for 2 seconds with no motors
	for (int i=0;i<4;i++) gz_interface.control_pwm[i] = 1100;
	for (int i=0;i<200;i++) {
		gz_interface.send_control_output();
		gz_interface.recv_state_input();
	}

	// Run two rounds of motor cycling
	for (int i=0;i<4;i++) {
		std::cout << "INLOOP - " << i << "\n";

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

	return 0;

}


