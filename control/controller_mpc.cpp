
#include "controller_mpc.h"


void MPC_Controller::run()
{

	/* **Part 1.** Retrieve current state dynamics and update l and u vectors with (linearised) dynamics (from A matrix)
	 * States 0:3 (the position in inertial frame) will come from inertial nav. States 3:6 (attitude) will come from the AHRS roll,pitch,yaw sensors
	 * Statest 6:9 (the translational velocities) will come from the inertial nav too. States 9:12 (body frame rates) will come from
	 * the gyros (do these need to be converted to earth frame rates??)
	 *
	 * **Part 2. Calculate and retrieve reference trajectories and update q matrix
	 * These should be constant for now at the desired location
	 *
	 * **Part 3.** Solve optimisation problem
	 *
	 * **Part 4.** Log the results for the time being to see that control action is being produced
	 *
	 */


}

