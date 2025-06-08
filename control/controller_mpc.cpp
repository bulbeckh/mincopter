
#include "controller_mpc.h"

#include "mcstate.h"
extern MCState mcstate;

#include "mcinstance.h"
extern MCInstance mincopter;

// NOTE The solver is defined in the generated **workspace.c** file
extern "C" {
	extern OSQPSolver solver;
}

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
	 * **Part 4.** Send next input to mixer. Log the results for the time being to see that control action is being produced
	 *
	 */

	// Part 1
	float state12[12];

	// Position & velocity values are in cm. Convert to m to match state dynamic equations used by MPC.
	Vector3f position = mcstate.inertial_nav.get_position();
	Vector3f velocity = mcstate.inertial_nav.get_velocity();

	state12[0] = position.x*1e-2;
	state12[1] = position.y*1e-2;
	state12[2] = position.z*1e-2;

	state12[6] = velocity.x*1e-2;
	state12[7] = velocity.y*1e-2;
	state12[8] = velocity.z*1e-2;

	// NOTE normally <roll,pitch,yaw>_sensor would be used to get angles as an int32_t in deg*100 but can use <roll,pitch,yaw> angles directly here
	
	state12[3] = mcstate.ahrs.roll;
	state12[4] = mcstate.ahrs.pitch;
	state12[5] = mcstate.ahrs.yaw;

	// Gyro rates are already in rad/s
	Vector3f ang_rates = mincopter.get_gyro();
	state12[9] = ang_rates.x;
	state12[10] = ang_rates.y;
	state12[11] = ang_rates.z;
	
	// Update L and U constraint matrices with system dynamics
	for (int i=0;i<12;i++) {
		lower_constraint[120+i] = 0.0f;
		for (int j=0;j<12;j++) {
			/* Here, we are reusing the original l and u matrices so as to not remove the other constraints.
			* Our state dynamic constraints start at index 120 */
			lower_constraint[120+i] += linearised_A[i*12+j]*state12[j];
		}
		upper_constraint[120+i] = lower_constraint[120+i];
	}

	// Part 2.

	// Update Q matrix with reference trajectory
	for (int i=0;i<10;i++) {
		for (int j=0;j<12;j++) {
			// TODO This equation assumes an identity P matrix for the MPC problem. The correct formulation is q = -1*x_ref^{T}@P
			q_constraint[i*12+j] = -1*state_reference[i*12+j];
		}
	}

	osqp_update_data_vec(&solver, q_constraint, lower_constraint, upper_constraint);

	// Part 3.
	
	// Solve MPC problem
	exitflag = osqp_solve(&solver);

	// Part 4.
	
	// Update control vector TODO This is a vector now but should soon call a method to update the mixer values.
	float control_vector[4];
	for (int i=0;i<4;i++) {
		control_vector[i] = solver.solution.x[120+i];
	}
	






}

