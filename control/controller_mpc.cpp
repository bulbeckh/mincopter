
#include "controller_mpc.h"
#include "osqp_api_types.h"

#ifdef TARGET_ARCH_LINUX
	#include <iostream>

	#include "gz_interface.h"
	extern GZ_Interface gz_interface;

	#include "simulation_logger.h"
	extern SimulationLogger simlog;
#endif

#include "mcstate.h"
extern MCState mcstate;

#include "mcinstance.h"
extern MCInstance mincopter;

// NOTE The solver is defined in the generated **workspace.c** file
extern "C" {
	extern OSQPSolver solver;
}

/* Instantiate MPC_Controller here */
MPC_Controller controller;

void MPC_Controller::run()
{
	// NOTE This is called at 100Hz currently but our linearised system is at 10Hz so we should actually update the MPC every 10 iterations
	static uint8_t mpc_iteration=0;
	if (mpc_iteration==9) {
		// 10th iteration
		mpc_iteration=0;
	} else {
		mpc_iteration+=1;
		return;
	}

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
	Vector3f ang_rates = mincopter.ins.get_gyro();
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
		control_vector[i] = solver.solution->x[120+i];
	}

	/* The MPC solution was linearised around [9.8, 0, 0, 0] which was a change of variables. We need to add back 9.8 to our first
	 * control vector to get the actual required force */
	control_vector[0] += 9.8f;

#ifdef TARGET_ARCH_LINUX
	static uint32_t iter=0;
	if (iter%10==0) {
		std::cout << "MPC Control Output: " << control_vector[0] << " " << control_vector[1] << " " << control_vector[2] << " " << control_vector[3] << "\n";
		iter=0;
	}
	iter++;

	simlog.write_mpc_control_output(control_vector[0], control_vector[1], control_vector[2], control_vector[3]);
#endif
	
	// TODO For now, use a mixer function embedded into the MPC to convert to a PWM signal but later move mixer to own class
	mixer_generate_pwm(control_vector[0], control_vector[1], control_vector[2], control_vector[3]);

}

void MPC_Controller::mixer_generate_pwm(float thrust, float roll, float pitch, float yaw)
{
	// Generate allocation for each motor
	// Conversion from motor force to velocity (using quadratic model)
	// Call the GZ_interface directly to update (don't use AP_Motors)

	float allocation[4];
	float rotor_speed[4]; // rad/s

	/* This allocation matrix has been pre-calculated in the MixerSolution ipynb */

	/* Using standard model quadratic model for rotor thrust: F = k_{t}\omega^{2}
	 *
	 * \omega = sqrt(F/k_{t})
	 *
	 */

	allocation[0] = 83333.333*thrust - 1282051.282*roll + 83333.333*pitch + 83333.333*yaw;
	allocation[1] = 83333.333*thrust + 1282051.282*roll - 83333.333*pitch + 83333.333*yaw;
	allocation[2] = 83333.333*thrust - 1282051.282*roll - 83333.333*pitch + 83333.333*yaw;
	allocation[3] = 83333.333*thrust + 1282051.282*roll + 83333.333*pitch + 83333.333*yaw;

	for (int i=0;i<4;i++) {
		allocation[i] = ap_max(0.0f, allocation[i]);
		rotor_speed[i] = sqrt(allocation[i]);
	}

	/* Scaling
	 *
	 * Simulated motors have a PWM range of [1100,1900]
	 *
	 * Max rotor_speed is 838 RPM ~= 87.7 rad/s
	 *
	 */

	uint32_t pwm[4];

	for (int i=0;i<4;i++) {
		//float calc_max = 2157.55f;
		float calc_max = 1500;
		float rs_limit = ap_min(rotor_speed[i], calc_max);
		pwm[i] = ap_min(1100 + uint32_t((rs_limit/calc_max)*800), 1900);
	}

#ifdef TARGET_ARCH_LINUX

	// Assign control to signal
	for (int i=0;i<4;i++) gz_interface.control_pwm[i] = pwm[i];

	static uint32_t iter2=0;
	if (iter2%10==0) {
		std::cout << "RS: " << rotor_speed[0] << " " << rotor_speed[1] << " " << rotor_speed[2] << " " << rotor_speed[3] << "\n";
		std::cout << "PWM: " << pwm[0] << "  " << pwm[1] << " " << pwm[2] << " " << pwm[3] << "\n";
		iter2=0;
	}
	iter2++;
#endif

}

void MPC_Controller::update_state_reference(float* ref_array)
{

}

void MPC_Controller::update_constant_state_reference(float* ref_array)
{
	// TODO No out of bounds checks here
	for (int i=0;i<10;i++) {
		for (int j=0;j<12;j++) {
			state_reference[i*12+j] = ref_array[j];
		}
	}
}




