
#include <AP_InertialSensor.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/HAL_Interface.h>

#include <AP_Math.h>

// NOTE This is not a reference to a hal that already exists (like the one defined in mincopter.cpp). We need to create our own here.
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Should return 0 on successful tests and 1 on any other status
uint8_t run_unit_tests(AP_InertialSensor& _imu)
{
	/* Ultimately, the IMU tests should be checking that we have a valid reading for each of the 6 orientations, and that
	 * the reading is consistent with the NED frame convention that MinCopter uses.
	 *
	 * In simulation, we can simply rotate the quadrotor using the sim interface but for the physical sensors, we need to
	 * have a prompt (via the console) for the user to rotate the quadrotor. */

	// Initialise IMU
	_imu.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);

	/* Our orientation order and expected readings is as follows:
	 *
	 * 1. Stationary orientation, quadrotor forward pointing north 	- {    0,    0, -9.8 }
	 * 2. Quadrotor forward pointing directly up 					- {  9.8,    0,    0 }
	 * 3. Quadrotor right-wing pointing directly up    				- {    0,  9.8,    0 }
	 * 4. Quadrotor upside-down 									- {    0,    0,  9.8 }
	 * 5. Quadrotor forward pointing directly down					- { -9.8,    0,    0 }
	 * 6. Quadrotor right-wing pointing directly down				- {    0, -9.8,    0 }
	 *
	 * We sequentially rotate to each rotation and then wait until the expected reading matches for at least 3 seconds, or
	 * until we reach a 15-second timeout. */

	float target_readings[6][3] = {
		{    0,    0, -9.8 },
		{  9.8,    0,    0 },
		{    0,  9.8,    0 },
		{    0,    0,  9.8 },
		{ -9.8,    0,    0 },
		{    0, -9.8,    0 }
	};

	// These are roll,pitch,yaw angles associated with each orientation. They are relative to the original (0,0,0,0,0,0) stationary pose
	float orientations[6][3] = {
		{0,0,0},
		{0, 1.5708, 0},
		{-1.5708, 0, 0},
		{3.141, 0, 0},
		{0, -1.5708, 0},
		{1.5708, 0, 0}
	};

	uint8_t orientation_number = 0;
	uint16_t correct_reading_counter = 0;
	uint16_t timeout_counter = 0;

	// Run this loop at 100Hz
	for (int i=0;i<1e6;i++) {
		uint32_t start = hal.scheduler->micros();

		// Do checks for progressing test state
		if (orientation_number==6) {
			// We have completed all tests and can return now
			hal.console->printf("IMU tests successful..\r\n");
			break;
		}
		
		if (correct_reading_counter>=300) {
			// We have reached 3 seconds (300 readings) of expected measurements so we progress to next test
			orientation_number++;
			continue;
		}

		if (timeout_counter>=1500) {
			// We have reached 15 seconds without a correct reading so we fail the test
			hal.console->printf("IMU tests failed with timeout..\r\n");
			break;
		}

#if defined(MC_IMU_SIM)
		// TODO Rotate to desired orientation

		// We always use 5m in the air so we don't have any unexpected ground collisions
		hal.sim->set_mincopter_position(0,0,-5);
		//hal.sim->set_mincopter_velocity(0,0,0);
		hal.sim->set_mincopter_linvelocity(0,0,0);

		// Set the correct position
		hal.sim->set_mincopter_attitude(
				orientations[orientation_number][0],
				orientations[orientation_number][1],
				orientations[orientation_number][2]
			);
#else
		// TODO Prompt user to rotate to desired rotation and log a prompt to screen every second
#endif

#if defined(MC_IMU_SIM)
		// Tick simulation for a 100Hz interval
		hal.sim->tick(10000);
#endif

		bool status = _imu.update();

		if (!status) {
			hal.console->printf("Error in IMU read\n");
			return 1;
		}

		Vector3f accel = _imu.get_accel();

		//hal.console->printf("%d,X: %f, Y: %f, Z: %f\n", i, accel.x, accel.y, accel.z);

		// Check that our measurement matches what is expected for this index, within a certain tolerance

		if ( fabs(target_readings[orientation_number][0]-accel.x) < 1e-3 &&
				fabs(target_readings[orientation_number][1] - accel.y) < 1e-3 &&
				fabs(target_readings[orientation_number][2] - accel.z) < 1e-3 ) {

			// Increment the correct reading counter
			correct_reading_counter++;
		} else {
			// Reset the correct reading counter
			correct_reading_counter = 0;
		}

		timeout_counter++;

		// Delay
		uint32_t elapsed = hal.scheduler->micros() - start;
		//if (elapsed < 10000) hal.scheduler->delay(10000-elapsed);
		// TODO Confirm that in simulation, the delay is actually working correctly
	}

	return 0;
}

int main()
{
	// Core setup before actual testing
	hal.init(0, NULL);
	
	// TODO NOTE We would never have an instance where a hardware target has all types of IMUs should maybe
	// remove the MC_TEST_IMU_ALL flag/option

#if defined(MC_IMU_MPU6050) || defined(MC_TEST_IMU_ALL)
	AP_InertialSensor_MPU6050 imu;
	run_unit_tests(imu);
#elif defined(MC_IMU_MPU6000) || defined(MC_TEST_IMU_ALL)
	AP_InertialSensor_MPU6000 imu;
	run_unit_tests(imu);
#elif defined(MC_IMU_ICM20948) || defined(MC_TEST_IMU_ALL)
	AP_InertialSensor_ICM20948 imu;
	run_unit_tests(imu);
#elif defined(MC_IMU_L3G4200D) || defined(MC_TEST_IMU_ALL)
	AP_InertialSensor_L3G4200D imu;
	run_unit_tests(imu);
#elif defined(MC_IMU_SIM) || defined(MC_TEST_IMU_ALL)
	// TODO Do we need to add/instantiate any other sim parameters here or is that handled in the hal.sim object
	AP_InertialSensor_Sim imu;
	run_unit_tests(imu);
#endif

	// Test 1
	//
	// Test 2
	//
	// Test 3
	//
	// ...
	//

}

