
#include <AP_HAL/AP_HAL.h>
#include <arch/AP_HAL/HAL_Interface.h>

#include "gazebo_simulation_test_base.h"

#include <gtest/gtest.h>

#include <thread>

/* This test suite is responsible for ensuring that our gazebo interface is created successfully
 * and we are able to communicate with the gazebo simulation.
 *
 * NOTE Before running test executable, we need to source both a valid gz distribution (i.e. via
 * /opt/ros/jazzy/setup.bash and also the mincopter-specific gz setup via ${PROJECT_ROOT}/setup.bash */

using namespace gz;
using namespace sim;

// TODO This is a bad hack which permeates different layers, and breaks the isolation the mc-arch is supposed
// to have because mc-arch now depends on this HAL object
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// NOTE TODO We now have a case where the hal object is defined in the header, but statically

// NOTE Perhaps we don't need a separate test derived class here but if we need to add more functionality
// then we should create one
class GzInterfaceTest : public GazeboSimulationTestBase {
	protected:
		GzInterfaceTest() {}

};

TEST_F(GzInterfaceTest, Startup) {

	// By this point, we should have an initialised server
	std::thread serverThread = std::thread([this]() {
		this->server->Run(true, 1 + 10*10, false);
	});
	
	hal.init(0, NULL);

	for (int i=0;i<10;i++) {
		hal.sim->tick(1000);
	}

	serverThread.join();

	// Run tests
	
	// 1. Gyro readings are zero
	EXPECT_NEAR(hal.sim->last_sensor_state.imu_gyro_x, 0.0, 1e-1);
	EXPECT_NEAR(hal.sim->last_sensor_state.imu_gyro_y, 0.0, 1e-1);
	EXPECT_NEAR(hal.sim->last_sensor_state.imu_gyro_z, 0.0, 1e-1);

	// 2. Accelerometer readings are as expected
	EXPECT_NEAR(hal.sim->last_sensor_state.imu_accel_x, 0.0, 1e-1);
	EXPECT_NEAR(hal.sim->last_sensor_state.imu_accel_y, 0.0, 1e-1);
	EXPECT_NEAR(hal.sim->last_sensor_state.imu_accel_z, -9.81, 1e-1);

	// 1. Server is stopped at end of our desired number of iterations
	EXPECT_EQ(server->Running(), false);

	std::cout << "Finished sucessfully\n";
}

