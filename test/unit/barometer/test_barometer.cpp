
#include <thread>

#include <AP_HAL/AP_HAL.h>
#include <arch/AP_HAL/HAL_Interface.h>

#include "gazebo_simulation_test_base.h"

#include <gz/common/Console.hh>

#include <gtest/gtest.h>

/* This test suite is responsible for testing our barometer in simulation.
 *
 * NOTE Before running test executable, we need to source both a valid gz distribution (i.e. via
 * /opt/ros/jazzy/setup.bash and also the mincopter-specific gz setup via ${PROJECT_ROOT}/setup.bash */

using namespace gz;
using namespace sim;

// TODO This is a bad hack which permeates different layers, and breaks the isolation the mc-arch is supposed
// to have because mc-arch now depends on this HAL object
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// NOTE Perhaps we don't need a separate test derived class here but if we need to add more functionality
// then we should create one
class BarometerTest : public GazeboSimulationTestBase {
	protected:
		BarometerTest() {}

	protected:
		// This is a good example of overriding a configuration without
		// re-implementing the entire base class SetUp
		void SetUp(void) override {
			GazeboSimulationTestBase::SetUp();

			// Set logging verbosity to 1 (error)
			common::Console::SetVerbosity(4);
		}

};

TEST_F(BarometerTest, ExpectedStationaryValues) {

	// Run 100 iterations in new thread - will block until MinCopter GZ Interface connects
	std::thread serverThread([this]() {
		// NOTE There is a nuance in gazebo whereby the first run/iteration of system plugins (like the ardupilot plugin)
		// will not be called, so we add 1 to our desired number of simulation steps
		this->server->Run(true, 1 + 990, false);
	});

	// TODO Add trapping of signal ctrl+C so that we break the mincopter loop
	
	hal.init(0, NULL);
	
	// Loop the simulation for 1s. Note that here, the simulation loop real-time step is driven by
	// Gazebo and is not limited here to a tightly 10ms loop as it is in the full executable.
	for (int i=0;i<100;i++) {
		hal.sim->tick(10000);
		std::cout << "Tick " << i << std::endl;
	}

	serverThread.join();

	// Run tests
	//
	// 1. Barometer data is as expected for each axis
	EXPECT_NEAR(hal.sim->last_sensor_state.pressure, 101323, 10.0);
}


