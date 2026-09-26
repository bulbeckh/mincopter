
/*
#include <filesystem>
#include <memory>

#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/common/Console.hh>
*/

#include <thread>
// #include <cassert>

#include <AP_HAL/AP_HAL.h>
#include <arch/AP_HAL/HAL_Interface.h>

#include "gazebo_simulation_test_base.h"

#include <gtest/gtest.h>

/* This test suite is responsible for ensuring that our gazebo interface is created successfully
 * and we are able to communicate with the gazebo simulation.
 *
 * NOTE Before running test executable, we need to source both a valid gz distribution (i.e. via
 * /opt/ros/jazzy/setup.bash and also the mincopter-specific gz setup via ${PROJECT_ROOT}/setup.bash */

using namespace gz;
using namespace sim;

// TODO This is a bad hack which permeates different layers, and breaks the isolation the mc-arch is supposed
// to have because mc-arch now depends on this HAL object
//const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// NOTE TODO We now have a case where the hal object is defined in the header, but statically

// NOTE Perhaps we don't need a separate test derived class here but if we need to add more functionality
// then we should create one
class GzInterfaceTest : public GazeboSimulationTestBase {
	protected:
		GzInterfaceTest() {}

};

TEST_F(GzInterfaceTest, Startup) {

	// Run 100 iterations in new thread - will block until MinCopter GZ Interface connects
	std::thread serverThread([this]() {
		// NOTE There is a nuance in gazebo whereby the first run/iteration of system plugins (like the ardupilot plugin)
		// will not be called, so we add 1 to our desired number of simulation steps
		this->server->Run(true, 1 + 990, false);
	});

	// TODO Add trapping of signal ctrl+C so that we break the mincopter loop

	std::cout << "Server setup complete" << std::endl;
	
	// Loop the simulation for 1s. Note that here, the simulation loop real-time step is driven by
	// Gazebo and is not limited here to a tightly 10ms loop as it is in the full executable.
	for (int i=0;i<100;i++) {
		if (!hal.sim->connected()) break;
		hal.sim->tick(10000);
	}

	serverThread.join();

	// Run tests
	//
	// 1. Iterations is as expected
	EXPECT_EQ(server->IterationCount(), 991);

	// 2. Server is stopped at end of our desired number of iterations
	EXPECT_EQ(server->Running(), false);

	std::cout << "Finished sucessfully\n";
}

