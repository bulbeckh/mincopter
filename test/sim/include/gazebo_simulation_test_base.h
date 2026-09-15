
#include <filesystem>
#include <memory>
#include <csignal>
#include <atomic>

#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/common/Console.hh>

#include <AP_HAL/AP_HAL.h>
#include <arch/AP_HAL/HAL_Interface.h>

#include <gtest/gtest.h>

/* This test suite is responsible for ensuring that our gazebo interface is created successfully
 * and we are able to communicate with the gazebo simulation.
 *
 * NOTE Before running test executable, we need to source both a valid gz distribution (i.e. via
 * /opt/ros/jazzy/setup.bash and also the mincopter-specific gz setup via ${PROJECT_ROOT}/setup.bash */

using namespace gz;
using namespace sim;

// TODO This is causing all sort of linking issues

// TODO This is a bad hack which permeates different layers, and breaks the isolation the mc-arch is supposed
// to have because mc-arch now depends on this HAL object
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

/* The MinCopter <-> Gazebo interface works as follows:
 *
 * Gazebo					MinCopter
 * -----------------------------------------------------------------
 *
 * PreUpdate: 					tick:
 * 	receive_servo_packet < ----------------  send_control_input
 *		|
 *	apply_motor_forces
 * PostUpdate:
 * 	create_state_json
 * 		|
 * 	send_state           ----------------->  recv_state_input
 *
 * -----------------------------------------------------------------
 *
 * This is one iteration, and corresponds to a single MinCopter tick, but 10 simulation
 * iterations. This is because the MinCopter loop runs at 100Hz (10ms) but Gazebo runs
 * at 1000Hz (1ms).
 *
 * Both processes will block until they receive their required messages. MinCopter will
 * block at recv_state_input and Gazebo at receive_state_packet. */

class GazeboSimulationTestBase : public testing::Test {
	protected:
		GazeboSimulationTestBase() {}

		void SetUp(void) override {
			// TODO Make sure that our simulation hal interface has a guard to check
			// if we have already initialised. Alternatively, we can do a full 'reset'
			// when the hal.init function is called
			hal.init(0,NULL);

			// Setup signal trapping
			sigemptyset(&_signals);
			sigaddset(&_signals, SIGTERM);
			sigaddset(&_signals, SIGINT);
			sigaddset(&_signals, SIGHUP);

			if(pthread_sigmask(SIG_BLOCK, &_signals, nullptr) != 0) throw std::runtime_error("pthread_sigmask failed");

			// Filepath for world file
			const std::filesystem::path world_file = 
				std::filesystem::path(PROJECT_SOURCE_DIR) /
				"ap-gz" /
				"worlds" /
				"iris_runway.sdf";

			// Set logging verbosity to 2 (warn)
			common::Console::SetVerbosity(2);

			serverConfig.SetSdfFile(world_file.string());
			
			// Create server object
			server = std::make_shared<Server>(serverConfig);
		}

		void TearDown(void) override {}

	protected:
		/* @brief Server configuration */
		ServerConfig serverConfig{};

		/* @brief Pointer to server object */
		std::shared_ptr<Server> server;

		sigset_t _signals;

		std::atomic<bool> _received_signal{false};

};

