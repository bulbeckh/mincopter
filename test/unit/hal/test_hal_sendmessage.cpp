

#include <thread>

#include <AP_HAL/AP_HAL.h>
#include <arch/AP_HAL/HAL_Interface.h>

#include <gtest/gtest.h>

/* This tests suite simple creates a HAL object and ensures that we initialise properly */

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

TEST(HalSendMessage, Init) {

	hal.init(0, NULL);

	hal.scheduler->system_initialized();

	// 1. Assert that the socket is connected

	hal.sim->send_control_output(false);

	// TODO Using hal.scheduler should not be the correct way to check initialisation
	// Run tests
	//
	// 1. Assert that we have finished initialisation
	EXPECT_EQ(hal.scheduler->system_initializing(), false);
}

