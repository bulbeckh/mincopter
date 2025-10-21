
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <arch/linux/generic/AP_HAL_Generic.h>

/* Scheduler class for simulation target
 *
 * The simulation differs greatly from the typical execution environment. It is run
 * in lockstep with Gazebo at 1kHz in Gazebo but at 100Hz from the perspective of MinCopter.
 *
 * An updated world state and sensor readings are transmitted at 100Hz at which point MinCopter
 * runs the state update and control determination pipeline and transmits control outputs (in the
 * form of PWM signals) back to Gazebo which runs 10 iterations (10ms simulation time) and repeats.
 *
 * In simulation, we don't need things like an IO thread or a delay callback as these happen effectively
 * instaneously (<10us) for a generic (linux) target.
 *
 * ## Potential pitfalls
 * On bare-metal baords, the timer processes by default each run at 1kHz but our MinCopter computation
 * pipeline runs at 100Hz. Anything that uses the timer process functionality (mostly sensor _poll methods)
 * should instead delegate this computation to the gazebo plugin. An example would be the gyro and accel
 * update and filtering which is done in the Gazebo plugin rather than the simulated sensor driver. The
 * driver simply reads the updated/filtered value each 10ms (100Hz).
 *
 * ## Simulation Pipeline
 *
 * -> Simulation is at timestamp 0.000s and runs first 10 iterations. Timestamp at 0.01s.
 * -> Simulation sends state packet with
 *
 */

class generic::GenericScheduler : public AP_HAL::Scheduler {
	public:
		GenericScheduler(void);

		/* @brief Initialise the simulation timing */
		void init(void*) override;

		void delay(uint16_t ms);
		void delay_microseconds(uint16_t us);

		uint32_t millis(void);
		uint32_t micros(void);

		void register_delay_callback(AP_HAL::Proc, uint16_t min_time_ms);
		void register_timer_process(AP_HAL::MemberProc);
		void register_io_process(AP_HAL::MemberProc);
		void register_timer_failsafe(AP_HAL::Proc, uint32_t period_us);

		void suspend_timer_procs(void);
		void resume_timer_procs(void);

		bool in_timerprocess(void);

		bool system_initializing(void);
		void system_initialized(void); 

		void panic(const prog_char_t *errormsg);
		void reboot(bool hold_in_bootloader);

		// NOTE Ignore both of these - no implementation and will likely be removed from HAL
		//void set_timer_speed(uint16_t speed_hz);
		//void time_shift(uint32_t shift_ms);
	
	private:

		/* @brief Flag to run timer processes */
		bool _timer_suspended{false};

		bool _initialized{false};

	
};



