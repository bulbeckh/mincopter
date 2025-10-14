
#pragma once

#include <AP_HAL/AP_HAL.h>

#include <arch/arm/stm32/AP_HAL_STM32_Namespace.h>

#include "stm32f4xx_hal.h"

#define STM32_SCHEDULER_MAX_TIMER_PROCS 16

class stm32::STM32Scheduler : public AP_HAL::Scheduler {

	public:
		STM32Scheduler(void);

		/* @brief Setup the scheduler including timer clock initialisation, delay clock initialisation and registration of callbacks */
		void init(void*);

		/* @brief Delay the system by a number of milli-seconds */
		void delay(uint16_t ms);

		/* @brief Delay the system by a number of micro-seconds */
		void delay_microseconds(uint16_t us);

		/* @brief Return the number of milli-seconds elapsed since system start */
		uint32_t millis(void);

		/* @brief Return the number of micro-seconds elapsed since system start */
		uint32_t micros(void);

		/* @brief Add a callback (member) function to be triggered during the timer process interrupt (runs @1kHz by default) */
		void register_timer_process(AP_HAL::MemberProc);

		// TODO Not yet supported for this target
		/* @brief Add a callback function to use during milli-second delays */
		void register_delay_callback(AP_HAL::Proc, uint16_t min_time_ms);

		// TODO Not yet supported for this target
		/* @brief Add a callback function to perform IO tasks (may run on a different timer) */
		void register_io_process(AP_HAL::MemberProc);

		// TODO Not yet supported for this target
		/* @brief Add a callback function to run on every timer interrupt, regardless if we are suspended */
		void register_timer_failsafe(AP_HAL::Proc, uint32_t period_us);

		/* @brief Stop timer processes from running */
		void suspend_timer_procs(void);

		/* @brief Resume running timer processes */
		void resume_timer_procs(void);

		/* @brief Returns true if we are currently in the timer process interrupt callback function */
		bool in_timerprocess(void);

		// TODO Bad names - change them
		/* @brief Whether we have initialised our system yet (HAL and drivers) */
		bool system_initializing(void);

		/* @brief Set system to initialized. This is the final function call before we start the core loop */
		void system_initialized(void);

		/* @brief Dump an error message to the console and run an endless loop */
		void panic(const prog_char_t *errormsg);

		/* @brief Reboot the microcontroller via NVIC_SystemReset */
		void reboot(bool hold_in_bootloader);

		// TODO Not implemented
		void time_shift(uint32_t shift_ms);

	public:
		// TODO Update with STM32 implementation

		/* @brief HAL Timer Handle instance */
		TIM_HandleTypeDef timer_handle;

		/* @brief HAL Timer Handle for the microsecond delay timer */
		TIM_HandleTypeDef delay_handle;

		/* @brief Counter used to hold the number of milli-seconds elapsed */
		uint32_t _ms_counter;

		/* @brief Flag for whether we should run timer processes */
		bool _suspended;

		/* @brief Timer process to flash the LED each second to indicate a running **loop** function */
		void _timer_led_heartbeat(void);

		/* @brief Callback to run each of the timer processes at 1kHz */
		void _run_timer_processes(void);

	private:

		AP_HAL::MemberProc _timer_proc[STM32_SCHEDULER_MAX_TIMER_PROCS];

		uint8_t _num_timer_procs;

		volatile bool _in_timer_proc;

		/* @brief Flag for whether system initialisation is complete */
		bool _initialised;

		/*
		struct timespec _sketch_start_time;    
		void _timer_handler(int signum);
		void _microsleep(uint32_t usec);

		AP_HAL::Proc _delay_cb;
		uint16_t _min_delay_cb_ms;

		AP_HAL::Proc _failsafe;

		bool _initialized;
		volatile bool _timer_pending;

		volatile bool _timer_suspended;


		AP_HAL::MemberProc _io_proc[LINUX_SCHEDULER_MAX_TIMER_PROCS];
		uint8_t _num_io_procs;
		volatile bool _in_io_proc;

		volatile bool _timer_event_missed;

		pthread_t _timer_thread_ctx;
		pthread_t _io_thread_ctx;
		pthread_t _uart_thread_ctx;

		void *_timer_thread(void);
		void *_io_thread(void);
		void *_uart_thread(void);

		void _run_timers(bool called_from_timer_thread);
		void _run_io(void);
		void _setup_realtime(uint32_t size);
		*/

};


