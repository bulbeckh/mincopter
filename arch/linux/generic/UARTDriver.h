
#pragma once

#include <arch/linux/generic/AP_HAL_Generic.h>

#include <cstdio>

class generic::GenericUARTDriver : public AP_HAL::UARTDriver {
	public:
		GenericUARTDriver(bool default_console);

		/* Generic implementations of UARTDriver virtual methods */
		void begin(uint32_t b) override;

		void begin(uint32_t b, uint16_t rxS, uint16_t txS) override;

		void end(void) override;

		void flush(void) override;

		bool is_initialized(void) override { return initialised; }

		void set_blocking_writes(bool blocking) override;

		bool tx_pending(void) override;

		/* Generic implementations of Stream virtual methods */
		int16_t available(void) override;
		int16_t txspace(void) override;
		int16_t read(void) override;

		/* Generic implementations of Print virtual methods */
		size_t write(uint8_t c) override;
		size_t write(const uint8_t *buffer, size_t size) override;

	private:
		int master_fd;

		/* @brief Whether we have initialised */
		bool initialised{false};

		/* @brief Pointer to file descriptor for this UART */
		FILE* generic_fp;

		/* @brief Whether this UART is the console UART (in which case we use STDIN and STDOUT and do not create pseudo-terminals */
		bool _console;

	public:
		/*
		void set_device_path(const char *path);

		void _timer_tick(void);
		*/

	private:
		/*
		const char *device_path;
		int _rd_fd;
		int _wr_fd;
		bool _nonblocking_writes;
		volatile bool _initialised;
		volatile bool _in_timer;

		// we use in-task ring buffers to reduce the system call cost
		// of ::read() and ::write() in the main loop
		uint8_t *_readbuf;
		uint16_t _readbuf_size;

		// _head is where the next available data is. _tail is where new
		// data is put
		volatile uint16_t _readbuf_head;
		volatile uint16_t _readbuf_tail;

		uint8_t *_writebuf;
		uint16_t _writebuf_size;
		volatile uint16_t _writebuf_head;
		volatile uint16_t _writebuf_tail;

		int _write_fd(const uint8_t *buf, uint16_t n);
		int _read_fd(uint8_t *buf, uint16_t n);
		uint64_t _last_write_time;
		*/
};

