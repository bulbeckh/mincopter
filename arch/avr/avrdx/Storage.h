
#pragma once

#include <AP_HAL.h>

#include "avrdx/AP_HAL_AVRDx_Namespace.h"

class AP_HAL_AVRDx::AVRDxEEPROMStorage : public AP_HAL::Storage {
	public:
		AVRDxEEPROMStorage() {}
		void init(void* machtnichts) {}
		uint8_t  read_byte(uint16_t loc);
		uint16_t read_word(uint16_t loc);
		uint32_t read_dword(uint16_t loc);
		void     read_block(void *dst, uint16_t src, size_t n);

		void write_byte(uint16_t loc, uint8_t value);
		void write_word(uint16_t loc, uint16_t value);
		void write_dword(uint16_t loc, uint32_t value);
		void write_block(uint16_t dst, const void* src, size_t n);
};

