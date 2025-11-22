#include <AP_HAL.h>

#include <avr/io.h>
#include <avr/eeprom.h>

#include "avrdx/Storage.h"

using namespace AP_HAL_AVRDx;

uint8_t AVRDxEEPROMStorage::read_byte(uint16_t loc) {
    return eeprom_read_byte((uint8_t*)loc);
}

uint16_t AVRDxEEPROMStorage::read_word(uint16_t loc) {
    return eeprom_read_word((uint16_t*)loc);
}

uint32_t AVRDxEEPROMStorage::read_dword(uint16_t loc) {
    return eeprom_read_dword((uint32_t*)loc);
}

void AVRDxEEPROMStorage::read_block(void *dst, uint16_t src, size_t n) {
    eeprom_read_block(dst,(const void*)src,n);
}

void AVRDxEEPROMStorage::write_byte(uint16_t loc, uint8_t value) {
    uint8_t b = eeprom_read_byte((uint8_t*)loc);
    if (b != value) {
        eeprom_write_byte((uint8_t*)loc, value);
    }
}

void AVRDxEEPROMStorage::write_word(uint16_t loc, uint16_t value) {
    write_block(loc, &value, sizeof(value));
}

void AVRDxEEPROMStorage::write_dword(uint16_t loc, uint32_t value) {
    write_block(loc, &value, sizeof(value));
}

void AVRDxEEPROMStorage::write_block(uint16_t dst, const void *src, size_t n) {
    uint8_t *p = (uint8_t *)src;
    while (n--) {
        write_byte(dst++, *p++);
    }
}

