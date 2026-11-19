#pragma once

#include <stdint.h>

// Emulated EEPROM writing and erasing. Currently no-ops.
// Teensy core exports these with C linkage. FlashTxx.hpp declares them the same way.
#ifdef __cplusplus
extern "C" {
#endif
void eepromemu_flash_write(void* addr, const void* data, uint32_t len);
void eepromemu_flash_erase_sector(void* addr);
#ifdef __cplusplus
}
#endif

// Get the emulated flash base address, malloc'ed on first call
uintptr_t get_emulated_flash_base_addr(void);

// Get the emulated reboot address, malloc'ed on first call
uintptr_t get_emulated_reboot_addr(void);
