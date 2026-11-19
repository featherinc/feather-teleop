#include "em_eeprom.hpp"
#include "../src/ota_flash/FlashTxx.hpp"
#include <stdlib.h>

flash_elem_t* flash_base_addr = NULL;
flash_elem_t* reboot_addr = NULL;

extern "C" void eepromemu_flash_write(void* addr, const void* data, uint32_t len) {
    (void)addr; (void)data; (void)len;
}

extern "C" void eepromemu_flash_erase_sector(void* addr) {
    (void)addr;
}

uintptr_t get_emulated_flash_base_addr(void) {
    if (flash_base_addr == NULL) {
        flash_base_addr = (flash_elem_t*)malloc(FLASH_SIZE);
    }
    return (uintptr_t)flash_base_addr;
}

uintptr_t get_emulated_reboot_addr(void) {
    if (reboot_addr == NULL) {
        reboot_addr = (flash_elem_t*)malloc(sizeof(flash_elem_t));
    }
    return (uintptr_t)reboot_addr;
}