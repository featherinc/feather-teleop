#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifndef COMPILE_FOR_TEENSY  // To make the IDE happy
#define __IMXRT1062__
#define ARDUINO_TEENSY41
#endif

#if !defined(__IMXRT1062__) || !defined(ARDUINO_TEENSY41)
#error MCU NOT SUPPORTED
#endif

// The data type of a single element in flash memory. Teensy 4.1 flash memory uses 4-byte individual values.
typedef uint32_t flash_elem_t;

// [<------- code ------->][<--------- buffer --------->][<-- FLASH_RESERVE -->]
// [<------------------------------ FLASH_SIZE ------------------------------->]
// ^FLASH_BASE_ADDR
#define FLASH_ID		    "fw_teensy41"		    // target ID (in code)
#define FLASH_SIZE		    (0x800000)		        // 8MB
#define FLASH_SECTOR_SIZE	(0x1000)		        // 4KB sector size
#define FLASH_WRITE_SIZE	(sizeof(flash_elem_t))	// 4-byte/32-bit writes    
// Reserve top of flash for Teensy 4.1 EEPROM emulation (63×4KB @ 0x607C0000)
// plus a little slack. Too small a reserve makes firmware_buffer_init treat
// EEPROM as "end of code" and leaves a staging area too small for OTA.
#define FLASH_RESERVE		(64*FLASH_SECTOR_SIZE)	// 256KB

#define FLASH_BASE_ADDR	    (0x60000000)		    // code starts here
// By setting this memory address to this value, the MCU will reboot
#define REBOOT_ADDR (0xE000ED0C)
#define REBOOT (*((flash_elem_t*)REBOOT_ADDR) = 0x5FA0004)

// Override the FLASH_BASE_ADDR and REBOOT_ADDR macro to use malloc'ed memory addresses for testing on emulators.
#ifndef COMPILE_FOR_TEENSY
extern uintptr_t get_emulated_flash_base_addr(void);
extern uintptr_t get_emulated_reboot_addr(void);
#undef FLASH_BASE_ADDR
#undef REBOOT_ADDR
#define FLASH_BASE_ADDR (get_emulated_flash_base_addr())
#define REBOOT_ADDR (get_emulated_reboot_addr())
#endif

// Check if a memory address resides within flash memory
#define IN_FLASH(a) ((a) >= FLASH_BASE_ADDR && (a) < FLASH_BASE_ADDR + FLASH_SIZE)


// functions that can be in flash
int flash_write_block(flash_elem_t* addr, char* data, size_t count);

/**
 * Calculates the location of the firmware buffer and allowable buffer size. Stores the results in the given pointers.
 */
void firmware_buffer_init(flash_elem_t** buf_addr_store, size_t* buf_size_store);

/**
 * Erases the current firmware buffer data from flash memory.
 */
void firmware_buffer_free(flash_elem_t* buf_addr, size_t buf_size);

/**
 * Checks if the firmware buffer contains the correct flash ID. Used to verify the given hex file was built for the
 * correct target board.
 */
bool check_flash_id(flash_elem_t* buf_addr, size_t buf_size);


// apparently better - thanks to Frank Boesing.
// Fine to leave in for emulation. The ".fastrun" gets added as an orphan section next to .text section, still runs
#define RAMFUNC __attribute__ ((section(".fastrun"), noinline, noclone, optimize("Os")))

RAMFUNC int flash_sector_not_erased(uintptr_t address);

// functions used to move code from buffer to program flash (must be in RAM)
RAMFUNC void flash_move(uintptr_t dst, uintptr_t src, uint32_t size);


// Teensy core (eeprom.c) exports these with C linkage.
#ifdef __cplusplus
extern "C" {
#endif
void eepromemu_flash_write(void* addr, const void* data, uint32_t len);
void eepromemu_flash_erase_sector(void* addr);
#ifdef __cplusplus
}
#endif
