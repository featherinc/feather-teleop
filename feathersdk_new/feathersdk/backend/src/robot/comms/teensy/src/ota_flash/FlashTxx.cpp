#ifndef _GNU_SOURCE
#define _GNU_SOURCE     // for memmem(), from string.h
#endif

#include <malloc.h>
#include <string.h>
#include <stdbool.h>
#include "FlashTxx.hpp"

static int leave_interrupts_disabled = 0;

void firmware_buffer_init(flash_elem_t** buf_addr_store, size_t* buf_size_store) {
  // Start at the last flash_elem_t below FLASH_RESERVE and walk down until
  // non-erased flash is found (top of the currently programmed firmware).
  flash_elem_t* p = (flash_elem_t*)(uintptr_t)(FLASH_BASE_ADDR + FLASH_SIZE - FLASH_RESERVE) - 1;
  while ((uintptr_t)p > 0 && *p == (flash_elem_t)0xFFFFFFFF) p--;
  p++; // first element above code

  // Round up to the next sector boundary if not already aligned.
  uintptr_t addr = (uintptr_t)p;
  if ((addr % FLASH_SECTOR_SIZE) > 0) addr += FLASH_SECTOR_SIZE - (addr % FLASH_SECTOR_SIZE);
  p = (flash_elem_t*)addr;

  *buf_addr_store = p;
  *buf_size_store = (size_t)(FLASH_BASE_ADDR + FLASH_SIZE - FLASH_RESERVE - (uintptr_t)p);
}

void firmware_buffer_free(flash_elem_t* buf_addr, size_t buf_size) {
  uintptr_t address = (uintptr_t)buf_addr;
  const uintptr_t end = address + buf_size;
  while (address < end) {
    if ((address & (FLASH_SECTOR_SIZE - 1)) == 0) {
      if (flash_sector_not_erased(address)) {
        eepromemu_flash_erase_sector((void*)address);
      }
    }
    address += FLASH_SECTOR_SIZE;
  }
}

bool check_flash_id(flash_elem_t* buf_addr, size_t buf_size) {
  return memmem((const void*)buf_addr, buf_size, FLASH_ID, strlen(FLASH_ID)) != NULL;
}

//******************************************************************************
// flash_sector_not_erased()	returns 0 if erased and !0 (error) if NOT erased
//******************************************************************************
RAMFUNC int flash_sector_not_erased(uintptr_t address) {
  flash_elem_t* sector = (flash_elem_t*)(address & ~(FLASH_SECTOR_SIZE - 1u));
  for (uint32_t i = 0; i < FLASH_SECTOR_SIZE / FLASH_WRITE_SIZE; i++) {
    if (*sector++ != (flash_elem_t)0xFFFFFFFF)
      return 1; // NOT erased
  }
  return 0; // erased
}

//******************************************************************************
// move from source to destination (flash), erasing destination sectors as we go
// DANGER: this is critical and cannot be interrupted, else T3.x can be damaged
//******************************************************************************
RAMFUNC void flash_move(uintptr_t dst, uintptr_t src, uint32_t size) {
  uint32_t offset=0, error=0;
  uintptr_t addr;
  
  // set global flag leave_interrupts_disabled = 1 to prevent the T3.x flash
  // write and erase functions from re-enabling interrupts when they complete 
  leave_interrupts_disabled = 1;
  
  // move size bytes containing new program from source to destination
  while (offset < size && error == 0) {

    addr = dst + offset;

    // if new sector, erase, then immediately write FSEC/FOPT if in this sector
    // this is the ONLY place that FSEC values are written, so it's the only
    // place where calls to KINETIS flash write functions have aFSEC = oFSEC = 1
    if ((addr & (FLASH_SECTOR_SIZE - 1)) == 0) {
      if (flash_sector_not_erased(addr)) {
        eepromemu_flash_erase_sector((void*)addr);
      }
    }
    
    // for T4.x, data address passed to flash_write() must be in RAM
    uint32_t value = *(uint32_t*)(src + offset);
    eepromemu_flash_write((void*)addr, &value, sizeof(uint32_t));

    offset += FLASH_WRITE_SIZE;
  }
  
  // move is complete. if the source buffer (src) is in FLASH, erase the buffer
  // by erasing all sectors from top of new program to bottom of FLASH_RESERVE,
  // which leaves FLASH in same state as if code was loaded using TeensyDuino.
  // For KINETIS, this erase cannot include FSEC, so erase uses aFSEC=0.
  if (IN_FLASH(src)) {
    while (offset < (FLASH_SIZE - FLASH_RESERVE) && error == 0) {
      addr = dst + offset;
      if ((addr & (FLASH_SECTOR_SIZE - 1)) == 0) {
        if (flash_sector_not_erased(addr)) {
          eepromemu_flash_erase_sector((void*)addr);
	    }
      }
      offset += FLASH_WRITE_SIZE;
    }   
  }

  // for T3.x, at least, must REBOOT here (via macro) because original code has
  // been erased and overwritten, so return address is no longer valid
  REBOOT;
  // wait here until REBOOT actually happens 
  for (;;) {}
}

//******************************************************************************
// Write count bytes from data[] into erased flash at addr.
// addr and count must be FLASH_WRITE_SIZE-aligned. May buffer a partial
// flash_elem_t across calls; successive calls must be contiguous.
//******************************************************************************
int flash_write_block(flash_elem_t* addr, char* data, size_t count) {
  static flash_elem_t buf __attribute__((aligned(sizeof(flash_elem_t))));
  static size_t buf_count = 0;           // bytes currently in buf
  static flash_elem_t* next_addr = NULL; // expected address of next call

  if (((uintptr_t)addr % FLASH_WRITE_SIZE) != 0 || (count % FLASH_WRITE_SIZE) != 0)
    return 1; // alignment error

  if (buf_count > 0 && addr != next_addr)
    return 2; // unexpected (non-contiguous) address

  next_addr = (flash_elem_t*)((uintptr_t)addr + count);
  // Rewind to the start of any partially filled flash_elem_t from a prior call.
  addr = (flash_elem_t*)((uintptr_t)addr - buf_count);

  size_t data_i = 0;
  while (data_i < count) {
    ((char*)&buf)[buf_count++] = data[data_i++];
    if (buf_count < FLASH_WRITE_SIZE)
      continue;
    eepromemu_flash_write((void*)addr, (void*)&buf, (uint32_t)FLASH_WRITE_SIZE);
    buf_count = 0;
    addr++;
  }
  return 0;
}


