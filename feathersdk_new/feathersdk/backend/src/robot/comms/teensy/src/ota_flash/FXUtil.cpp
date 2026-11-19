#include "FXUtil.hpp"
#include "../comms/comms.hpp"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../logging.hpp"


typedef struct {	// 
    char data[32] __attribute__((aligned(8)));
    unsigned int addr;	// address in intel hex record
    unsigned int code;	    // intel hex record type (0=data, etc.)
    unsigned int dlen;	// number of data bytes in intel hex record

    unsigned int base;	// base address to be added to intel hex 16-bit addr
    unsigned int min;		// min address in hex file
    unsigned int max;		// max address in hex file

    bool eof;		// set true on intel hex EOF (code = 1)
} hex_info_t;


#define MAX_HEXLINE_LENGTH (11 + 32 * 2)  // Max I ever saw was 16 dlen bytes, but double it just in case


/**
 * Parses a line of intel hex code into hex->dlen/addr/code/data.
 *
 * Intel Hex Record Format -
 *      :<byte_count(uint8_t)><address(uint16_t)><record type(uint8_t)><data(uint8_t[])><checksum(uint8_t)>
 * where `byte_count` is the number of bytes in the data field.
 *
 * Examples (spaces artificially added for readability)
 * :10 9D30 00 711F0000AD38000005390000F5460000 35
 * :04 9D40 00 01480000 D6
 * :00 0000 01 FF
 *
 * Each line is exactly 11 + byte_count*2 characters (plus terminating NUL).
 *
 * @return true if the line was valid, false if an error occurred.
 */
static bool parse_hex_line(const char* line, hex_info_t* hex) {
    // Make sure line/hex are valid and line starts with ':'
    ASSERT_LOCAL(line != NULL && hex != NULL && hex->data != NULL, false, "NULL argument");
    ASSERT_LOCAL(line[0] == ':', false, "Line must start with ':'");

    // Make sure line is within expected min/max lengths.
    const size_t len = strnlen(line, MAX_HEXLINE_LENGTH + 1);
    ASSERT_LOCAL(len >= 11, false, "Line too short");
    ASSERT_LOCAL(len <= MAX_HEXLINE_LENGTH, false, "Line too long (missing ending \\0?)");

    // Parse the byte count, address, and record type.
    ASSERT_LOCAL(sscanf(line + 1, "%2x", &hex->dlen) == 1, false, "Failed to parse byte count");
    ASSERT_LOCAL(hex->dlen <= 32, false, "dlen exceeds data buffer capacity");
    ASSERT_LOCAL(len == 11u + hex->dlen * 2u, false, "Line length does not match dlen");
    ASSERT_LOCAL(sscanf(line + 3, "%4x", &hex->addr) == 1, false, "Failed to parse address");
    ASSERT_LOCAL(sscanf(line + 7, "%2x", &hex->code) == 1, false, "Failed to parse record type");

    // Parse data bytes while calculating the checksum.
    unsigned int sum = (hex->dlen & 0xFF) + ((hex->addr >> 8) & 0xFF) + (hex->addr & 0xFF) + (hex->code & 0xFF);
    for (unsigned int i = 0; i < hex->dlen; i++) {
        unsigned int data_byte = 0;
        ASSERT_LOCAL(sscanf(line + 9 + (i * 2), "%2x", &data_byte) == 1, false, "Failed to parse data");
        hex->data[i] = (char)data_byte;
        sum += data_byte & 0xFF;
    }

    // Parse the checksum and verify it.
    unsigned int cksum = 0;
    ASSERT_LOCAL(sscanf(line + 9 + (hex->dlen * 2), "%2x", &cksum) == 1, false, "Failed to parse checksum");
    ASSERT_LOCAL((((sum & 0xFF) + (cksum & 0xFF)) & 0xFF) == 0, false, "Checksum error");
    return true;
}


static bool process_hex_record(hex_info_t* hex) {
    switch (hex->code) {
        case 0: // data -- update min/max address so far
            hex->max = MAX(hex->max, hex->base + hex->addr + hex->dlen);
            hex->min = MIN(hex->min, hex->base + hex->addr);
            break;
        case 1: hex->eof = true; break;  // EOF
        case 2: // extended segment address (top 16 of 24-bit addr)
            hex->base = (((uint32_t)(uint8_t)hex->data[0] << 8) | (uint8_t)hex->data[1]) << 4;
            break;
        case 4: // extended linear address (top 16 of 32-bit addr)
            hex->base = (((uint32_t)(uint8_t)hex->data[0] << 8) | (uint8_t)hex->data[1]) << 16;
            break;
        case 5: // start linear address (32-bit big endian addr)
            hex->base = ((uint32_t)(uint8_t)hex->data[0] << 24) | ((uint32_t)(uint8_t)hex->data[1] << 16)
                | ((uint32_t)(uint8_t)hex->data[2] << 8) | (uint8_t)hex->data[3];
            break;
        default: ASSERT_LOCAL(false, false, "Invalid hex record code: %u", hex->code);
    }
    return true;
}

// Reads a single line of ASCII characters from the EthernetClient into the line buffer. Reads until maxbytes is reached
// or until a newline or carriage return is encountered. Line will be null-terminated.
static void read_ascii_line(EthernetClient* client, uint8_t* line, int maxbytes) {
    int nchar = 0;
    while (client->connected() && nchar < maxbytes - 1) {
        if (client->read(&line[nchar], 1) <= 0) continue;

        if (line[nchar] == '\n' || line[nchar] == '\r') {
            if (nchar == 0) continue;  // skip leading newlines
            break;                     // end of line
        }
        nchar++;
    }
    line[nchar] = 0;  // null-terminate
}

static hex_info_t hex = {0};

//******************************************************************************
// update_firmware()	read hex file and write new firmware to program flash
//******************************************************************************
bool prepare_teensy_flash(EthernetClient* client, flash_elem_t* buf_addr, size_t buf_size) {
  static uint8_t line[96];					// buffer for hex lines

  // Initialize hex info struct. Reset to all 0's, except for min which is initialized to max value to find min later.
  memset(&hex, 0, sizeof(hex));
  hex.min = 0xFFFFFFFF;

  // read and process intel hex lines until EOF or error
  while (!hex.eof) {
    ASSERT_LOCAL(client->connected(), false, "Client disconnected");
    read_ascii_line(client, line, sizeof(line));

    // Skip malformed/empty lines rather than aborting the whole transfer.
    if (!parse_hex_line((const char*)line, &hex)) continue;
    if (!process_hex_record(&hex)) return false;

    if (hex.code == 0) { // data record
      // Hex addresses are absolute flash addresses; relocate into the staging buffer.
      const uintptr_t offset = (uintptr_t)hex.base + hex.addr - FLASH_BASE_ADDR;
      flash_elem_t* addr = (flash_elem_t*)((uintptr_t)buf_addr + offset);

      ASSERT_LOCAL(hex.max <= FLASH_BASE_ADDR + buf_size, false, "Hex file exceeds buffer size");
      ASSERT_LOCAL(flash_write_block(addr, hex.data, hex.dlen) == 0, false, "Failed to write hex data to buffer");
    }
  }

  // check FLASH_ID in new code - abort if not found
  ASSERT_LOCAL(check_flash_id(buf_addr, hex.max - hex.min), false, "FLASH_ID not found in hex file");

  return true;
}

void flash_noreturn(flash_elem_t* buf_addr) {
    flash_move(FLASH_BASE_ADDR, (uintptr_t)buf_addr, hex.max - hex.min);
}