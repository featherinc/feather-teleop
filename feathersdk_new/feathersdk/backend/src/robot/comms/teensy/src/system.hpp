#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "logging.hpp"
#include "qdefs/data_type_qdef.hpp"
#include "qdefs/teensy_commands_qdef.hpp"

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
    #error "protocol assumes little-endian host"
#endif

/////////////
// Configs //
/////////////

// The maximum number of motors that can be supported. Thus maximum valid motor ID is (MAX_NUM_MOTORS - 1).
#define MAX_NUM_MOTORS 0x80

// X-macro list of all data types and their corresponding C types.
#define DTYPE_X_LIST \
    X(DATA_TYPE_U8,   uint8_t ) \
    X(DATA_TYPE_U16,  uint16_t) \
    X(DATA_TYPE_U32,  uint32_t) \
    X(DATA_TYPE_U64,  uint64_t) \
    X(DATA_TYPE_I8,   int8_t  ) \
    X(DATA_TYPE_I16,  int16_t ) \
    X(DATA_TYPE_I32,  int32_t ) \
    X(DATA_TYPE_I64,  int64_t ) \
    X(DATA_TYPE_F32,  float   ) \
    X(DATA_TYPE_F64,  double  ) \
    X(DATA_TYPE_BOOL, bool   )

// An exact copy of DTYPE_X_LIST for nested X-macros. The dest C type is passed through so inner
// X2 expansions can use it — outer X parameters are not visible after X2 is expanded.
#define DTYPE_X_LIST2(__dst_ctype) \
    X2(DATA_TYPE_U8,   uint8_t , __dst_ctype) \
    X2(DATA_TYPE_U16,  uint16_t, __dst_ctype) \
    X2(DATA_TYPE_U32,  uint32_t, __dst_ctype) \
    X2(DATA_TYPE_U64,  uint64_t, __dst_ctype) \
    X2(DATA_TYPE_I8,   int8_t  , __dst_ctype) \
    X2(DATA_TYPE_I16,  int16_t , __dst_ctype) \
    X2(DATA_TYPE_I32,  int32_t , __dst_ctype) \
    X2(DATA_TYPE_I64,  int64_t , __dst_ctype) \
    X2(DATA_TYPE_F32,  float   , __dst_ctype) \
    X2(DATA_TYPE_F64,  double  , __dst_ctype) \
    X2(DATA_TYPE_BOOL, bool    , __dst_ctype)

// Assert that DTYPE_X_LIST and DTYPE_X_LIST2 are the same (order, names, and C types).
#define X(__dtype, __ctype) #__dtype "," #__ctype ";"
#define X2(__dtype, __ctype, __dst_ctype) #__dtype "," #__ctype ";"
#if defined(__cplusplus)
static_assert(__builtin_strcmp("" DTYPE_X_LIST, "" DTYPE_X_LIST2(_)) == 0,
    "DTYPE_X_LIST and DTYPE_X_LIST2 must be identical");
#else
_Static_assert(__builtin_strcmp("" DTYPE_X_LIST, "" DTYPE_X_LIST2(_)) == 0,
    "DTYPE_X_LIST and DTYPE_X_LIST2 must be identical");
#endif
#undef X
#undef X2

/**
 * Casts a value from one data type to another. Does no safety checking (overflow, float->int truncation, etc.).
 * 
 * It is assumed that the source/destination endianness is the same, and matches system endianness.
 *
 * @param dst The destination pointer to write the cast value to.
 * @param dst_dtype The data type of the destination.
 * @param src The source pointer to read the value from.
 * @param src_dtype The data type of the source.
 * @param scope The scope to set the error message on.
 * @return True if the cast was successful, false otherwise.
 */
 static inline bool dtype_cast(void* dst, DataType dst_dtype, const void* src, DataType src_dtype, ErrorScope scope) {
    // Widen to int64/double, then narrow. Two sequential DTYPE_X_LIST switches — not nested —
    // so the preprocessor can expand the same list twice (no paint-blue).
    uint8_t value_store[8] __attribute__((aligned(8)));
    memcpy(value_store, src, dtype_size(src_dtype));

    // Some macro magic to cartesion-product all possible src/dest dtypes/ctypes to convert nicely
    #define X2(__src_dtype, __src_ctype, __dst_ctype)                       \
        case __src_dtype: {                                                 \
            *(__dst_ctype*)dst = (__dst_ctype)(*(__src_ctype*)value_store); \
            break;                                                          \
        }

    #define X(__dst_dtype, __dst_ctype)                                                     \
        case __dst_dtype: {                                                                 \
            switch (src_dtype) {                                                            \
                DTYPE_X_LIST2(__dst_ctype)                                                  \
                default:                                                                    \
                    set_error_msg(scope, "Unknown src_dtype %u for dtype_cast", src_dtype); \
                    return false;                                                           \
            }                                                                               \
            break;                                                                          \
        }

    switch (dst_dtype) {
        DTYPE_X_LIST
        default:
            set_error_msg(scope, "Unknown dst_dtype %u for dtype_cast", dst_dtype);
            return false;
    }

    #undef X
    #undef X2

    return true;
}


///////////
// Utils //
///////////

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

/**
 * Initializes the DWT cycle counter. Must be called before currtime_ns() on hardware.
 */
void init_cycle_counter(void);

/**
 * Returns nanoseconds since DWT cycle-counter enable (hardware) or an arbitrary
 * monotonic epoch (emulation), plus any timestamp offset set via
 * set_timestamp_offset_ns(). Call regularly (e.g. from the main loop) so the
 * 32-bit CYCCNT wrap (~7s at 600 MHz) is detected.
 */
uint64_t currtime_ns(void);

/**
 * Sets the offset added to every currtime_ns() / board-time reading.
 *
 * @param offset Nanoseconds to add (may be negative). Must be < 2^60; if negative,
 *     its magnitude must be strictly less than the current raw board time.
 * @return True if the offset was applied, false if invalid (local error set).
 */
bool set_timestamp_offset_ns(int64_t offset);

bool handle_get_board_time(uint32_t msg_id, void* payload);
bool handle_set_timestamp_offset(uint32_t msg_id, void* payload);


//////////////////////
// System Emulation //
//////////////////////


#ifdef TESTING_TEENSY
#define runtime_static
#else
#define runtime_static \
    /* Define something as static during runtime, but not during testing */ \
    static
#endif

/**
 * We can emulate the system for testing/IDE static analysis purposes. To compile for the actual hardware, 
 * just define the COMPILE_FOR_TEENSY macro when compiling.
 */

#ifdef COMPILE_FOR_TEENSY

    #include <Arduino.h>
    #define PRINT_STRBUF(buffer) Serial.println(buffer)
    #define BEGIN_SERIAL(baud) Serial.begin(baud)
    #define SLEEP_MS(ms) delay(ms)

    #include <QNEthernet.h>
    #include <FlexCAN_T4.h>
    #include "ota_flash/FlashTxx.hpp"
    #include "ota_flash/FXUtil.hpp"

#else

    #define BEGIN_SERIAL(baud)

    #include <stdio.h>
    #include <time.h>

    #define PRINT_STRBUF(buffer) printf("%s\n", buffer); fflush(stdout);

    #define SLEEP_MS(ms) \
        struct timespec __ts; \
        __ts.tv_sec = (time_t)((ms) / 1000u); \
        __ts.tv_nsec = (long)((ms) % 1000u) * 1000000L; \
        nanosleep(&__ts, NULL); \

    // Emulated eeprom writing and erasing
    #include "../emulator/em_eeprom.hpp"

#endif


// Need to do this to fix a compiler warning since on some systems, EAGAIN == EWOULDBLOCK.
#if EAGAIN != EWOULDBLOCK
    #define ERRNO_IS_EAGAIN_OR_EWOULDBLOCK(e) ((e) == EAGAIN || (e) == EWOULDBLOCK)
#else
    #define ERRNO_IS_EAGAIN_OR_EWOULDBLOCK(e) ((e) == EAGAIN)
#endif
