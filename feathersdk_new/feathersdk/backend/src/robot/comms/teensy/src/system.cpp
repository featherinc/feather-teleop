#include "system.hpp"
#include "comms/ethernet.hpp"
#include <stdint.h>

runtime_static int64_t timestamp_offset_ns = 0;

#ifdef COMPILE_FOR_TEENSY

static uint32_t last_cyccnt = 0;
static uint64_t cyccnt_hi = 0;  // Upper 32 bits of the extended cycle count

void init_cycle_counter(void) {
    // Enable the trace block, then the DWT cycle counter.
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
    last_cyccnt = ARM_DWT_CYCCNT;
    cyccnt_hi = 0;
}

static uint64_t cycles64(void) {
    uint32_t cyccnt = ARM_DWT_CYCCNT;
    if (cyccnt < last_cyccnt) {
        cyccnt_hi += 1ULL << 32;
    }
    last_cyccnt = cyccnt;
    return cyccnt_hi + (uint64_t)cyccnt;
}

static uint64_t raw_currtime_ns(void) {
    uint64_t cycles = cycles64();
    // Prefer the runtime CPU clock (Teensy may change F_CPU_ACTUAL).
    uint32_t cpu_hz = F_CPU_ACTUAL;
    // Split division avoids overflow for large cycle counts:
    //   ns = cycles * 1e9 / cpu_hz
    return (cycles / cpu_hz) * 1000000000ULL + ((cycles % cpu_hz) * 1000000000ULL) / cpu_hz;
}

#else

void init_cycle_counter(void) {}

runtime_static uint64_t raw_currtime_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

#endif


uint64_t currtime_ns(void) {
    return (uint64_t)((int64_t)raw_currtime_ns() + timestamp_offset_ns);
}

bool set_timestamp_offset_ns(int64_t offset) {
    if (offset >= ((int64_t)1 << 60)) {
        set_local_error_msg("Timestamp offset out of range (must be < 2^60): %lld", (long long)offset);
        return false;
    }
    if (offset < 0) {
        uint64_t magnitude = (uint64_t)(-offset);
        uint64_t now = raw_currtime_ns();
        if (magnitude >= now) {
            set_local_error_msg(
                "Negative timestamp offset magnitude (%llu) must be < raw board time (%llu)",
                (uint64_t)magnitude,
                (uint64_t)now
            );
            return false;
        }
    }
    timestamp_offset_ns = offset;
    return true;
}

bool handle_get_board_time(uint32_t msg_id, void* payload) {
    (void)payload;
    GetBoardTimeResponsePayload response = {.board_time_nanos = currtime_ns()};
    eth_buffer_write(msg_id, TEENSY_COMMAND_OPCODE_RESP_BOARD_TIME, &response, sizeof(GetBoardTimeResponsePayload));
    return true;
}

bool handle_set_timestamp_offset(uint32_t msg_id, void* payload) {
    (void)msg_id;
    SetTimestampOffsetPayload* set_offset_payload = (SetTimestampOffsetPayload*)payload;
    (void)set_timestamp_offset_ns(set_offset_payload->offset);
    return false;  // Caller sends success or error based on local error message
}
