/**
 * Controls our teensy 4.1 ethernet -> CAN board.
 */

#pragma once

#include "system.hpp"
#include "comms/can.hpp"
#include "comms/ethernet.hpp"
#include "logging.hpp"
#include "teensy_source_hash.hpp"

#define VERSION "0.0.1"

#define _MAX_LOOP_TIME 10000000

static bool init_success = false;

static inline void teensy_setup(uint32_t sleep_delay_ms) {
    BEGIN_SERIAL(115200);
    SLEEP_MS(sleep_delay_ms);
    feather_log("Starting teensy 4.1 ethernet -> CAN board version %s", VERSION);
    
    if (!initialize_can()) return;
    initialize_ethernet();
    feather_log("Teensy 4.1 ethernet -> CAN board initialized.");

    init_cycle_counter();  // Do last so we know there will be <7s before next update in teensy_loop()
    init_success = true;
}

#define LOOP_TIMES_SIZE 1000

static uint64_t loop_times[LOOP_TIMES_SIZE];
static uint64_t ethernet_times[LOOP_TIMES_SIZE];
static uint64_t can_times[LOOP_TIMES_SIZE];

static uint64_t max_loop_time = 0;
static uint64_t max_ethernet_time = 0;
static uint64_t max_can_time = 0;

static uint64_t last_loop_time = 0;
static uint64_t loop_index = 0;
static uint64_t loop_count = 0;

static bool print_stats = false;

static inline uint64_t get_max_time(uint64_t* times) {
    uint64_t max_time = 0;
    for (uint64_t i = 0; i < LOOP_TIMES_SIZE; i++) {
        if (times[i] > max_time) {
            max_time = times[i];
        }
    }
    return max_time;
}

static inline uint64_t get_min_time(uint64_t* times) {
    uint64_t min_time = UINT64_MAX;
    for (uint64_t i = 0; i < LOOP_TIMES_SIZE; i++) {
        if (times[i] < min_time) {
            min_time = times[i];
        }
    }
    return min_time;
}

static inline uint64_t get_avg_time(uint64_t* times) {
    uint64_t sum = 0;
    for (uint64_t i = 0; i < LOOP_TIMES_SIZE; i++) {
        sum += times[i];
    }
    return sum / LOOP_TIMES_SIZE;
}

static inline void teensy_loop(void) {
    if (!init_success) return;  // Initialization failed, so don't do anything.

    // Keep DWT wrap tracking current (~7s wrap at 600 MHz).
    (void)currtime_ns();

    uint64_t start_time = currtime_ns();
    loop_times[loop_index] = start_time - last_loop_time;
    if (loop_times[loop_index] > max_loop_time && loop_times[loop_index] < _MAX_LOOP_TIME) {
        max_loop_time = loop_times[loop_index];
    }
    last_loop_time = start_time;
    loop_count++;

    start_time = currtime_ns();
    poll_ethernet();
    ethernet_times[loop_index] = currtime_ns() - start_time;
    if (ethernet_times[loop_index] > max_ethernet_time && ethernet_times[loop_index] < _MAX_LOOP_TIME) {
        max_ethernet_time = ethernet_times[loop_index];
    }
    
    start_time = currtime_ns();
    POLL_CAN();
    can_times[loop_index] = currtime_ns() - start_time;
    if (can_times[loop_index] > max_can_time && can_times[loop_index] < _MAX_LOOP_TIME) {
        max_can_time = can_times[loop_index];
    }

    if (loop_count % 5000 == 0 && print_stats) {
        feather_always_log(
            "[Last] Loop: %llu ns, Ethernet: %llu ns, CAN: %llu ns",
            (unsigned long long)loop_times[loop_index],
            (unsigned long long)ethernet_times[loop_index],
            (unsigned long long)can_times[loop_index]
        );
        feather_always_log(
            "[Max] Loop: %llu ns, Ethernet: %llu ns, CAN: %llu ns",
            (unsigned long long)get_max_time(loop_times),
            (unsigned long long)get_max_time(ethernet_times),
            (unsigned long long)get_max_time(can_times)
        );
        feather_always_log(
            "[Min] Loop: %llu ns, Ethernet: %llu ns, CAN: %llu ns",
            (unsigned long long)get_min_time(loop_times),
            (unsigned long long)get_min_time(ethernet_times),
            (unsigned long long)get_min_time(can_times)
        );
        feather_always_log(
            "[Avg] Loop: %llu ns, Ethernet: %llu ns, CAN: %llu ns",
            (unsigned long long)get_avg_time(loop_times),
            (unsigned long long)get_avg_time(ethernet_times),
            (unsigned long long)get_avg_time(can_times)
        );
        feather_always_log(
            "[Absolute Max] Loop: %llu ns, Ethernet: %llu ns, CAN: %llu ns\n",
            (unsigned long long)max_loop_time,
            (unsigned long long)max_ethernet_time,
            (unsigned long long)max_can_time
        );
    }

    loop_index = (loop_index + 1) % LOOP_TIMES_SIZE;
}

/**
 * Shuts down all connections. Should only be used on the emulated version.
 */
static inline void shutdown(void) {

}
