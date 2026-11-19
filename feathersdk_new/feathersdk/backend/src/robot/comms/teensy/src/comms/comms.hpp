/**
 * Handles the communication between the teensy and the jetson, and teensy and CAN lines.
 */

#pragma once

#include "../system.hpp"
#include "../logging.hpp"
#include <stdint.h>
#include "../actuator.hpp"

// Need to define this here so it can be used in either of the compile branches below
#define IP_PORT 6613


/**
 * Sets the communication configuration.
 * 
 * @param comms_config The communication configuration to set.
 */
void set_comms_config(const CommsConfig* comms_config);

extern CommsConfig comms_config;


#ifdef COMPILE_FOR_TEENSY

    #define BUILD_IP_ADDR(addr) IPAddress((addr >> 24) & 0xFF, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF)

    using namespace qindesign::network;
    // Defined once in ethernet.cpp — do not make these static in this header.
    extern EthernetServer ethernet_server;
    extern EthernetClient ethernet_client;

    #define ETHERNET_CLIENT_CONNECTED (ethernet_client)
    #define ETHERNET_CLIENT_AVAILABLE (ethernet_client = ethernet_server.available())

#else

    #include "../../emulator/em_ethernet.hpp"
    #define BUILD_IP_ADDR(_addr_int) (IPAddress){.addr = (_addr_int)}

#endif