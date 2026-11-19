#pragma once

/**
 * Drop-in stand-ins for the QNEthernet types used by this firmware.
 * Backed by a normal TCP listen socket so you can talk to the firmware
 * process from the host (e.g. `nc 127.0.0.1 6613`).
 *
 * Supported surface (matches usage in comms.cpp):
 *   IPAddress helpers,
 *   ethernet_set_mac_address / ethernet_begin / ethernet_local_ip / ethernet_link_state,
 *   eth_server_begin / eth_server_available,
 *   eth_client_* (connected, available, read, write, flush, setNoDelay, stop, remote ip/port)
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct IPAddress {
    uint32_t addr;  /* network byte order */
} IPAddress;

// Global EthernetClient instance
typedef struct EthernetClient {
    int fd;
    IPAddress remote_ip;
    uint16_t remote_port;

    bool (*connected)(void);
    int (*available)(void);
    int (*availableForWrite)(void);
    int (*read)(uint8_t* buf, size_t size);
    int (*write)(const uint8_t* buf, size_t size);
    void (*flush)(void);
    bool (*setNoDelay)(bool flag);
    void (*stop)(void);
} EthernetClient;

extern EthernetClient ethernet_client;

// Global EthernetServer instance
typedef struct EthernetServer {
    uint16_t port;
    int listen_fd;

    void (*begin)(void);
    void (*close)(void);
    bool (*available)(EthernetClient* out_client);
} EthernetServer;

extern EthernetServer ethernet_server;

// Global Ethernet class instance
typedef struct EthernetClass {
    IPAddress _local_ip;
    uint8_t _mac[6];
    bool _link_up;

    void (*setMACAddress)(const uint8_t mac[6]);
    bool (*begin)(IPAddress ip, IPAddress subnet, IPAddress gateway);
    bool (*linkState)(void);
} EthernetClass;

extern EthernetClass Ethernet;

// Extra needed defines for some C++ syntax.
// Match Teensy semantics: CONNECTED means we hold a client fd; use connected() for peer-alive checks.
#define ETHERNET_CLIENT_CONNECTED (ethernet_client.fd >= 0)
#define ETHERNET_CLIENT_AVAILABLE (ethernet_server.available(&ethernet_client))
