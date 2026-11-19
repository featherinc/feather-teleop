#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum FLEXCAN_INTERFACE {
    CAN0 = 0,
    CAN1 = 1,
    CAN2 = 2,
    CAN3 = 3,
} FLEXCAN_INTERFACE;

typedef enum FLEXCAN_RXQUEUE_TABLE {
    RX_SIZE_2 = 2,
    RX_SIZE_4 = 4,
    RX_SIZE_8 = 8,
    RX_SIZE_16 = 16,
    RX_SIZE_32 = 32,
    RX_SIZE_64 = 64,
    RX_SIZE_128 = 128,
    RX_SIZE_256 = 256,
    RX_SIZE_512 = 512,
    RX_SIZE_1024 = 1024,
} FLEXCAN_RXQUEUE_TABLE;

typedef enum FLEXCAN_TXQUEUE_TABLE {
    TX_SIZE_2 = 2,
    TX_SIZE_4 = 4,
    TX_SIZE_8 = 8,
    TX_SIZE_16 = 16,
    TX_SIZE_32 = 32,
    TX_SIZE_64 = 64,
    TX_SIZE_128 = 128,
    TX_SIZE_256 = 256,
    TX_SIZE_512 = 512,
    TX_SIZE_1024 = 1024,
} FLEXCAN_TXQUEUE_TABLE;

typedef struct CAN_message_t {
    uint32_t id;
    uint16_t timestamp;
    uint8_t idhit;
    struct {
        bool extended;
        bool remote;
        bool overrun;
        bool reserved;
    } flags;
    uint8_t len;
    uint8_t buf[8];
    int8_t mb;
    uint8_t bus;
    bool seq;
} CAN_message_t;


typedef struct EmulatedFlexCAN {
    uint8_t bus_index;
    int fd;
    uint32_t baud;
} EmulatedFlexCAN;

// Initializes the CAN busses.
bool initialize_can(void);
// Returns the number of CAN busses available.
uint8_t get_num_can_lines(void);
// Writes a CAN message to the given bus index. Returns >0 on success, 0 on failure/invalid bus.
int write_physical_can(uint8_t bus, const CAN_message_t* msg);
// Reads a CAN message from the given bus index and stores it in the given message.
int read_physical_can(uint8_t bus, CAN_message_t* msg);

#define MAX_NUM_CAN_LINES 8  // Currently 8 since that's the largest number we can store in candump telemetry info bits