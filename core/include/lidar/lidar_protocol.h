#include <cstdint>
#pragma one


#define YD_PACKMAXNODES 80 
#define YD_PACKHEADSIZE 10
#define MAX_SCANNODES 5000

#define YDLIDAR_START_BIT             0xA5

#define YDLIDAR_START_SCAN            0x60
#define YDLIDAR_STOP_SCAN             0x65
#define YDLIDAR_GET_INFO              0x90
#define YDLIDAR_GET_HEALTH            0x92
#define YDLIDAR_FREQ_UP_0_1HZ         0x09
#define YDLIDAR_FREQ_DOWN_0_1HZ       0x0A
#define YDLIDAR_FREQ_UP_1HZ           0x0B
#define YDLIDAR_FREQ_DOWN_1HZ         0x0C
#define YDLIDAR_GET_FREQ              0x0D
#define YDLIDAR_SOFT_RESTART          0x40

struct node_info
{
    uint16_t intensity;
    uint16_t angle;
    uint16_t dist;
    uint8_t scan_freq;
    uint8_t index;
};


struct node_pakcage {
    uint16_t    header;
    uint8_t     ct;
    uint8_t     lsn;
    uint16_t    fsa;
    uint16_t    lsa;
    uint16_t    cs;
    uint16_t    nodes[YD_PACKMAXNODES];
} __attribute__((packed));

struct cmd_packet {
    uint8_t syncByte;
    uint8_t cmd_flag;
    uint8_t size;
    uint8_t data;
} __attribute__((packed));