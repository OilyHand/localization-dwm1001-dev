#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include <string.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi.h"
#include "port.h"
#include "ble_device.h"

#define LOG_LEVEL 3
#include <logging/log.h>
LOG_MODULE_REGISTER(main);

// Device information
#define APP_HEADER  "*** DWM1001-Ranging Project ***\n"
#define DEV_TYPE    "Anchor\n"
#define APP_LINE    "===============================\n"


/**
 * Definitions for Time of Flight (ToF) calculations and ranging parameters.
 */

// Default antenna delay values for 64 MHz PRF
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

// UWB microsecond (uus) to device time unit (dtu, around 15.65 ps)
// 1 uus = 512 / 499.2 usec and 1 usec = 499.2 * 128 dtu.
#define UUS_TO_DWT_TIME 65536

// Delay definitions
#define POLL_RX_TO_RESP_TX_DLY_UUS 6000
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
#define FINAL_RX_TIMEOUT_UUS 65000 // final message receive timeout
#define PRE_TIMEOUT 30 // preamble timeout

// Speed of light in air (in mm/us)
#define SPEED_OF_LIGHT 299702547

// Data types for timestamps (need 40 bits)
typedef signed long long   int64;
typedef unsigned long long uint64;


// Default communication configuration
dwt_config_t config = {
    5,               // Channel number
    DWT_PRF_64M,     // Pulse Repetition Frequency
    DWT_PLEN_128,    // Preamble Length
    DWT_PAC8,        // Preamble Accumulator
    9,               // TX Code
    9,               // RX Code
    1,               // SFD Timeout
    DWT_BR_6M8,      // Data Rate
    DWT_PHRMODE_STD, // PHR Mode
    129              // SFD timeout
                     //(preamble length + 1 + SFD length - PAC size).
};

// Device IDs (Used for skipping discovery phase)
#define DEVICE_TAG_ID 0xAA01
#ifdef DEVICE_ANCHOR_01
#define DEVICE_ANC_ID 0xBB01
#elif DEVICE_ANCHOR_02
#define DEVICE_ANC_ID 0xBB02
#elif DEVICE_ANCHOR_03
#define DEVICE_ANC_ID 0xBB03
#elif DEVICE_ANCHOR_04
#define DEVICE_ANC_ID 0xBB04
#else
#define DEVICE_ANC_ID 0xBB01 // Default ID
#endif


/**
 * Definitions for MAC frame
 */

// MAC Header Frame Control
#define MHR_FC_TYPE_DATA 0x8841

// Message types
#define MSG_TYPE_POLL 0x61
#define MSG_TYPE_RESP 0x50
#define MSG_TYPE_FINL 0x69
#define MSG_TYPE_REPO 0x51

#define FRAME_FC_IDX 0
#define FRAME_SN_IDX 2
#define FRAME_PANID_IDX 3
#define FRAME_MSG_DSTADR_IDX 5
#define FRAME_MSG_SRCADR_IDX 7
#define FRAME_MSG_FNC_IDX 9
#define FRAME_MSG_FINAL_POLL_TO_RESP_IDX 10

#define FRAME_INIT_LEN 14
#define FRAME_BLNK_LEN 10
#define FRAME_MSG_POLL_LEN 10
#define FRAME_MSG_RESP_LEN 10
#define FRAME_MSG_FINAL_LEN 18
#define FRAME_MSG_REPORT_LEN 14

// Length of the common part of the message
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define POLL_DEVICE_ID_IDX 3
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4


#define RNGST_POLL 0x00
#define RNGST_FINAL 0x01

/**
 * Ranging Message Frames
 */

static uint8 anchor_resp_msg[] = {
/* Data Frame MAC Header */
    // Frame Control
    0x41, 0x8C,
    // Sequence Number (SN)
    0x00,
    // PAN ID
    0xCA, 0xDE,
    // Dest. Address (Tag Short Address)
    0x01, 0xAA,
    // Source Address
    0x00, 0xBB,
/* Data Frame MAC Payload */
    // Function Code (0x50 for resp msg)
    0x50,
    // For FCS
    0x00,0x00
};

// static uint8 anchor_report_msg[] = {
// /* Data Frame MAC Header */
//     // Frame Control
//     0x41, 0x8C,
//     // Sequence Number (SN)
//     0x00,
//     // PAN ID
//     0xCA, 0xDE,
//     // Dest. Address (Tag Short Address)
//     0x01, 0xAA,
//     // Source Address
//     0x00, 0xBB,
// /* Data Frame MAC Payload */
//     // Function Code (0x51 for report msg)
//     0x51,
//     // Calculated Time-of-Flight (ToF)
//     0x00, 0x00, 0x00, 0x00,
//     // For FCS
//     0x00, 0x00
// };

// Frame sequence number
static uint8 frame_seq_nb = 0;
static uint8 frame_seq_nb_rx = 0;

// Buffer to store received messages
#define RX_BUF_LEN 24
static uint8 rx_buffer[RX_BUF_LEN];
static uint32 status_reg = 0;

// timestamp variables for ds_twr_resp device
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;

// distance calculation variables
static double tof;
static double distance;
char dist_str[16] = {0}; // for displaying distance in console


static inline uint64 get_tx_timestamp_u64(void) {
    uint8 ts_tab[5];
    uint64 ts = 0;
    dwt_readtxtimestamp(ts_tab);
    for (int i = 4; i >= 0; i--) {
        ts = (ts << 8) | ts_tab[i];
    }
    return ts;
}

static inline uint64 get_rx_timestamp_u64(void) {
    uint8 ts_tab[5];
    uint64 ts = 0;
    dwt_readrxtimestamp(ts_tab);
    for (int i = 4; i >= 0; i--) {
        ts = (ts << 8) | ts_tab[i];
    }
    return ts;
}

static inline void final_msg_get_ts(const uint8 * ts_field, uint32 * ts) {
    *ts = 0;
    for (int i = 0; i < FINAL_MSG_TS_LEN; i++) {
        *ts += ts_field[i] << (i * 8);
    }
}

////////////////////////////////////////////////////////////////////////////////

// Main application entry point
int dw_main(void) {
    // Display device information
    printk("\n");
    printk(APP_LINE);
    printk(APP_HEADER);
    printk("Device Type: ");
    printk(DEV_TYPE);
    printk("Device ID: 0x%04X\n", DEVICE_ANC_ID);
    printk(APP_LINE);

    // Initialize the DW1000
    openspi();
    reset_DW1000();
    port_set_dw1000_slowrate();
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        printk("<ranging> DW1000 initialization failed!\n");
        k_sleep(K_MSEC(500));
        while (1)
        { };
    }
    port_set_dw1000_fastrate();

    dwt_configure(&config);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setleds(1);

    k_yield();

    // Initialize BLE Buffer
    ble_reps_t * ble_reps;
    uint8_t ble_buf[120] = {0};
    ble_reps = (ble_reps_t *)(&ble_buf[0]);

    // Initialize ranging state
    uint8 ranging_state = RNGST_POLL;

    k_yield();

    while (1) {
        k_yield();
        dwt_setrxtimeout(0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        // Wait for POLL message
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
            (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        { };

        if (status_reg & SYS_STATUS_RXFCG) {
            uint32 frame_len;

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= RX_BUF_LEN) {
                dwt_readrxdata(rx_buffer, frame_len, 0);
#ifdef DEBUG
                printk("<ranging> Received frame: ");
                for(int i = 0; i < frame_len; i++) {
                    printk("%02X ", rx_buffer[i]);
                } printk("\n");
#endif
            }

            dwt_forcetrxoff();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);

            // FC Check
            uint16 mhr_fc = rx_buffer[0] | (rx_buffer[1] << 8);
            if(mhr_fc != MHR_FC_TYPE_DATA) {
                printk("<ranging> Invalid Frame Control\n");
                continue;
            }

            frame_seq_nb = rx_buffer[ALL_MSG_SN_IDX];
            uint16 mhr_dstadr = rx_buffer[5] | (rx_buffer[6] << 8);
            uint16 mhr_srcadr = rx_buffer[7] | (rx_buffer[8] << 8);
            uint16 func_code = rx_buffer[9];

#ifdef DEBUG
            printk("------------------------------------------------------------\n");
            printk("Received Frame: ");
            for(int i = 0; i < frame_len; i++) printk("%02X ", rx_buffer[i]);
            printk("\n");
            printk(" - Sequence Number: %d\n", frame_seq_nb);
            printk(" - Destination Address: 0x%04X\n", mhr_dstadr);
            printk(" - Source Address: 0x%04X\n", mhr_srcadr);
            printk(" - Function Code: 0x%02X\n", func_code);
            printk("------------------------------------------------------------\n");
#endif

            if(mhr_dstadr != DEVICE_ANC_ID) {
            #ifdef DEBUG
                printk("<ranging> ADDR mismatch\n");
            #endif
                continue;
            }

            if (ranging_state == RNGST_POLL) {
                if(func_code != MSG_TYPE_POLL) {
                    printk("<ranging> not a POLL frame\n");
                    continue;
                }

                anchor_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

            #ifdef DEBUG
                printk("<ranging> Received POLL: ");
                for(int i = 0; i < frame_len; i++) {
                    printk("%02X ", rx_buffer[i]);
                } printk("\n");

                printk("<ranging> Sended RESP: ");
                for(int i=0; i < sizeof(anchor_resp_msg); i++) {
                    printk("%02X ", anchor_resp_msg[i]);
                } printk("\n");
            #endif

                poll_rx_ts = get_rx_timestamp_u64();
                uint32 resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;

                dwt_setdelayedtrxtime(resp_tx_time);
                dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
                dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

                dwt_writetxdata(sizeof(anchor_resp_msg), anchor_resp_msg, 0);
                dwt_writetxfctrl(sizeof(anchor_resp_msg), 0, 1);
                if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) == DWT_ERROR){
                    printk("<ranging> Error starting transmission\n");
                    continue;
                }

                ranging_state = RNGST_FINAL;
                printk("<ranging> RESP sent, waiting for FINAL\n");
            } else if (ranging_state == RNGST_FINAL){
                // FINAL MESSAGE RECEIVE
                if(func_code != MSG_TYPE_FINL){
                    continue;
                }
            #ifdef DEBUG
                printk("<ranging> Received FINAL: ");
                for(int i = 0; i < frame_len; i++) {
                    printk("%02X ", rx_buffer[i]);
                } printk("\n");
            #endif

                uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
                final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                resp_tx_ts = get_tx_timestamp_u64();
                final_rx_ts = get_rx_timestamp_u64();

                uint32 poll_rx_ts_32 = (uint32)poll_rx_ts;
                uint32 resp_tx_ts_32 = (uint32)resp_tx_ts;
                uint32 final_rx_ts_32 = (uint32)final_rx_ts;

                double Ra = (double)(resp_rx_ts - poll_tx_ts);
                double Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                double Da = (double)(final_tx_ts - resp_rx_ts);
                double Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);

                int64 tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
                tof = tof_dtu * DWT_TIME_UNITS;
                distance = tof * SPEED_OF_LIGHT;

                char str[32];
                snprintf(str, sizeof(str), "%.3f", distance);
                printk("<ranging> Distance: %s [m]\n", str);

                ble_reps->cnt = 1;
                ble_reps->ble_rep[0].seq_nb = frame_seq_nb;
                ble_reps->ble_rep[0].node_id = DEVICE_ANC_ID;
                ble_reps->ble_rep[0].dist = (float)distance;
                ble_reps->ble_rep[0].tqf = 0;

                dwm1001_notify((uint8_t*)ble_buf, 1 + sizeof(ble_rep_t) * ble_reps->cnt);
                k_yield();

                frame_seq_nb_rx = frame_seq_nb + 1;
            }
        } else {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
            printk("<ranging> RX Error\n");
            continue;
        }
    }
    return 0;
}

