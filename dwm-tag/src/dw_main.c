#include <zephyr.h>
#include <sys/printk.h>

#include <string.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi.h"
#include "port.h"

#include "ble_device.h"

#define LOG_LEVEL 3
#include <logging/log.h>
LOG_MODULE_REGISTER(main);

// Data Strings
#define APP_HEADER  "*** DWM1001-Localization Project ***"
#define DEV_TYPE    "Tag"
#define PART_LINE   "----------------------------------------"

// Default communication configuration
static dwt_config_t config = {
    5,               // Channel number
    DWT_PRF_64M,     // Pulse Repetition Frequency (64 MHz)
    DWT_PLEN_128,    // Preamble Length (128 symbols)
    DWT_PAC8,        // Preamble Accumulator (8)
    9,               // TX Preamble Code
    9,               // RX Preamble Code
    1,               // SFD Timeout (0 to use standard SFD, 1 to use non-standard SFD)
    DWT_BR_6M8,      // Bit Rate (6.8 Mbps)
    DWT_PHRMODE_STD, // PHR Mode (Standard)
    129              // SFD Length (129 symbols)
};

// EUI-64 address (Unique Device Identifier)
uint8 eui_64[8] = {0xDE, 0xCA, 0x01, 0x02, 0x03, 0x04, 0xAB, 0xCD};

// Default antenna delay values for 64 MHz PRF
//16436
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

// Inter-ranging delay period, in milliseconds
#define RNG_DELAY_MS 200

// Device Informations
#define DEVCTRL_NUM_ANCHORS 4
#define DEVCTRL_ANCHOR1_ADDR 0xBB01
#define DEVCTRL_ANCHOR2_ADDR 0xBB02
#define DEVCTRL_ANCHOR3_ADDR 0xBB03
#define DEVCTRL_ANCHOR4_ADDR 0xBB04

// Length of the common part of the message
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4

// Frame sequence number
static uint8 frame_seq_nb = 0;
static uint8 frame_seq_nb_rx = 0;

// Receive Buffer
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];

// Status Register
static uint32 status_reg = 0;

// UWB microsecond (uus) to device time unit (dtu, 15.65 ps)
#define UUS_TO_DWT_TIME 65536

// Delay between frames, in uus
#define POLL_TX_TO_RESP_RX_DLY_UUS 300

// Receive response timeout
#define RESP_RX_TO_FINAL_TX_DLY_UUS 4000
#define RESP_RX_TIMEOUT_UUS 6000

static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;

typedef unsigned long long uint64;

uint16 mhr_dstadr;
uint16 mhr_srcadr;
uint16 func_code;

static uint8 tag_poll_msg[] = {
    // Frame Control
    0x41, 0x88,
    // Sequence Number (SN)
    0x00,
    // PAN ID
    0xCA, 0xDE,
    // Dest. Address (Anchor Address)
    0x01, 0xBB,
    // Source Address (Tag Short Address)
    0x01, 0xAA,
    // Function Code (0x61 for poll msg)
    0x61,
    // For FCS
    0x00,0x00
};

static uint8 tag_final_msg[] = {
    // Frame Control
    0x41, 0x88,
    // Sequence Number (SN)
    0x00,
    // PAN ID
    0xCA, 0xDE,
    // Dest. Address (Anchor Address)
    0x01, 0xBB,
    // Source Address (Tag Short Address)
    0x01, 0xAA,
    // Function Code (0x61 for poll msg)
    0x69,
    // poll_tx_ts (10~)
    0x00, 0x00, 0x00, 0x00,
    // resp_rx_ts (14~)
    0x00, 0x00, 0x00, 0x00,
    // final_tx_ts (18~)
    0x00, 0x00, 0x00, 0x00,
    // For FCS
    0x00, 0x00
};

static inline uint64 get_tx_timestamp_u64(void) {
    uint8 ts_tab[5]; uint64 ts = 0;
    dwt_readtxtimestamp(ts_tab);
    for (int i = 4; i >= 0; i--) {
        ts <<= 8; ts |= ts_tab[i];
    }
    return ts;
}

static inline uint64 get_rx_timestamp_u64(void) {
    uint8 ts_tab[5]; uint64 ts = 0;
    dwt_readrxtimestamp(ts_tab);
    for (int i = 4; i >= 0; i--) {
        ts <<= 8; ts |= ts_tab[i];
    }
    return ts;
}

static inline void final_msg_set_ts(uint8 *ts_field, uint64 ts) {
    for (int i = 0; i < FINAL_MSG_TS_LEN; i++) {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int dw_main(void) {
    // Print application header
    LOG_INF(PART_LINE);
    LOG_INF(APP_HEADER);
    LOG_INF("Device Type: %s", DEV_TYPE);

    // initialize the DW1000 device
    openspi();
    reset_DW1000();
    port_set_dw1000_slowrate();
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        LOG_ERR("Device initialization failed");
        k_sleep(K_MSEC(500));
        while (1) {};
    }
    port_set_dw1000_fastrate();

    // configure the DW1000 device
    dwt_configure(&config);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    dwt_setleds(1);

    // set the device EUI-64 address
    dwt_seteui(eui_64);
    LOG_INF("Tag EUI-64: 0x%02X%02X%02X%02X%02X%02X%02X%02X",
                eui_64[0],eui_64[1],eui_64[2],eui_64[3],
                eui_64[4],eui_64[5],eui_64[6],eui_64[7]);

    // uint8 num_anchors = DEVCTRL_NUM_ANCHORS;
    uint8 anchor_addresses[DEVCTRL_NUM_ANCHORS][2];
    uint8 target_anchor = 0;

    anchor_addresses [0][0] = DEVCTRL_ANCHOR1_ADDR & 0xFF;
    anchor_addresses [0][1] = (DEVCTRL_ANCHOR1_ADDR >> 8) & 0xFF;
    anchor_addresses [1][0] = DEVCTRL_ANCHOR2_ADDR & 0xFF;
    anchor_addresses [1][1] = (DEVCTRL_ANCHOR2_ADDR >> 8) & 0xFF;
    anchor_addresses [2][0] = DEVCTRL_ANCHOR3_ADDR & 0xFF;
    anchor_addresses [2][1] = (DEVCTRL_ANCHOR3_ADDR >> 8) & 0xFF;
    anchor_addresses [3][0] = DEVCTRL_ANCHOR4_ADDR & 0xFF;
    anchor_addresses [3][1] = (DEVCTRL_ANCHOR4_ADDR >> 8) & 0xFF;

    k_yield();

    // Initialize BLE Buffer
    ble_reps_t * ble_reps;
    uint8_t ble_buf[120] = {0};
    ble_reps = (ble_reps_t *)(&ble_buf[0]);

    k_yield();

    while (1) {
        LOG_DBG(PART_LINE);
        LOG_DBG("Sending POLL message");

        if(target_anchor == 0){
            LOG_INF(PART_LINE);
            LOG_INF("Sequence Number %02X", frame_seq_nb);
            frame_seq_nb++;
        }

        // Configure POLL message
        tag_poll_msg[2] = frame_seq_nb; // sequence number
        memcpy(&tag_poll_msg[5], &anchor_addresses[target_anchor], 2);

        // Transmit POLL message
        dwt_writetxdata(sizeof(tag_poll_msg), tag_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tag_poll_msg), 0, 1);
        if(dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED)){
            LOG_ERR("** ERROR: POLL transmission failed");
            continue;
        }

        // Listen anchor's RESP message
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID))
              & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        { };

        if (status_reg & SYS_STATUS_RXFCG) {
            LOG_DBG(PART_LINE);
            LOG_DBG("Receiving RESP message");

            uint32 frame_len;
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRB );

            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (frame_len <= RX_BUF_LEN) {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            // validate frame
            frame_seq_nb_rx = rx_buffer[2];
            memcpy(&mhr_dstadr, &rx_buffer[5], 2);
            memcpy(&mhr_srcadr, &rx_buffer[7], 2);
            func_code = rx_buffer[9];

            LOG_DBG(PART_LINE);
            LOG_DBG("Sending FINAL message");

            // compute timestamp
            uint32 final_tx_time;
            int ret;

            poll_tx_ts = get_tx_timestamp_u64();
            resp_rx_ts = get_rx_timestamp_u64();
            final_tx_time = (resp_rx_ts +
                (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;

            dwt_setdelayedtrxtime(final_tx_time);

            final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

            final_msg_set_ts(&tag_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
            final_msg_set_ts(&tag_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
            final_msg_set_ts(&tag_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

            tag_final_msg[2] = frame_seq_nb;
            memcpy(&tag_final_msg[5], &anchor_addresses[target_anchor], 2); // Set Anchor Address

            dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
            // Sending the final message
            dwt_writetxdata(sizeof(tag_final_msg), tag_final_msg, 0);
            dwt_writetxfctrl(sizeof(tag_final_msg), 0, 1);

            ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
            if (ret == DWT_SUCCESS) {
                while (!(dwt_read32bitreg(SYS_STATUS_ID)& SYS_STATUS_TXFRS))
                { };
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS); // clear TXFRS event
                LOG_DBG("TX FINAL complete, SN: %02X", frame_seq_nb);
            }

            // Listen anchor's REPORT message
            while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID))
                & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
            { };

            if (status_reg & SYS_STATUS_RXFCG) {
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRB );

                frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                if (frame_len <= RX_BUF_LEN) {
                    dwt_readrxdata(rx_buffer, frame_len, 0);
                }

                float distance;
                memcpy(&distance, &rx_buffer[10], sizeof(float));

                char dist_str[16];
                snprintf(dist_str, 16, "%.4f", distance);
                LOG_INF("- Dev: %d, Distance: %s  ", target_anchor, log_strdup(dist_str));

                ble_reps->cnt = 1;
                ble_reps->ble_rep[0].seq_nb = frame_seq_nb;
                ble_reps->ble_rep[0].node_id = target_anchor;
                ble_reps->ble_rep[0].dist = distance;
                ble_reps->ble_rep[0].tqf = 0;
                dwm1001_notify((uint8_t*)ble_buf, 1 + sizeof(ble_rep_t) * ble_reps->cnt);

                target_anchor = (target_anchor + 1) % DEVCTRL_NUM_ANCHORS; // Move to next anchor

            } else {
                LOG_ERR("** REPORT reception failed (timeout)");
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                dwt_rxreset();
            }
        } else {
            LOG_ERR("** RESP reception failed (timeout)");
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
        }
        if(target_anchor == 0)
            k_sleep(K_MSEC(RNG_DELAY_MS));
        else 
            k_sleep(K_MSEC(10));
    }
}
