#include <string.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi.h"
#include "port.h"

#include <zephyr.h>
#include <sys/printk.h>

#define LOG_LEVEL 3
#include <logging/log.h>
LOG_MODULE_REGISTER(main);

#define APP_HEADER  "*** DWM1001-Localization Project ***\n"
#define DEV_TYPE    "Tag\n"
#define APP_LINE    "============================\n"

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
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

// Delay for random number generation (in milliseconds)
#define RNG_DELAY_MS 5000


////////////////////////////////////////////////////////////////////////////////
// DEV CONTROL                                                                //
////////////////////////////////////////////////////////////////////////////////
#define DEVCTRL_NUM_ANCHORS 4
#define DEVCTRL_ANCHOR1_ADDR 0xBB01
#define DEVCTRL_ANCHOR2_ADDR 0xBB02
#define DEVCTRL_ANCHOR3_ADDR 0xBB03
#define DEVCTRL_ANCHOR4_ADDR 0xBB04

////////////////////////////////////////////////////////////////////////////////
// RANGING MESSAGES                                                           //
////////////////////////////////////////////////////////////////////////////////
// Length of the common part of the message
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4

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

////////////////////////////////////////////////////////////////////////////////
// INTERNAL FUNCTIONS                                                         //
////////////////////////////////////////////////////////////////////////////////

#define RX_BUF_LEN 20
#define UUS_TO_DWT_TIME 65536
#define POLL_TX_TO_RESP_RX_DLY_UUS 300
#define RESP_RX_TO_FINAL_TX_DLY_UUS 4000//4000
#define RESP_RX_TIMEOUT_UUS 6000//6000

static uint8 rx_buffer[RX_BUF_LEN];
static uint8 frame_seq_nb = 0;
static uint8 frame_seq_nb_rx = 0;
static uint32 status_reg = 0;

static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;

typedef unsigned long long uint64;

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

static inline void display_status_reg(uint32 status_reg){
    if(status_reg & SYS_STATUS_IRQS) printk(" - IRQs set\n");
    if(status_reg & SYS_STATUS_CPLOCK) printk(" - Clock PLL Locked\n");
    if(status_reg & SYS_STATUS_AAT) printk(" - Automatic Acknowledge Triggered\n");
    if(status_reg & SYS_STATUS_TXFRB) printk(" - Transmit Frame Begins\n");
    if(status_reg & SYS_STATUS_TXPRS) printk(" - Transmit Preamble Sent\n");
    if(status_reg & SYS_STATUS_TXPHS) printk(" - Transmit PHY Header Sent\n");
    if(status_reg & SYS_STATUS_TXFRS) printk(" - Transmit Frame Sent\n");
    if(status_reg & SYS_STATUS_RXPRD) printk(" - Receiver Preamble Detected\n");
    if(status_reg & SYS_STATUS_RXSFDD) printk(" - Receiver SFD Detected\n");
    if(status_reg & SYS_STATUS_LDEDONE) printk(" - LDE Processing Done\n");
    if(status_reg & SYS_STATUS_RXPHD) printk(" - Receiver PHY Header Detected\n");
    if(status_reg & SYS_STATUS_RXPHE) printk(" - Receiver PHY Header Error\n");
    if(status_reg & SYS_STATUS_RXDFR) printk(" - Receiver Data Frame Ready\n");
    if(status_reg & SYS_STATUS_RXFCG) printk(" - Receiver FCS Good\n");
    if(status_reg & SYS_STATUS_RXFCE) printk(" - Receiver FCS Error\n");
    if(status_reg & SYS_STATUS_RXRFSL) printk(" - Receiver Reed Solomon Frame Sync Loss\n");
    if(status_reg & SYS_STATUS_RXRFTO) printk(" - Receive Frame Wait Timeout\n");
    if(status_reg & SYS_STATUS_LDEERR) printk(" - Leading Edge Detection Error\n");
    if(status_reg & SYS_STATUS_RXOVRR) printk(" - Receiver Overrun\n");
    if(status_reg & SYS_STATUS_RXPTO) printk(" - Preamble Detection Timeout\n");
    if(status_reg & SYS_STATUS_GPIOIRQ) printk(" - GPIO Interrupt\n");
    if(status_reg & SYS_STATUS_SLP2INIT) printk(" - SLEEP to INIT Transitioned\n");
    if(status_reg & SYS_STATUS_RFPLL_LL) printk(" - RF PLL Losing Lock\n");
    if(status_reg & SYS_STATUS_CLKPLL_LL) printk(" - Clock PLL Losing Lock\n");
    if(status_reg & SYS_STATUS_RXSFDTO) printk(" - Receive SFD Timeout\n");
    if(status_reg & SYS_STATUS_HPDWARN) printk(" - Half Period Delay Warning\n");
    if(status_reg & SYS_STATUS_TXBERR) printk(" - Transmit Buffer Error\n");
    if(status_reg & SYS_STATUS_AFFREJ) printk(" - Automatic Frame Filtering Rejection\n");
    if(status_reg & SYS_STATUS_HSRBP) printk(" - Host Side Receive Buffer Pointer\n");
    if(status_reg & SYS_STATUS_ICRBP) printk(" - IC Side Receive Buffer Pointer\n");
}

static void device_init(void){
    openspi(); reset_DW1000();
    port_set_dw1000_slowrate();
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        printk("<dev> Initialization failed");
        k_sleep(K_MSEC(500));
        while (1) {};
    }
    port_set_dw1000_fastrate();
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int dw_main(void) {
    // Print application header
    printk("\n");
    printk(APP_LINE);
    printk(APP_HEADER);
    printk("Device Type: ");
    printk(DEV_TYPE);

    // initialize the DW1000 device
    device_init();

    // configure the DW1000 device
    dwt_configure(&config);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    
    dwt_setleds(1);

    // set the device EUI-64 address
    dwt_seteui(eui_64);
    printk("Tag EUI-64: 0x");
    for (int i = 0; i < 8; i++) {
        printk("%02X", eui_64[i]);
    }
    printk("\n");
    printk(APP_LINE);

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

    while (1) {
        // Configure POLL message
        tag_poll_msg[2] = frame_seq_nb; // sequence number
        tag_poll_msg[5] = anchor_addresses[target_anchor][0]; 
        tag_poll_msg[6] = anchor_addresses[target_anchor][1]; // destination address

        // Displaying POLL message
        printk("------------------------------------------------------------\n");
        printk("<ranging> Send POLL: ");
        for(int i = 0; i < sizeof(tag_poll_msg); i++)
            printk("%02X ", tag_poll_msg[i]);
        printk("\n");
        
        // Transmit POLL message
        dwt_writetxdata(sizeof(tag_poll_msg), tag_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tag_poll_msg), 0, 1);
        if(dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED)){
            printk("<ranging> ERROR: POLL transmission failed\n");
            k_sleep(K_MSEC(1));
            continue;
        }
        
        // Listen anchor's RESP message
        while (
            !((status_reg = dwt_read32bitreg(SYS_STATUS_ID))
              & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))
        ){
            printk("------------------------------------------------------------\n");
            printk("<ranging> ERROR: RESP reception failed\n");
            printk("  - SYS_STATUS: %08lX\n", status_reg);
            dwt_forcetrxoff();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            k_sleep(K_MSEC(500));
        };

        if (status_reg & SYS_STATUS_RXFCG) {
            uint32 frame_len = dwt_read32bitreg(RX_FINFO_ID)
                                & RX_FINFO_RXFLEN_MASK;
            if (frame_len <= RX_BUF_LEN) {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            // validate frame
            frame_seq_nb_rx = rx_buffer[ALL_MSG_SN_IDX];
            uint16 mhr_dstadr = rx_buffer[5] | (rx_buffer[6] << 8);
            uint16 mhr_srcadr = rx_buffer[7] | (rx_buffer[8] << 8);
            uint16 func_code = rx_buffer[9];
            printk("------------------------------------------------------------\n");
            printk("<ranging> Received Frame: ");
            for(int i = 0; i < frame_len; i++) printk("%02X ", rx_buffer[i]);
            printk("\n");
            printk("    - Sequence Number: %d\n", frame_seq_nb_rx);
            printk("    - Destination Address: 0x%04X\n", mhr_dstadr);
            printk("    - Source Address: 0x%04X\n", mhr_srcadr);
            printk("    - Function Code: 0x%02X\n", func_code);

            // compute timestamp
            uint32 final_tx_time;
            int ret;
            poll_tx_ts = get_tx_timestamp_u64();
            
            while(1){
                resp_rx_ts = get_rx_timestamp_u64();
                final_tx_time = (resp_rx_ts +
                    (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;

                dwt_setdelayedtrxtime(final_tx_time);
                final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                final_msg_set_ts(&tag_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                final_msg_set_ts(&tag_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                final_msg_set_ts(&tag_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);
                
                tag_final_msg[2] = frame_seq_nb;
                tag_final_msg[5] = anchor_addresses[target_anchor][0]; // Set Anchor Address
                tag_final_msg[6] = anchor_addresses[target_anchor][1]; // Set Anchor Address

                // printk("------------------------------------------------------------\n");
                // printk("<ranging> Sended FINAL: ");
                // for(int i = 0; i < frame_len; i++) printk("%02X ", tag_final_msg[i]);
                // printk("\n");

                // Sending the final message
                dwt_writetxdata(sizeof(tag_final_msg), tag_final_msg, 0);
                dwt_writetxfctrl(sizeof(tag_final_msg), 0, 1);

                ret = dwt_starttx(DWT_START_TX_DELAYED);
                if (ret == DWT_SUCCESS) {
                    while (!(dwt_read32bitreg(SYS_STATUS_ID)& SYS_STATUS_TXFRS)) {};
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
                    printk("<ranging> TX FINAL successful\n");
                    frame_seq_nb++;
                    break;
                    // target_anchor = (target_anchor + 1) % DEVCTRL_NUM_ANCHORS; // Move to next anchor
                }
            }

        } else {
            dwt_write32bitreg(SYS_STATUS_ID,
                SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            printk("<ranging> RESP reception failed\n");
        }

        printk("<ranging> Waiting for next poll...\n");
        k_sleep(K_MSEC(RNG_DELAY_MS));
    }
}
