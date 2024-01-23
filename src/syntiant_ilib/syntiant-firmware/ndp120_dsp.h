/*
 * SYNTIANT CONFIDENTIAL
 *
 * _____________________
 *
 * Copyright (c) 2020 Syntiant Corporation
 * All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains the property of
 * Syntiant Corporation and its suppliers, if any.  The intellectual and
 * technical concepts contained herein are proprietary to Syntiant Corporation
 * and its suppliers and may be covered by U.S. and Foreign Patents, patents in
 * process, and are protected by trade secret or copyright law.  Dissemination
 * of this information or reproduction of this material is strictly forbidden
 * unless prior written permission is obtained from Syntiant Corporation.
 *
 */

#ifndef __NDP120_DSP_H__
#define __NDP120_DSP_H__

#include <stdint.h>

/* DSP2MCU protocol */
enum dsp2mcu_protocol_e {
    DSP_TO_MCU_SEQ_MASK         = (1<<31), /* bit[31]    1 bit */
    DSP_TO_MCU_PAYLOAD_MASK     = 0xffff,
};

/* action number 8 bits */
enum dsp2mcu_cmd_e {
    DSP2MCU_PING     = 0, /* ping MCU*/
    DSP2MCU_DNN_DONE = 1, /* DNN done, bit[0:7] index of the output layer in ISA*/
    DSP2MCU_SYN      = 2,
    DSP2MCU_BASE_ADDR = 3,  /* read base address of the MIADDR */
    DSP2MCU_MIADDR_OFFSET = 4,
    DSP2MCU_DSP2H_REQ     = 5,
};

/* response from MCU2DSP  8 bits */
enum mcu2dsp_response_e {
    MCU2DSP_RESPONSE_PING_ACK       = 0,
    MCU2DSP_RESPONSE_NN_DONE_ACK    = 1,
    MCU2DSP_RESPONSE_SYN_ACK        = 2,
    MCU2DSP_RESPONSE_BASE_ADDR      = 3,
    MCU2DSP_RESPONSE_MIADDR_OFFSET  = 4,
    MCU2DSP_RESPONSE_ERROR          = 0x20,
    MCU2DSP_RESPONSE_DSP2H_RESP     = 5,
};


/* action  number 8 bits */
enum mcu2dsp_cmd_e {
    MCU2DSP_PING       = 0, /* ping DSP*/
    MCU2DSP_TANK_START = 1, /* tank buffer start, address [18:2] in dram  */
    MCU2DSP_TANK_END   = 2, /* tank buffer end address [18:2] in dram */
    MCU2DSP_MATCH      = 3,
    MCU2DSP_DSP_FW_BASE_ADDR_HI = 4,
    MCU2DSP_DSP_FW_BASE_ADDR_LO = 5,
    MCU2DSP_BYPASS_H2DSP_IND = 6,
};

/* action  number 8 bits */
enum dsp2mcu_response_e {
    DSP2MCU_RESPONSE_PING       = 0, /* ping response */
    DSP2MCU_RESPONSE_TANK_START = 1, /* tank buffer start, address [18:2] in dram  */
    DSP2MCU_RESPONSE_TANK_END   = 2, /* tank buffer end address [18:2] in dram */
    DSP2MCU_RESPONSE_DSP_FW_BASE_ADDR_HI = 4,
    DSP2MCU_RESPONSE_DSP_FW_BASE_ADDR_LO = 5,
    DSP2MCU_DSP2H_ACK           = 6,
};

enum mcu2dsp_error_e {
    ERR_UNKNOWN_CMD    = 1,
};

enum dsp_state_e {
    DSP_STATE_IDLE,
    DSP_STATE_WAIT_RESPONE,
};

enum dsp_status_e {    
    DSP_STATUS_GOT_PING_RESPONSE =1,
};


#define D2M_MESSAGE_QUEUE_SIZE  16
struct dsp2mcu_queue_s {
    uint32_t write_idx;
    uint32_t read_idx;
    uint32_t d2m[D2M_MESSAGE_QUEUE_SIZE];
} ;

struct ndp_dsp_state_s {
    uint8_t  dsp_state;
    uint8_t  dsp_previous_state;
    uint8_t  dsp_status;
    uint8_t  dsp_stall;

    uint32_t previous_dsp2mcu;
    uint32_t previous_mcu2dsp;
    uint32_t processing_d2m;
    uint32_t processed_responsed;

    uint32_t mcu_sequence_number;
    uint32_t dsp_sequence_number;

    struct dsp2mcu_queue_s d2m_queue;
};



/**
 * @brief initializes the DSP state.
 *
 * @param dsp_state   dsp state object
 */
void dsp_init(struct ndp_dsp_state_s *dsp_state);


/**
 * @brief initializes the DSP state.
 *
 * @param dsp_state   dsp state object
 * @param stall       put dsp to stall state 
 * 
 * @return previous stall state
 */
int dsp_stall(struct ndp_dsp_state_s *dsp_state,int stall);


void ndp_dsp_send_match(struct ndp_dsp_state_s *dsp_state ,struct ndp120_result_s *current_result);

uint32_t ndp_dsp_send_ping(struct ndp_dsp_state_s *dsp_state);

/**
 * @brief init the DSP2MCU queue
 *
*/
void dsp2mcu_queue_init(struct ndp_dsp_state_s *dsp_state);

/**
 * @brief DMA transfer from src to dst
 *      where src and dst should be 16 bytes alignement to get
 *      best performance
 *
 * @param dst    destination address
 * @param src    destination address
 * @param len    length
 */
int dma_transfer(uint32_t dst, uint32_t src, uint32_t len);


#define D2M_TYPE_REQUEST    0
#define D2M_TYPE_RESPONSE   1

/*
 *  Values are 32 bit values laid out as follows:
 *
 *   3 3 2 2 2 2 2 2 2 2 2 2 1 1 1 1 1 1 1 1 1 1
 *   1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0
 *  +-+-+-----------+---------------+-------------------------------+
 *  |T|t| Sequence  | message       |               response        |
 *  +-+-+-----------+---------------+-------------------------------+
 *
 *  where
 *
 *      T - is Toggle bit change change in each transaction
 *
 *      t - is type
 *          0 - Request
 *          1 - response
 *  
 *      Sequence - is 6 bit sequence
 *  
 *      Message -  Request/resposne message
 *
 *      payload - payload
 */
struct dsp_mcu_handshake_s {
    union {
        struct {
            uint32_t payload:16;
            uint32_t message:8;
            uint32_t seq:6;
            uint32_t type:1;
            uint32_t toggle:1;
        };

        uint32_t value;
    };
}  __attribute__((packed));


void
ndp_dsp_send_m2d_command(
    struct ndp_dsp_state_s *dsp_state,
    uint32_t request,
    uint32_t payload);

void
ndp_dsp_send_m2d_response(
    struct ndp_dsp_state_s *dsp_state,
    uint32_t response,
    uint32_t payload);

#endif


