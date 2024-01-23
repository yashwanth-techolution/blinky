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

#ifndef _NDP120_MAIN_H_
#define _NDP120_MAIN_H_

#include "syntiant-firmware/ndp120_mb.h"
#include "syntiant-firmware/ndp120_result.h"
#include "syntiant-firmware/ndp120_pkg.h"
#include "syntiant-firmware/ndp120_dsp.h"
#include "syntiant-firmware/ndp120_ph.h"
#include "syntiant-firmware/ndp120_firmware.h"
#include "syntiant-firmware/ndp120_orchestrator.h"

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
#ifndef NULL
#define NULL 0
#endif

#define DNN_STATE_HI_ADDR_FILLED  0x1
#define DNN_STATE_LO_ADDR_FILLED  0x2

struct ndp120_main_state_s {
    /* base fw state that match with ndp10x also share with ilib*/
    struct ndp120_fw_state_s fw_state;

    /* state for pkg load*/
    struct ndp_pkg_state_s  pkg_state;

    /* state for dsp*/
    struct ndp_dsp_state_s dsp_state;

    /* Handle to dsp_dnn state */
    uint32_t dnn_state_addr;
    /* used duing filling in DNN state address from DSP */
    uint8_t dnn_init_state;
    /* keeps error, if any duing dnn state address fetch from DSP */
    uint8_t dnn_init_state_error;
};

extern struct ndp120_main_state_s  sfw_state;
extern struct ndp120_ph_state_s    sph_state[MAX_PH];
extern struct ndp120_result_softmax_smoother_s ssmoother_params[MAX_PH];
extern struct ndp120_ph_params_collection_s   sph_params_coll;
extern uint32_t irq_triggered;
extern ndp120_nno_node_t    g_ndp120_nno_nodes[MAX_NDP120_NNGRAPH_NODES];

struct ndp_fw_lookup_table_s {
    union {
        struct {
            /* fix the first 8 entries to be exaclty same as
             * ndp10x
             */
            uint32_t fw_state_addr;
            uint32_t ph_state_addr;
            uint32_t smooth_params_addr;
            uint32_t ph_params_addr;
            uint32_t nn_graph_addr;
            uint32_t reserved;

            uint32_t scratch_ram_addr;
            uint32_t fw_info_table_addr;
        };
        uint32_t address[32];
    };

    union {
        struct {
            uint32_t fw_state_size;
            uint32_t ph_state_size;
            uint32_t smooth_params_size;
            uint32_t ph_params_size;
            uint32_t nn_graph_size;
            uint32_t reserved_size;

            uint32_t scratch_ram_size;
            uint32_t fw_info_table_size;
        };
        uint32_t size[32];
    };
};

extern struct ndp_fw_lookup_table_s __fw_lookup_table_start__;
extern struct handshake_ram_T       __pktload_handshake_ram__;
extern uint32_t __secure_params_start__;
extern uint32_t __secure_params_end__;
extern uint32_t __scratch_ram_start__;
extern uint32_t __scratch_ram_end__;
extern uint32_t __fw_info_table_start__;
extern uint32_t __pkgload_open_ram_start__;
extern uint32_t __pkgload_open_ram_end__;

#if defined(DSP_TEST)
void ndp_d2m_nn_done_test_for_DSP(struct ndp120_main_state_s *fw_main_state);
#endif

/**
 * @brief handles the dnn interrupt. This interrupt happens whenever dnn block
 * completes one inference.
 *
 * @param fw_state
 */
void ndp_dnn_int_handler(struct ndp120_main_state_s *fw_main_state);

/**
 * @brief handles the mailbox interrupt. This interrupt happens whenever there
 * is a new Request from host or Response from host.
 *
 * @param fw_state
 */
void ndp_mb_int_handler(struct ndp120_main_state_s *fw_main_state);

/**
 * @brief handles the freq block interrupt. This interrupt happens whenever
 * freq block is done computing filterbanks for a frame.
 * @param fw_state
 */
void ndp_freq_int_handler(struct ndp120_main_state_s *fw_main_state);

/**
 * @brief handles the interrupts from DSP. 
 *  
 * @param fw_state
 */
void ndp_dsp2mcu_int_handler(struct ndp120_main_state_s *fw_main_state);
void dsp2mcu_process_queue(struct ndp120_main_state_s *fw_main_state);


/**
 * @brief handles the mbin  interrupts from host. 
 *  
 * @param fw_state
 */
void ndp_h2m_request_handler(struct ndp120_main_state_s* fw_main_state);


/**
 * @brief handles the mbout response interrupts from host.
 *  
 * @param fw_state
 */
void ndp_m2h_response_handler(struct ndp120_main_state_s* fw_main_state);


/**
 * @brief Reset the mailbox state machine to default state.
 *  
 * @param fw_state
 */
void ndp_reset_mailbox(struct ndp120_main_state_s* fw_main_state);

void ndp_d2m_nn_done(struct ndp120_main_state_s *fw_main_state,
        uint32_t sample_tank_idx);


/*----------------------------------------------------------*/
/* below are for debug only                                 */
/*----------------------------------------------------------*/
/* max 256 bytes */
/* max 64 debug counters */
struct ndp120_debug_cnt_s {
/*S */    uint32_t signature;
/*1 */    uint32_t dsp2mcu_intr_cnt;
/*2 */    uint32_t dsp2mcu_nn_done_cnt;
/*3 */    uint32_t mcu2host_match_cnt;
/*4 */    uint32_t mcu2host_frame_cnt;
/*5 */    uint32_t matches;
/*6 */    uint32_t expected_value;
/*7 */    uint32_t output_addr;
/*8 */    uint32_t dsp2mcu_queue_cnt;
/*9 */    uint32_t unknown_int_count;
/*a */    uint32_t mbin_int_cnt;
/*b */    uint32_t mbout_int_cnt;

#if defined(ENABLE_LOG)
          void *log_buffer_start;
          void *log_buffer_end;
          uint32_t log_cur_idx;
          uint32_t log_stopat;
#endif
          uint32_t enable;
          uint32_t dbg1;
          uint32_t dbg2;
/*end */  uint32_t signature_end;
};

extern struct ndp120_debug_cnt_s __debug_counters_table__;
extern struct ndp120_debug_cnt_s *dbg_state;

enum debug_print_enable_e {
        eDBGPRINT_ENABLE_DSP2MCU = (1<<0),
        eDBGPRINT_ENABLE_MCU2DSP = (1<<1),
        eDBGPRINT_ENABLE_MBOX    = (1<<2),
        eDBGPRINT_TAG_LEN        = (1<<3),
        eDBGPRINT_ACTIVATIONS    = (1<<4),
        eDBGPRINT_READ_MBOX      = (1<<5),
};
#endif
