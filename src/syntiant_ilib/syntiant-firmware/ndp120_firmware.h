/*
 * SYNTIANT CONFIDENTIAL
 *
 * _____________________
 *
 * Copyright (c) 2017-2018 Syntiant Corporation
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

/*
 * NOTE:
 * 1. this header file is used in the syntiant-ilib repo as an interface file
 * 2. Some compilers do not pack the structs which means they are always 32 bit
 * aligned so they are explicitly made 32 bit aligned so please follow that.
 */

#ifndef NDP120_FIRMWARE_H
#define NDP120_FIRMWARE_H

#include "syntiant-firmware/ndp120_result.h"
#include "syntiant-firmware/ndp120_mb.h"
#include "syntiant-firmware/ndp120_fw_lookup_table.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FW_VER "##FIRMWARE_VERSION##"
#define FW_VER_SIZE 24

#define NDP120_ILIB_SCRATCH_ORIGIN 0x20002000
#define NDP120_ILIB_SCRATCH_LENGTH 0x00c00
enum {
    NDP120_FW_STATE_ADDRESS_INDEX_FW_STATE              = 0x0,
    NDP120_FW_STATE_ADDRESS_INDEX_POSTERIOR_STATE       = 0x1,
    NDP120_FW_STATE_ADDRESS_INDEX_SMAX_SMOOTHER         = 0x2,
    NDP120_FW_STATE_ADDRESS_INDEX_POSTERIOR_PARAMS      = 0x3,
    NDP120_FW_STATE_ADDRESS_INDEX_ORCHESTRATOR_PARAMS   = 0x4
};

struct ndp120_fw_state_pointers_s {
    uint32_t addresses[8];
};

enum ndp120_firmware_constants_e {
    NDP120_MATCH_RING_SIZE = 2
};


struct ndp120_fw_match_s {
    uint32_t summary;
    uint32_t tankptr;
};
    
/**
 * @brief Data Structure for storing firmware state.
 * Note: Please do not change this data struture because this struture is used
 * by ilib running on host for operational or debugging purposes.
 * Please add your data after this data structure and set the address for your
 * data in fw_state_pointers.
 *
 */
struct ndp120_fw_state_s {
    /*
     * These members must be in fixed locations and never change.
     * The uILib relies on these members and does not include
     * the firmware header files.
     */
    uint32_t tankptr;     /* tank pointer with all 17 bits */
    uint32_t match_producer[MAX_PH];
    struct ndp120_fw_match_s match_ring[MAX_PH][NDP120_MATCH_RING_SIZE];

    /*
     * the remaining members should be kept in a stable location but
     * will be accessed through these header files by the full ILib
     * so there is less pain of death to moving them
     */
    uint32_t debug;                   /* debug scratch area */
    uint32_t enable;
    uint32_t prev_enable;
    uint32_t reset;

    uint32_t tank_size;
    uint32_t tank_full;
    uint32_t result_fifo_full;

    /* interrupt counters*/
    uint32_t mb_int_count;
    uint32_t freq_int_count;
    uint32_t dnn_int_count;
    uint32_t unknown_int_count;

    /* firmware version */
    char version[FW_VER_SIZE];

    /* mailbox state */
    struct ndp120_mb_state_s mb_state;

    /* frame results after posterior computations */
    struct ndp120_result_s result[MAX_PH];
};

/**
 * @brief enum values to enable/disable various components of firmware. This
 * constants will also be used by ilib code to enable/disable the components.
 *
 * Note: please release header files to ilib if you change this constants.
 */
enum ndp120_fw_state_address_enable_e {
    NDP120_FW_STATE_ENABLE_POSTERIOR = 0x1,
    NDP120_FW_STATE_ENABLE_SMAX_SMOOTHER = 0x2
};

/**
 * @brief enum values to reset various components of firmware. This
 * constants will also be used by ilib code to reset the components.
 *
 * Note: please release header files to ilib if you change this constants.
 */
enum ndp120_fw_state_address_reset_e {
    NDP120_FW_STATE_RESET_POSTERIOR_STATE = 0x1,
    NDP120_FW_STATE_RESET_SMAX_SMOOTHER = 0x2,
    NDP120_FW_STATE_RESET_POSTERIOR_PARAMS = 0x4
};

enum {
    NDP120_LINEAR_ACTIVATION_DATATYPE = 0,
    NDP120_LINEAR16_ACTIVATION_DATATYPE = 1,
    NDP120_RELU_ACTIVATION_DATATYPE = 2,
    NDP120_TANH_ACTIVATION_DATATYPE = 3
};

#define NDP120_DNN_LAYER_ISA_WIDTH (32)

#ifdef __cplusplus
}
#endif
#endif /* NDP120_FIRMWARE_H */
