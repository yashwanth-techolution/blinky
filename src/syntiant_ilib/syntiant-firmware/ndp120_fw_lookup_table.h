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

#ifndef NDP120_LOOKUP_TABLE_H
#define NDP120_LOOKUP_TABLE_H


struct ndp10x_fw_lookup_table_s {
    uint32_t fw_state_addr;
    uint32_t ph_state_addr;
    uint32_t smooth_params_addr;
    uint32_t ph_params_addr;
    uint32_t address[4];
};

#define MAX_PH  6

#endif
