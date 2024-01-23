/*
 * SYNTIANT CONFIDENTIAL
 * _____________________
 *
 *   Copyright (c) 2020 Syntiant Corporation
 *   All Rights Reserved.
 *
 *  NOTICE:  All information contained herein is, and remains the property of
 *  Syntiant Corporation and its suppliers, if any.  The intellectual and
 *  technical concepts contained herein are proprietary to Syntiant Corporation
 *  and its suppliers and may be covered by U.S. and Foreign Patents, patents in
 *  process, and are protected by trade secret or copyright law.  Dissemination
 *  of this information or reproduction of this material is strictly forbidden
 *  unless prior written permission is obtained from Syntiant Corporation.
 */
#ifndef __NDP120_ISA_H_
#define __NDP120_ISA_H_

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

/**
 * ISA structure
 * NDP120_DNNPARAMS(0x60080000)
 */
struct dnn_ISA_s {
    uint32_t type:3;
    uint32_t activation:2;
    uint32_t bank_in:1;
    uint32_t bank_out:1;
    uint32_t signed_input:1;
    uint32_t input_base_coord:18;
    uint32_t output_base_coord_lsb:6;

    uint32_t output_base_coord_msb:12;
    uint32_t weights_base_addr:14;
    uint32_t bias_base_addr_lsb:6;

    uint32_t bias_base_addr_msb:8;
    uint32_t scaler_base_addr:14;
    uint32_t input_size0_lsb:10;

    uint32_t input_size0_msb:2;
    uint32_t input_size1:12;
    uint32_t input_size2:12;
    uint32_t output_size0_lsb:6;

    uint32_t output_size0_msb:6;
    uint32_t output_size1:12;
    uint32_t output_size2:12;
    uint32_t input_offset_lsb:2;

    uint32_t input_offset_msb:10;
    uint32_t num_filters:12;
    uint32_t kernel_pooling0:6;
    uint32_t kernel_pooling1_lsb:4;

    uint32_t kernel_pooling2_msb:2;
    uint32_t padding_size0:6;
    uint32_t padding_size1:6;
    uint32_t padding_size2:6;
    uint32_t padding_size3:6;
    uint32_t weights_width:3;
    uint32_t select_stride_dilation:1;
    uint32_t stride_dilation0_lsb:2;

    uint32_t stride_dilation0_msb:4;
    uint32_t stride_dilation2:6;
    uint32_t stop:1;
    uint32_t interrupt:8;
    uint32_t next_pointer:8;
    uint32_t signature:5;

} __attribute__((packed));


/* sarch the a*/
uint32_t
ndp_isa_get_final_output_layer_idx(
    uint32_t *layer_idx);

uint8_t
ndp_isa_read_output_addr(
    uint8_t layer_idx,
    uint32_t *output_addr,
    uint32_t *output_len);

void
ndp_isa_write_output_addr(
    uint32_t layer_idx,
    uint32_t output_addr);

void
ndp_isa_read_input_addr(
    uint32_t layer_idx,
    uint32_t *input_addr,
    uint32_t *input_len);

void
ndp_isa_write_input_addr(
    uint32_t layer_idx,
    uint32_t input_addr);


void init_read_input_layer(
    uint32_t *input_addr, /* out */
    uint32_t *input_len);  /* out*/


uint32_t ndp_isa_read_input_offset(
    uint32_t layer_idx,
    uint32_t *offset); /* out*/

void ndp_isa_write_input_offset(
    uint32_t layer_idx,
    uint32_t offset);

#endif


