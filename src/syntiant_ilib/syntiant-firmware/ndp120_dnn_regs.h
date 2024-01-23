/*
 * 
 * SYNTIANT CONFIDENTIAL
 * _____________________
 *  
 *   Copyright (c) 2018 Syntiant Corporation
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
/*
 * ******* Automatically generated for ndp120_dnn v0.10  DO NOT MODIFY!!!!!
 *          Generated Sat Nov 30 23:55:41 2019 UTC
 */
#ifndef NDP120_DNN_REGS_H
#define NDP120_DNN_REGS_H

/*
 * block ndp120_dnn.isa, base 0x60080000
 */
#define NDP120_DNN_ISA 0x60080000
/* register ndp120_dnn.isa.comp0 */
#define NDP120_DNN_ISA_COMP0 0x60080000
#define NDP120_DNN_ISA_COMP0_TYPE_SHIFT 0
#define NDP120_DNN_ISA_COMP0_TYPE_MASK 0x00000007
#define NDP120_DNN_ISA_COMP0_TYPE(v) \
        ((v) << NDP120_DNN_ISA_COMP0_TYPE_SHIFT)
#define NDP120_DNN_ISA_COMP0_TYPE_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP0_TYPE_SHIFT))
#define NDP120_DNN_ISA_COMP0_TYPE_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP0_TYPE_MASK) | ((v) << NDP120_DNN_ISA_COMP0_TYPE_SHIFT))
#define NDP120_DNN_ISA_COMP0_TYPE_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP0_TYPE_MASK) >> NDP120_DNN_ISA_COMP0_TYPE_SHIFT)
#define NDP120_DNN_ISA_COMP0_ACTIVATION_SHIFT 3
#define NDP120_DNN_ISA_COMP0_ACTIVATION_MASK 0x00000018
#define NDP120_DNN_ISA_COMP0_ACTIVATION(v) \
        ((v) << NDP120_DNN_ISA_COMP0_ACTIVATION_SHIFT)
#define NDP120_DNN_ISA_COMP0_ACTIVATION_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP0_ACTIVATION_SHIFT))
#define NDP120_DNN_ISA_COMP0_ACTIVATION_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP0_ACTIVATION_MASK) | ((v) << NDP120_DNN_ISA_COMP0_ACTIVATION_SHIFT))
#define NDP120_DNN_ISA_COMP0_ACTIVATION_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP0_ACTIVATION_MASK) >> NDP120_DNN_ISA_COMP0_ACTIVATION_SHIFT)
#define NDP120_DNN_ISA_COMP0_BANK_IN_SHIFT 5
#define NDP120_DNN_ISA_COMP0_BANK_IN_MASK 0x00000020
#define NDP120_DNN_ISA_COMP0_BANK_IN(v) \
        ((v) << NDP120_DNN_ISA_COMP0_BANK_IN_SHIFT)
#define NDP120_DNN_ISA_COMP0_BANK_IN_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP0_BANK_IN_SHIFT))
#define NDP120_DNN_ISA_COMP0_BANK_IN_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP0_BANK_IN_MASK) | ((v) << NDP120_DNN_ISA_COMP0_BANK_IN_SHIFT))
#define NDP120_DNN_ISA_COMP0_BANK_IN_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP0_BANK_IN_MASK) >> NDP120_DNN_ISA_COMP0_BANK_IN_SHIFT)
#define NDP120_DNN_ISA_COMP0_BANK_OUT_SHIFT 6
#define NDP120_DNN_ISA_COMP0_BANK_OUT_MASK 0x00000040
#define NDP120_DNN_ISA_COMP0_BANK_OUT(v) \
        ((v) << NDP120_DNN_ISA_COMP0_BANK_OUT_SHIFT)
#define NDP120_DNN_ISA_COMP0_BANK_OUT_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP0_BANK_OUT_SHIFT))
#define NDP120_DNN_ISA_COMP0_BANK_OUT_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP0_BANK_OUT_MASK) | ((v) << NDP120_DNN_ISA_COMP0_BANK_OUT_SHIFT))
#define NDP120_DNN_ISA_COMP0_BANK_OUT_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP0_BANK_OUT_MASK) >> NDP120_DNN_ISA_COMP0_BANK_OUT_SHIFT)
#define NDP120_DNN_ISA_COMP0_SIGNED_INPUT_SHIFT 7
#define NDP120_DNN_ISA_COMP0_SIGNED_INPUT_MASK 0x00000080
#define NDP120_DNN_ISA_COMP0_SIGNED_INPUT(v) \
        ((v) << NDP120_DNN_ISA_COMP0_SIGNED_INPUT_SHIFT)
#define NDP120_DNN_ISA_COMP0_SIGNED_INPUT_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP0_SIGNED_INPUT_SHIFT))
#define NDP120_DNN_ISA_COMP0_SIGNED_INPUT_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP0_SIGNED_INPUT_MASK) | ((v) << NDP120_DNN_ISA_COMP0_SIGNED_INPUT_SHIFT))
#define NDP120_DNN_ISA_COMP0_SIGNED_INPUT_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP0_SIGNED_INPUT_MASK) >> NDP120_DNN_ISA_COMP0_SIGNED_INPUT_SHIFT)
#define NDP120_DNN_ISA_COMP0_INPUT_BASE_COORD_SHIFT 8
#define NDP120_DNN_ISA_COMP0_INPUT_BASE_COORD_MASK 0x03ffff00
#define NDP120_DNN_ISA_COMP0_INPUT_BASE_COORD(v) \
        ((v) << NDP120_DNN_ISA_COMP0_INPUT_BASE_COORD_SHIFT)
#define NDP120_DNN_ISA_COMP0_INPUT_BASE_COORD_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP0_INPUT_BASE_COORD_SHIFT))
#define NDP120_DNN_ISA_COMP0_INPUT_BASE_COORD_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP0_INPUT_BASE_COORD_MASK) | ((v) << NDP120_DNN_ISA_COMP0_INPUT_BASE_COORD_SHIFT))
#define NDP120_DNN_ISA_COMP0_INPUT_BASE_COORD_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP0_INPUT_BASE_COORD_MASK) >> NDP120_DNN_ISA_COMP0_INPUT_BASE_COORD_SHIFT)
#define NDP120_DNN_ISA_COMP0_OUTPUT_BASE_COORD_LSB_SHIFT 26
#define NDP120_DNN_ISA_COMP0_OUTPUT_BASE_COORD_LSB_MASK 0xfc000000
#define NDP120_DNN_ISA_COMP0_OUTPUT_BASE_COORD_LSB(v) \
        ((v) << NDP120_DNN_ISA_COMP0_OUTPUT_BASE_COORD_LSB_SHIFT)
#define NDP120_DNN_ISA_COMP0_OUTPUT_BASE_COORD_LSB_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP0_OUTPUT_BASE_COORD_LSB_SHIFT))
#define NDP120_DNN_ISA_COMP0_OUTPUT_BASE_COORD_LSB_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP0_OUTPUT_BASE_COORD_LSB_MASK) | ((v) << NDP120_DNN_ISA_COMP0_OUTPUT_BASE_COORD_LSB_SHIFT))
#define NDP120_DNN_ISA_COMP0_OUTPUT_BASE_COORD_LSB_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP0_OUTPUT_BASE_COORD_LSB_MASK) >> NDP120_DNN_ISA_COMP0_OUTPUT_BASE_COORD_LSB_SHIFT)
/* register ndp120_dnn.isa.comp1 */
#define NDP120_DNN_ISA_COMP1 0x60080004
#define NDP120_DNN_ISA_COMP1_OUTPUT_BASE_COORD_MSB_SHIFT 0
#define NDP120_DNN_ISA_COMP1_OUTPUT_BASE_COORD_MSB_MASK 0x00000fff
#define NDP120_DNN_ISA_COMP1_OUTPUT_BASE_COORD_MSB(v) \
        ((v) << NDP120_DNN_ISA_COMP1_OUTPUT_BASE_COORD_MSB_SHIFT)
#define NDP120_DNN_ISA_COMP1_OUTPUT_BASE_COORD_MSB_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP1_OUTPUT_BASE_COORD_MSB_SHIFT))
#define NDP120_DNN_ISA_COMP1_OUTPUT_BASE_COORD_MSB_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP1_OUTPUT_BASE_COORD_MSB_MASK) | ((v) << NDP120_DNN_ISA_COMP1_OUTPUT_BASE_COORD_MSB_SHIFT))
#define NDP120_DNN_ISA_COMP1_OUTPUT_BASE_COORD_MSB_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP1_OUTPUT_BASE_COORD_MSB_MASK) >> NDP120_DNN_ISA_COMP1_OUTPUT_BASE_COORD_MSB_SHIFT)
#define NDP120_DNN_ISA_COMP1_WEIGHTS_BASE_ADDR_SHIFT 12
#define NDP120_DNN_ISA_COMP1_WEIGHTS_BASE_ADDR_MASK 0x03fff000
#define NDP120_DNN_ISA_COMP1_WEIGHTS_BASE_ADDR(v) \
        ((v) << NDP120_DNN_ISA_COMP1_WEIGHTS_BASE_ADDR_SHIFT)
#define NDP120_DNN_ISA_COMP1_WEIGHTS_BASE_ADDR_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP1_WEIGHTS_BASE_ADDR_SHIFT))
#define NDP120_DNN_ISA_COMP1_WEIGHTS_BASE_ADDR_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP1_WEIGHTS_BASE_ADDR_MASK) | ((v) << NDP120_DNN_ISA_COMP1_WEIGHTS_BASE_ADDR_SHIFT))
#define NDP120_DNN_ISA_COMP1_WEIGHTS_BASE_ADDR_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP1_WEIGHTS_BASE_ADDR_MASK) >> NDP120_DNN_ISA_COMP1_WEIGHTS_BASE_ADDR_SHIFT)
#define NDP120_DNN_ISA_COMP1_BIAS_BASE_ADDR_LSB_SHIFT 26
#define NDP120_DNN_ISA_COMP1_BIAS_BASE_ADDR_LSB_MASK 0xfc000000
#define NDP120_DNN_ISA_COMP1_BIAS_BASE_ADDR_LSB(v) \
        ((v) << NDP120_DNN_ISA_COMP1_BIAS_BASE_ADDR_LSB_SHIFT)
#define NDP120_DNN_ISA_COMP1_BIAS_BASE_ADDR_LSB_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP1_BIAS_BASE_ADDR_LSB_SHIFT))
#define NDP120_DNN_ISA_COMP1_BIAS_BASE_ADDR_LSB_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP1_BIAS_BASE_ADDR_LSB_MASK) | ((v) << NDP120_DNN_ISA_COMP1_BIAS_BASE_ADDR_LSB_SHIFT))
#define NDP120_DNN_ISA_COMP1_BIAS_BASE_ADDR_LSB_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP1_BIAS_BASE_ADDR_LSB_MASK) >> NDP120_DNN_ISA_COMP1_BIAS_BASE_ADDR_LSB_SHIFT)
/* register ndp120_dnn.isa.comp2 */
#define NDP120_DNN_ISA_COMP2 0x60080008
#define NDP120_DNN_ISA_COMP2_BIAS_BASE_ADDR_MSB_SHIFT 0
#define NDP120_DNN_ISA_COMP2_BIAS_BASE_ADDR_MSB_MASK 0x000000ff
#define NDP120_DNN_ISA_COMP2_BIAS_BASE_ADDR_MSB(v) \
        ((v) << NDP120_DNN_ISA_COMP2_BIAS_BASE_ADDR_MSB_SHIFT)
#define NDP120_DNN_ISA_COMP2_BIAS_BASE_ADDR_MSB_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP2_BIAS_BASE_ADDR_MSB_SHIFT))
#define NDP120_DNN_ISA_COMP2_BIAS_BASE_ADDR_MSB_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP2_BIAS_BASE_ADDR_MSB_MASK) | ((v) << NDP120_DNN_ISA_COMP2_BIAS_BASE_ADDR_MSB_SHIFT))
#define NDP120_DNN_ISA_COMP2_BIAS_BASE_ADDR_MSB_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP2_BIAS_BASE_ADDR_MSB_MASK) >> NDP120_DNN_ISA_COMP2_BIAS_BASE_ADDR_MSB_SHIFT)
#define NDP120_DNN_ISA_COMP2_SCALER_BASE_ADDR_SHIFT 8
#define NDP120_DNN_ISA_COMP2_SCALER_BASE_ADDR_MASK 0x003fff00
#define NDP120_DNN_ISA_COMP2_SCALER_BASE_ADDR(v) \
        ((v) << NDP120_DNN_ISA_COMP2_SCALER_BASE_ADDR_SHIFT)
#define NDP120_DNN_ISA_COMP2_SCALER_BASE_ADDR_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP2_SCALER_BASE_ADDR_SHIFT))
#define NDP120_DNN_ISA_COMP2_SCALER_BASE_ADDR_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP2_SCALER_BASE_ADDR_MASK) | ((v) << NDP120_DNN_ISA_COMP2_SCALER_BASE_ADDR_SHIFT))
#define NDP120_DNN_ISA_COMP2_SCALER_BASE_ADDR_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP2_SCALER_BASE_ADDR_MASK) >> NDP120_DNN_ISA_COMP2_SCALER_BASE_ADDR_SHIFT)
#define NDP120_DNN_ISA_COMP2_INPUT_SIZE0_LSB_SHIFT 22
#define NDP120_DNN_ISA_COMP2_INPUT_SIZE0_LSB_MASK 0xffc00000
#define NDP120_DNN_ISA_COMP2_INPUT_SIZE0_LSB(v) \
        ((v) << NDP120_DNN_ISA_COMP2_INPUT_SIZE0_LSB_SHIFT)
#define NDP120_DNN_ISA_COMP2_INPUT_SIZE0_LSB_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP2_INPUT_SIZE0_LSB_SHIFT))
#define NDP120_DNN_ISA_COMP2_INPUT_SIZE0_LSB_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP2_INPUT_SIZE0_LSB_MASK) | ((v) << NDP120_DNN_ISA_COMP2_INPUT_SIZE0_LSB_SHIFT))
#define NDP120_DNN_ISA_COMP2_INPUT_SIZE0_LSB_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP2_INPUT_SIZE0_LSB_MASK) >> NDP120_DNN_ISA_COMP2_INPUT_SIZE0_LSB_SHIFT)
/* register ndp120_dnn.isa.comp3 */
#define NDP120_DNN_ISA_COMP3 0x6008000c
#define NDP120_DNN_ISA_COMP3_INPUT_SIZE0_MSB_SHIFT 0
#define NDP120_DNN_ISA_COMP3_INPUT_SIZE0_MSB_MASK 0x00000003
#define NDP120_DNN_ISA_COMP3_INPUT_SIZE0_MSB(v) \
        ((v) << NDP120_DNN_ISA_COMP3_INPUT_SIZE0_MSB_SHIFT)
#define NDP120_DNN_ISA_COMP3_INPUT_SIZE0_MSB_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP3_INPUT_SIZE0_MSB_SHIFT))
#define NDP120_DNN_ISA_COMP3_INPUT_SIZE0_MSB_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP3_INPUT_SIZE0_MSB_MASK) | ((v) << NDP120_DNN_ISA_COMP3_INPUT_SIZE0_MSB_SHIFT))
#define NDP120_DNN_ISA_COMP3_INPUT_SIZE0_MSB_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP3_INPUT_SIZE0_MSB_MASK) >> NDP120_DNN_ISA_COMP3_INPUT_SIZE0_MSB_SHIFT)
#define NDP120_DNN_ISA_COMP3_INPUT_SIZE1_SHIFT 2
#define NDP120_DNN_ISA_COMP3_INPUT_SIZE1_MASK 0x00003ffc
#define NDP120_DNN_ISA_COMP3_INPUT_SIZE1(v) \
        ((v) << NDP120_DNN_ISA_COMP3_INPUT_SIZE1_SHIFT)
#define NDP120_DNN_ISA_COMP3_INPUT_SIZE1_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP3_INPUT_SIZE1_SHIFT))
#define NDP120_DNN_ISA_COMP3_INPUT_SIZE1_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP3_INPUT_SIZE1_MASK) | ((v) << NDP120_DNN_ISA_COMP3_INPUT_SIZE1_SHIFT))
#define NDP120_DNN_ISA_COMP3_INPUT_SIZE1_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP3_INPUT_SIZE1_MASK) >> NDP120_DNN_ISA_COMP3_INPUT_SIZE1_SHIFT)
#define NDP120_DNN_ISA_COMP3_INPUT_SIZE2_SHIFT 14
#define NDP120_DNN_ISA_COMP3_INPUT_SIZE2_MASK 0x03ffc000
#define NDP120_DNN_ISA_COMP3_INPUT_SIZE2(v) \
        ((v) << NDP120_DNN_ISA_COMP3_INPUT_SIZE2_SHIFT)
#define NDP120_DNN_ISA_COMP3_INPUT_SIZE2_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP3_INPUT_SIZE2_SHIFT))
#define NDP120_DNN_ISA_COMP3_INPUT_SIZE2_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP3_INPUT_SIZE2_MASK) | ((v) << NDP120_DNN_ISA_COMP3_INPUT_SIZE2_SHIFT))
#define NDP120_DNN_ISA_COMP3_INPUT_SIZE2_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP3_INPUT_SIZE2_MASK) >> NDP120_DNN_ISA_COMP3_INPUT_SIZE2_SHIFT)
#define NDP120_DNN_ISA_COMP3_OUTPUT_SIZE0_LSB_SHIFT 26
#define NDP120_DNN_ISA_COMP3_OUTPUT_SIZE0_LSB_MASK 0xfc000000
#define NDP120_DNN_ISA_COMP3_OUTPUT_SIZE0_LSB(v) \
        ((v) << NDP120_DNN_ISA_COMP3_OUTPUT_SIZE0_LSB_SHIFT)
#define NDP120_DNN_ISA_COMP3_OUTPUT_SIZE0_LSB_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP3_OUTPUT_SIZE0_LSB_SHIFT))
#define NDP120_DNN_ISA_COMP3_OUTPUT_SIZE0_LSB_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP3_OUTPUT_SIZE0_LSB_MASK) | ((v) << NDP120_DNN_ISA_COMP3_OUTPUT_SIZE0_LSB_SHIFT))
#define NDP120_DNN_ISA_COMP3_OUTPUT_SIZE0_LSB_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP3_OUTPUT_SIZE0_LSB_MASK) >> NDP120_DNN_ISA_COMP3_OUTPUT_SIZE0_LSB_SHIFT)
/* register ndp120_dnn.isa.comp4 */
#define NDP120_DNN_ISA_COMP4 0x60080010
#define NDP120_DNN_ISA_COMP4_OUTPUT_SIZE0_MSB_SHIFT 0
#define NDP120_DNN_ISA_COMP4_OUTPUT_SIZE0_MSB_MASK 0x0000003f
#define NDP120_DNN_ISA_COMP4_OUTPUT_SIZE0_MSB(v) \
        ((v) << NDP120_DNN_ISA_COMP4_OUTPUT_SIZE0_MSB_SHIFT)
#define NDP120_DNN_ISA_COMP4_OUTPUT_SIZE0_MSB_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP4_OUTPUT_SIZE0_MSB_SHIFT))
#define NDP120_DNN_ISA_COMP4_OUTPUT_SIZE0_MSB_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP4_OUTPUT_SIZE0_MSB_MASK) | ((v) << NDP120_DNN_ISA_COMP4_OUTPUT_SIZE0_MSB_SHIFT))
#define NDP120_DNN_ISA_COMP4_OUTPUT_SIZE0_MSB_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP4_OUTPUT_SIZE0_MSB_MASK) >> NDP120_DNN_ISA_COMP4_OUTPUT_SIZE0_MSB_SHIFT)
#define NDP120_DNN_ISA_COMP4_OUTPUT_SIZE1_SHIFT 6
#define NDP120_DNN_ISA_COMP4_OUTPUT_SIZE1_MASK 0x0003ffc0
#define NDP120_DNN_ISA_COMP4_OUTPUT_SIZE1(v) \
        ((v) << NDP120_DNN_ISA_COMP4_OUTPUT_SIZE1_SHIFT)
#define NDP120_DNN_ISA_COMP4_OUTPUT_SIZE1_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP4_OUTPUT_SIZE1_SHIFT))
#define NDP120_DNN_ISA_COMP4_OUTPUT_SIZE1_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP4_OUTPUT_SIZE1_MASK) | ((v) << NDP120_DNN_ISA_COMP4_OUTPUT_SIZE1_SHIFT))
#define NDP120_DNN_ISA_COMP4_OUTPUT_SIZE1_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP4_OUTPUT_SIZE1_MASK) >> NDP120_DNN_ISA_COMP4_OUTPUT_SIZE1_SHIFT)
#define NDP120_DNN_ISA_COMP4_OUTPUT_SIZE2_SHIFT 18
#define NDP120_DNN_ISA_COMP4_OUTPUT_SIZE2_MASK 0x3ffc0000
#define NDP120_DNN_ISA_COMP4_OUTPUT_SIZE2(v) \
        ((v) << NDP120_DNN_ISA_COMP4_OUTPUT_SIZE2_SHIFT)
#define NDP120_DNN_ISA_COMP4_OUTPUT_SIZE2_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP4_OUTPUT_SIZE2_SHIFT))
#define NDP120_DNN_ISA_COMP4_OUTPUT_SIZE2_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP4_OUTPUT_SIZE2_MASK) | ((v) << NDP120_DNN_ISA_COMP4_OUTPUT_SIZE2_SHIFT))
#define NDP120_DNN_ISA_COMP4_OUTPUT_SIZE2_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP4_OUTPUT_SIZE2_MASK) >> NDP120_DNN_ISA_COMP4_OUTPUT_SIZE2_SHIFT)
#define NDP120_DNN_ISA_COMP4_INPUT_OFFSET_LSB_SHIFT 30
#define NDP120_DNN_ISA_COMP4_INPUT_OFFSET_LSB_MASK 0xc0000000
#define NDP120_DNN_ISA_COMP4_INPUT_OFFSET_LSB(v) \
        ((v) << NDP120_DNN_ISA_COMP4_INPUT_OFFSET_LSB_SHIFT)
#define NDP120_DNN_ISA_COMP4_INPUT_OFFSET_LSB_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP4_INPUT_OFFSET_LSB_SHIFT))
#define NDP120_DNN_ISA_COMP4_INPUT_OFFSET_LSB_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP4_INPUT_OFFSET_LSB_MASK) | ((v) << NDP120_DNN_ISA_COMP4_INPUT_OFFSET_LSB_SHIFT))
#define NDP120_DNN_ISA_COMP4_INPUT_OFFSET_LSB_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP4_INPUT_OFFSET_LSB_MASK) >> NDP120_DNN_ISA_COMP4_INPUT_OFFSET_LSB_SHIFT)
/* register ndp120_dnn.isa.comp5 */
#define NDP120_DNN_ISA_COMP5 0x60080014
#define NDP120_DNN_ISA_COMP5_INPUT_OFFSET_MSB_SHIFT 0
#define NDP120_DNN_ISA_COMP5_INPUT_OFFSET_MSB_MASK 0x000003ff
#define NDP120_DNN_ISA_COMP5_INPUT_OFFSET_MSB(v) \
        ((v) << NDP120_DNN_ISA_COMP5_INPUT_OFFSET_MSB_SHIFT)
#define NDP120_DNN_ISA_COMP5_INPUT_OFFSET_MSB_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP5_INPUT_OFFSET_MSB_SHIFT))
#define NDP120_DNN_ISA_COMP5_INPUT_OFFSET_MSB_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP5_INPUT_OFFSET_MSB_MASK) | ((v) << NDP120_DNN_ISA_COMP5_INPUT_OFFSET_MSB_SHIFT))
#define NDP120_DNN_ISA_COMP5_INPUT_OFFSET_MSB_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP5_INPUT_OFFSET_MSB_MASK) >> NDP120_DNN_ISA_COMP5_INPUT_OFFSET_MSB_SHIFT)
#define NDP120_DNN_ISA_COMP5_NUM_FILTERS_SHIFT 10
#define NDP120_DNN_ISA_COMP5_NUM_FILTERS_MASK 0x003ffc00
#define NDP120_DNN_ISA_COMP5_NUM_FILTERS(v) \
        ((v) << NDP120_DNN_ISA_COMP5_NUM_FILTERS_SHIFT)
#define NDP120_DNN_ISA_COMP5_NUM_FILTERS_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP5_NUM_FILTERS_SHIFT))
#define NDP120_DNN_ISA_COMP5_NUM_FILTERS_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP5_NUM_FILTERS_MASK) | ((v) << NDP120_DNN_ISA_COMP5_NUM_FILTERS_SHIFT))
#define NDP120_DNN_ISA_COMP5_NUM_FILTERS_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP5_NUM_FILTERS_MASK) >> NDP120_DNN_ISA_COMP5_NUM_FILTERS_SHIFT)
#define NDP120_DNN_ISA_COMP5_KERNEL_POOLING0_SHIFT 22
#define NDP120_DNN_ISA_COMP5_KERNEL_POOLING0_MASK 0x0fc00000
#define NDP120_DNN_ISA_COMP5_KERNEL_POOLING0(v) \
        ((v) << NDP120_DNN_ISA_COMP5_KERNEL_POOLING0_SHIFT)
#define NDP120_DNN_ISA_COMP5_KERNEL_POOLING0_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP5_KERNEL_POOLING0_SHIFT))
#define NDP120_DNN_ISA_COMP5_KERNEL_POOLING0_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP5_KERNEL_POOLING0_MASK) | ((v) << NDP120_DNN_ISA_COMP5_KERNEL_POOLING0_SHIFT))
#define NDP120_DNN_ISA_COMP5_KERNEL_POOLING0_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP5_KERNEL_POOLING0_MASK) >> NDP120_DNN_ISA_COMP5_KERNEL_POOLING0_SHIFT)
#define NDP120_DNN_ISA_COMP5_KERNEL_POOLING1_LSB_SHIFT 28
#define NDP120_DNN_ISA_COMP5_KERNEL_POOLING1_LSB_MASK 0xf0000000
#define NDP120_DNN_ISA_COMP5_KERNEL_POOLING1_LSB(v) \
        ((v) << NDP120_DNN_ISA_COMP5_KERNEL_POOLING1_LSB_SHIFT)
#define NDP120_DNN_ISA_COMP5_KERNEL_POOLING1_LSB_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP5_KERNEL_POOLING1_LSB_SHIFT))
#define NDP120_DNN_ISA_COMP5_KERNEL_POOLING1_LSB_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP5_KERNEL_POOLING1_LSB_MASK) | ((v) << NDP120_DNN_ISA_COMP5_KERNEL_POOLING1_LSB_SHIFT))
#define NDP120_DNN_ISA_COMP5_KERNEL_POOLING1_LSB_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP5_KERNEL_POOLING1_LSB_MASK) >> NDP120_DNN_ISA_COMP5_KERNEL_POOLING1_LSB_SHIFT)
/* register ndp120_dnn.isa.comp6 */
#define NDP120_DNN_ISA_COMP6 0x60080018
#define NDP120_DNN_ISA_COMP6_KERNEL_POOLING1_MSB_SHIFT 0
#define NDP120_DNN_ISA_COMP6_KERNEL_POOLING1_MSB_MASK 0x00000003
#define NDP120_DNN_ISA_COMP6_KERNEL_POOLING1_MSB(v) \
        ((v) << NDP120_DNN_ISA_COMP6_KERNEL_POOLING1_MSB_SHIFT)
#define NDP120_DNN_ISA_COMP6_KERNEL_POOLING1_MSB_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP6_KERNEL_POOLING1_MSB_SHIFT))
#define NDP120_DNN_ISA_COMP6_KERNEL_POOLING1_MSB_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP6_KERNEL_POOLING1_MSB_MASK) | ((v) << NDP120_DNN_ISA_COMP6_KERNEL_POOLING1_MSB_SHIFT))
#define NDP120_DNN_ISA_COMP6_KERNEL_POOLING1_MSB_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP6_KERNEL_POOLING1_MSB_MASK) >> NDP120_DNN_ISA_COMP6_KERNEL_POOLING1_MSB_SHIFT)
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE0_SHIFT 2
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE0_MASK 0x000000fc
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE0(v) \
        ((v) << NDP120_DNN_ISA_COMP6_PADDING_SIZE0_SHIFT)
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE0_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP6_PADDING_SIZE0_SHIFT))
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE0_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP6_PADDING_SIZE0_MASK) | ((v) << NDP120_DNN_ISA_COMP6_PADDING_SIZE0_SHIFT))
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE0_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP6_PADDING_SIZE0_MASK) >> NDP120_DNN_ISA_COMP6_PADDING_SIZE0_SHIFT)
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE1_SHIFT 8
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE1_MASK 0x00003f00
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE1(v) \
        ((v) << NDP120_DNN_ISA_COMP6_PADDING_SIZE1_SHIFT)
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE1_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP6_PADDING_SIZE1_SHIFT))
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE1_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP6_PADDING_SIZE1_MASK) | ((v) << NDP120_DNN_ISA_COMP6_PADDING_SIZE1_SHIFT))
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE1_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP6_PADDING_SIZE1_MASK) >> NDP120_DNN_ISA_COMP6_PADDING_SIZE1_SHIFT)
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE2_SHIFT 14
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE2_MASK 0x000fc000
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE2(v) \
        ((v) << NDP120_DNN_ISA_COMP6_PADDING_SIZE2_SHIFT)
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE2_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP6_PADDING_SIZE2_SHIFT))
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE2_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP6_PADDING_SIZE2_MASK) | ((v) << NDP120_DNN_ISA_COMP6_PADDING_SIZE2_SHIFT))
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE2_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP6_PADDING_SIZE2_MASK) >> NDP120_DNN_ISA_COMP6_PADDING_SIZE2_SHIFT)
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE3_SHIFT 20
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE3_MASK 0x03f00000
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE3(v) \
        ((v) << NDP120_DNN_ISA_COMP6_PADDING_SIZE3_SHIFT)
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE3_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP6_PADDING_SIZE3_SHIFT))
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE3_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP6_PADDING_SIZE3_MASK) | ((v) << NDP120_DNN_ISA_COMP6_PADDING_SIZE3_SHIFT))
#define NDP120_DNN_ISA_COMP6_PADDING_SIZE3_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP6_PADDING_SIZE3_MASK) >> NDP120_DNN_ISA_COMP6_PADDING_SIZE3_SHIFT)
#define NDP120_DNN_ISA_COMP6_WEIGHTS_WIDTH_SHIFT 26
#define NDP120_DNN_ISA_COMP6_WEIGHTS_WIDTH_MASK 0x1c000000
#define NDP120_DNN_ISA_COMP6_WEIGHTS_WIDTH(v) \
        ((v) << NDP120_DNN_ISA_COMP6_WEIGHTS_WIDTH_SHIFT)
#define NDP120_DNN_ISA_COMP6_WEIGHTS_WIDTH_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP6_WEIGHTS_WIDTH_SHIFT))
#define NDP120_DNN_ISA_COMP6_WEIGHTS_WIDTH_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP6_WEIGHTS_WIDTH_MASK) | ((v) << NDP120_DNN_ISA_COMP6_WEIGHTS_WIDTH_SHIFT))
#define NDP120_DNN_ISA_COMP6_WEIGHTS_WIDTH_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP6_WEIGHTS_WIDTH_MASK) >> NDP120_DNN_ISA_COMP6_WEIGHTS_WIDTH_SHIFT)
#define NDP120_DNN_ISA_COMP6_SELECT_STRIDE_DILATION_SHIFT 29
#define NDP120_DNN_ISA_COMP6_SELECT_STRIDE_DILATION_MASK 0x20000000
#define NDP120_DNN_ISA_COMP6_SELECT_STRIDE_DILATION(v) \
        ((v) << NDP120_DNN_ISA_COMP6_SELECT_STRIDE_DILATION_SHIFT)
#define NDP120_DNN_ISA_COMP6_SELECT_STRIDE_DILATION_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP6_SELECT_STRIDE_DILATION_SHIFT))
#define NDP120_DNN_ISA_COMP6_SELECT_STRIDE_DILATION_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP6_SELECT_STRIDE_DILATION_MASK) | ((v) << NDP120_DNN_ISA_COMP6_SELECT_STRIDE_DILATION_SHIFT))
#define NDP120_DNN_ISA_COMP6_SELECT_STRIDE_DILATION_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP6_SELECT_STRIDE_DILATION_MASK) >> NDP120_DNN_ISA_COMP6_SELECT_STRIDE_DILATION_SHIFT)
#define NDP120_DNN_ISA_COMP6_STRIDE_DILATION0_LSB_SHIFT 30
#define NDP120_DNN_ISA_COMP6_STRIDE_DILATION0_LSB_MASK 0xc0000000
#define NDP120_DNN_ISA_COMP6_STRIDE_DILATION0_LSB(v) \
        ((v) << NDP120_DNN_ISA_COMP6_STRIDE_DILATION0_LSB_SHIFT)
#define NDP120_DNN_ISA_COMP6_STRIDE_DILATION0_LSB_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP6_STRIDE_DILATION0_LSB_SHIFT))
#define NDP120_DNN_ISA_COMP6_STRIDE_DILATION0_LSB_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP6_STRIDE_DILATION0_LSB_MASK) | ((v) << NDP120_DNN_ISA_COMP6_STRIDE_DILATION0_LSB_SHIFT))
#define NDP120_DNN_ISA_COMP6_STRIDE_DILATION0_LSB_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP6_STRIDE_DILATION0_LSB_MASK) >> NDP120_DNN_ISA_COMP6_STRIDE_DILATION0_LSB_SHIFT)
/* register ndp120_dnn.isa.comp7 */
#define NDP120_DNN_ISA_COMP7 0x6008001c
#define NDP120_DNN_ISA_COMP7_STRIDE_DILATION0_MSB_SHIFT 0
#define NDP120_DNN_ISA_COMP7_STRIDE_DILATION0_MSB_MASK 0x0000000f
#define NDP120_DNN_ISA_COMP7_STRIDE_DILATION0_MSB(v) \
        ((v) << NDP120_DNN_ISA_COMP7_STRIDE_DILATION0_MSB_SHIFT)
#define NDP120_DNN_ISA_COMP7_STRIDE_DILATION0_MSB_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP7_STRIDE_DILATION0_MSB_SHIFT))
#define NDP120_DNN_ISA_COMP7_STRIDE_DILATION0_MSB_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP7_STRIDE_DILATION0_MSB_MASK) | ((v) << NDP120_DNN_ISA_COMP7_STRIDE_DILATION0_MSB_SHIFT))
#define NDP120_DNN_ISA_COMP7_STRIDE_DILATION0_MSB_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP7_STRIDE_DILATION0_MSB_MASK) >> NDP120_DNN_ISA_COMP7_STRIDE_DILATION0_MSB_SHIFT)
#define NDP120_DNN_ISA_COMP7_STRIDE_DILATION1_SHIFT 4
#define NDP120_DNN_ISA_COMP7_STRIDE_DILATION1_MASK 0x000003f0
#define NDP120_DNN_ISA_COMP7_STRIDE_DILATION1(v) \
        ((v) << NDP120_DNN_ISA_COMP7_STRIDE_DILATION1_SHIFT)
#define NDP120_DNN_ISA_COMP7_STRIDE_DILATION1_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP7_STRIDE_DILATION1_SHIFT))
#define NDP120_DNN_ISA_COMP7_STRIDE_DILATION1_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP7_STRIDE_DILATION1_MASK) | ((v) << NDP120_DNN_ISA_COMP7_STRIDE_DILATION1_SHIFT))
#define NDP120_DNN_ISA_COMP7_STRIDE_DILATION1_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP7_STRIDE_DILATION1_MASK) >> NDP120_DNN_ISA_COMP7_STRIDE_DILATION1_SHIFT)
#define NDP120_DNN_ISA_COMP7_STOP_SHIFT 10
#define NDP120_DNN_ISA_COMP7_STOP_MASK 0x00000400
#define NDP120_DNN_ISA_COMP7_STOP(v) \
        ((v) << NDP120_DNN_ISA_COMP7_STOP_SHIFT)
#define NDP120_DNN_ISA_COMP7_STOP_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP7_STOP_SHIFT))
#define NDP120_DNN_ISA_COMP7_STOP_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP7_STOP_MASK) | ((v) << NDP120_DNN_ISA_COMP7_STOP_SHIFT))
#define NDP120_DNN_ISA_COMP7_STOP_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP7_STOP_MASK) >> NDP120_DNN_ISA_COMP7_STOP_SHIFT)
#define NDP120_DNN_ISA_COMP7_INTERRUPT_SHIFT 11
#define NDP120_DNN_ISA_COMP7_INTERRUPT_MASK 0x0007f800
#define NDP120_DNN_ISA_COMP7_INTERRUPT(v) \
        ((v) << NDP120_DNN_ISA_COMP7_INTERRUPT_SHIFT)
#define NDP120_DNN_ISA_COMP7_INTERRUPT_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP7_INTERRUPT_SHIFT))
#define NDP120_DNN_ISA_COMP7_INTERRUPT_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP7_INTERRUPT_MASK) | ((v) << NDP120_DNN_ISA_COMP7_INTERRUPT_SHIFT))
#define NDP120_DNN_ISA_COMP7_INTERRUPT_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP7_INTERRUPT_MASK) >> NDP120_DNN_ISA_COMP7_INTERRUPT_SHIFT)
#define NDP120_DNN_ISA_COMP7_POINTER_SHIFT 19
#define NDP120_DNN_ISA_COMP7_POINTER_MASK 0x07f80000
#define NDP120_DNN_ISA_COMP7_POINTER(v) \
        ((v) << NDP120_DNN_ISA_COMP7_POINTER_SHIFT)
#define NDP120_DNN_ISA_COMP7_POINTER_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP7_POINTER_SHIFT))
#define NDP120_DNN_ISA_COMP7_POINTER_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP7_POINTER_MASK) | ((v) << NDP120_DNN_ISA_COMP7_POINTER_SHIFT))
#define NDP120_DNN_ISA_COMP7_POINTER_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP7_POINTER_MASK) >> NDP120_DNN_ISA_COMP7_POINTER_SHIFT)
#define NDP120_DNN_ISA_COMP7_SIGNATURE_SHIFT 27
#define NDP120_DNN_ISA_COMP7_SIGNATURE_MASK 0xf8000000
#define NDP120_DNN_ISA_COMP7_SIGNATURE(v) \
        ((v) << NDP120_DNN_ISA_COMP7_SIGNATURE_SHIFT)
#define NDP120_DNN_ISA_COMP7_SIGNATURE_INSERT(x, v) \
        ((x) | ((v) << NDP120_DNN_ISA_COMP7_SIGNATURE_SHIFT))
#define NDP120_DNN_ISA_COMP7_SIGNATURE_MASK_INSERT(x, v) \
        (((x) & ~NDP120_DNN_ISA_COMP7_SIGNATURE_MASK) | ((v) << NDP120_DNN_ISA_COMP7_SIGNATURE_SHIFT))
#define NDP120_DNN_ISA_COMP7_SIGNATURE_EXTRACT(x) \
        (((x) & NDP120_DNN_ISA_COMP7_SIGNATURE_MASK) >> NDP120_DNN_ISA_COMP7_SIGNATURE_SHIFT)

#endif