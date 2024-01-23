
/*
 * SYNTIANT CONFIDENTIAL
 *
 * _____________________
 *
 * Copyright (c) 2017-2020 Syntiant Corporation
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
#ifndef _NDP120_PKG_H_
#define _NDP120_PKG_H_


enum ndp_constants_e {
    AES_BLOCK_LEN = 16,
    PACKAGE_MAGIC_VALUE = 0x53bde5a1,
    CRC_POLYNOMIAL = 0xEDB88320,
    CRC_START = 0xFFFFFFFF,
    FIRMWARE_MAX_SIZE   = 48*1024,
    DSP_FW_MAX_SIZE     = 96*1024,
    MULTI_SEG_BIT_MASK  = 0x80000000,
};


/**
 * @brief struct of AES information data from .synpkg file
 */
typedef struct aes_v2_synpkg_hdr_s {
    uint8_t key_sel;
    uint8_t nonce[11];
    uint32_t actual_fw_len;
    uint8_t ver_key[16];
    uint8_t auth_tag[16];
    uint8_t otp_key[16];
} AES_Synpkg_V2_Header_T;


/**
 * multiple segments in one TLV
 */
struct segment {
	uint32_t address; /* address in MCU or DSP memory */
	uint32_t size;    /* length of the payload */
};

#define SEGMENT_HEADER_SIZE 8

struct ndp_pkg_state_s {
    void*       ptr;
    void*       open_ram_begin;
    void*       open_ram_end;
    uint32_t    u32tag;
    uint32_t    u32length;
    uint32_t    exp_crc;
    uint32_t    calc_crc;
    uint32_t    expected_bytes;
    uint32_t    mode;

    uint8_t     magic_header_found;
    uint8_t     metadata_complete;
    uint8_t     ph_header_found;
    uint8_t     not_use;

    /* temporary use buffer, mainly to load PH tage from host
     * before parsing
     */
    void*       temp_buffer_ptr;
    uint32_t    temp_buffer_offset;

    uint32_t    loaded_type;
    uint32_t    buffered;
    uint8_t     buffer[16];

    /* for multi segment load */
    uint32_t    cur_seg_address;
    uint32_t    cur_seg_size;
    uint8_t     seg_header_found;

    /* if segment header is located at the open ram boundary
     * copy the first part to seg_buffer[]. 
     * copy the follow headers when the open ram has updated data 
     */
    uint8_t     seg_buffered;
    uint8_t     reserved[2]; /* for alignment */
    uint8_t     seg_buffer[16];

    /* for debug */
    uint32_t log_start_addr;
    uint32_t log_end_addr;
    void    *log_buf_begin;
    void    *log_buf_end;
};


enum ndp_pkg_mode_e {
    PACKAGE_MODE_DONE = 0x0,
    PACKAGE_MODE_TAG_START = 0x1,
    PACKAGE_MODE_TAG_CONT = 0x2,
    PACKAGE_MODE_LENGTH_START = 0x3,
    PACKAGE_MODE_LENGTH_CONT = 0x4,
    PACKAGE_MODE_VALUE_START = 0x5,
    PACKAGE_MODE_VALUE_CONT = 0x6
};

enum ndp_pkg_loaded_e {
    LOADED_MAGIC            = (1<<0),
    LOADED_TAG_VERSION      = (1<<1),
    LOADED_DSP_FW           = (1<<2),
    LOADED_NN_PARAMETERS_V1 = (1<<3),
    LOADED_NN_PARAMETERS_V4 = (1<<4),

};

enum ndp_pkg_constants_e {
    PACKAGE_TAG_MAGIC = 0x1,
    PACKAGE_TAG_VERSION = 0x2,
    PACKAGE_TAG_SENTINEL = 0x4,
    TAG_NN_VERSION_STRING_V1 = 5,
    TAG_NN_LABELS_V1 = 10,
    TAG_PACKAGE_VERSION_STRING = 19,
    TAG_NN_PH_PARAMETERS_V4 = 23,
    TAG_NDP120_NN_PARAMETERS_V1 = 0x20,
    PACKAGE_TAG_ARMV6M_V2 = 0x22,
    PACKAGE_TAG_ARMV6M_V1_AES_CCM = 0xE,
    PACKAGE_TAG_ARMV6M_V2_AES_CCM = 0x23,
    PACKAGE_TAG_DSP_FW            = 40, //0x28
    PACKAGE_TAG_DSP_FW_ENCRYPTED  = 41, // 0x29
};


struct handshake_ram_T {
    uint32_t secure_ram_start;
    uint32_t secure_ram_end;
    uint32_t error_code;
    uint32_t reserved1;
    uint32_t reserved2[4];
} ;
extern struct handshake_ram_T *handshake_ram;


/**
 * @brief NDP packager download state init
 *
 * @param pstate point to pkg relative database
 * @param pkg_ram_begin open_ram start address
 * @param pkg_ram_end open_ram end address 
 * @param fw_ram_begin targer_ram start address, not use for 
 *                     MTLV cases
 * @param fw_ram_end  targer_ram end address, not use for MTLV 
 *                     cases
 */
void
ndp_pkg_state_init(
    struct ndp_pkg_state_s* pstate, 
    void* pkg_ram_begin,
    void* pkg_ram_end, 
    void* fw_ram_begin, 
    void* fw_ram_end);


/**
 * @brief reset the open ram processing pointer, mainly use when 
 *        get new block from host or from serial flash
 *
 * @param pstate point to pkg relative database
 */
void
ndp_pkg_state_reset_ptr(
    struct ndp_pkg_state_s* pstate);


/**
 * @brief reset the pkg database, mainly use when get NOP from 
 *        host or detect any errors
 *
 * @param pstate point to pkg relative database
 */
void
ndp_pkg_state_reset(
    struct ndp_pkg_state_s* pstate);


/**
 * @brief check the header for magic number. 
 *
 * @param pstate point to pkg relative database
 */
int
ndp_pkg_check_header(
    struct ndp_pkg_state_s* pstate);


/**
 * @brief read one TLV fields from open_ram, calcualte CRC at 
 *        the same time
 *
 * @param pstate point to pkg relative database 
 * @param dest point destion buffer
 * @param max_length length for the expected data
 * @param action_type process the data or just skip
 * @param calc_crc enable calcualte CRC
 */
#define ACTION_TYPE_PROCESS         0
#define ACTION_TYPE_SKIP            1
#define ACTION_TYPE_MTLV            2
#define ACTION_TYPE_MTLV_AES_HDR    3
#define ACTION_TYPE_MTLV_AES        4
int
ndp_pkg_read(struct ndp_pkg_state_s* pstate,
             void* dest, uint32_t max_length,
             int action_type, 
             int calc_crc);


/**
 * @brief Process new VALUE 
 *
 * @param pstate point to pkg relative database 
 */
int
ndp_pkg_parse_new_data(
    struct ndp_pkg_state_s * pstate);

/**
 * @brief Process unknow tag, the unknow here means unknow to 
 *        this firmware. Possible this tag use for other
 *        purpose.
 *
 * @param pstate point to pkg relative database 
 */
int
ndp_pkg_parse_unknown_tlv(
    struct ndp_pkg_state_s * pstate);

/**
 * @brief Process CRC tag 
 *
 * @param pstate point to pkg relative database 
 */
int
ndp_pkg_parse_sentinel_tlv(
    struct ndp_pkg_state_s * pstate);


/**
 * @brief clear memory
 *
 * @param ptr point to buffer to be cleared
 * @param len length of buffer
 */
void
ndp_clear_memory(uint8_t *ptr, int len);
#endif
