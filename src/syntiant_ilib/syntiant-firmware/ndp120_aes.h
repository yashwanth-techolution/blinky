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

#ifndef __NDP120_AES_H__
#define __NDP120_AES_H__


#define MAX_WAIT_LOOP   0x10000
#define AES_BLOCK_LEN   16


/**
 * @brief struct of arguments required to 
 * for AES decryption 
 */
struct aes_ctl_block_s {
    uint8_t   iv[16];
    uint8_t   otp_key[16];
    uint8_t   auth_tag[16];
    uint8_t   ver_key[16];
    uint32_t  actual_fw_len;
    void     *fw_begin;
    uint8_t   key_sel;
    uint8_t   reserved[3];
    uint8_t  *ciphertext;
    uint32_t  ciphertextlen;
    uint8_t  *plaintext;
    uint32_t  plaintextlen;
} aes_ctl_block_s;

extern struct aes_ctl_block_s aesv2_control_block ;

/**
 * @brief Authenticaion the decrypted plain text
 *
 * @param pAes point to AES control block
 *
 */
uint32_t authentication_tag(struct aes_ctl_block_s *pAes);

uint32_t finalize_ccm128( uint8_t* iv_cbc,  uint8_t* ctr_tag);


/**
 * @brief decrypt one AES block
 *
 * @param pAes point to AES control block 
 * @param inBuf input crypted text- 16 bytes 
 * @param outBuf output plain text- 16 bytes 
 *
 */
void decrypt_one_block(struct aes_ctl_block_s *pAes,
                       uint8_t *inBuf,
                       uint8_t *outBuf);

/**
 * @brief read OTP
 *
 * @param wordaddr OTP memory offset 
 * @param data buffer to store the read value
 *
 */
int otp_readword(int wordaddr, uint32_t *data);


/**
 * @brief Init the AES Control block
 *
 * @param psynpkg_hdr AES control block data from packager
 *
 */
int aes_control_block_init(struct aes_v2_synpkg_hdr_s *psynpkg_hdr);

#endif
