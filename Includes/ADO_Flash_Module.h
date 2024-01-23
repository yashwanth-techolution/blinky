#ifndef ADO_FLASH_MODULE_H_
#define ADO_FLASH_MODULE_H_


#define AUDIO_FILE_FLASH_LENGTH          32768


#define MAX_FLASH_ERASE_LEN                0x20000  // 128kb max erase block
#define FLASH_AUDIO_ADDRESS_INCREMENT       0x8000  // added by Uttam
#define FLASH_TEST_REGION_OFFSET            0x0000
//#define FLASH_AUDIO_DATA_REGION_MAX     0x400000  // 4Mb of memory location is dedicated to store the audio files
#define FLASH_AUDIO_DATA_REGION_MAX      0x4000000  // 64Mb of memory location is dedicated to store the audio files
#define FLASH_AUDIO_DATA_START_REGION      0x20000


#define STM32_FIRMWARE_MAX_SIZE                0x400000  // 4Mb size
#define STM32_HEX_FILE_START_ADDRESS          0x4400000  // starts from 64Mb location.
#define STM32_FILES_VERSION_STORE_ADDRESS     0x4800000  // starts from 64Mb location.


/* Function declarations */
void spi2_init(void);
void external_flash_init();
static uint32_t nrf_spi_transfer_own(uint8_t* p_tx_buffer,uint8_t tx_buffer_length, uint8_t* p_rx_buffer,uint8_t rx_buffer_length);

int8_t external_flash_block_erase(uint32_t start_add, uint32_t no_of_blocks);
int8_t external_flash_file_write(uint32_t start_add, uint8_t* write_buf, uint32_t write_len);
int8_t external_flash_file_read(uint32_t start_add, uint8_t* read_buf, uint32_t read_len);

#endif  // ADO_FLASH_MODULE_H_