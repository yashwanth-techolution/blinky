#ifndef STM32_WAKE_WORD_MODULE_H_
#define STM32_WAKE_WORD_MODULE_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <stdbool.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/flash.h>

#include "Device_Status_Module.h"

/* STM sleep Mode parameters */
#define STM32_RUN_MODE         true   // Active low GPIO pin requires true.
#define STM32_SLEEP_MODE      false  // Active low GPIO pin requires false.
extern bool stm32_interrupt_flag;

/* STM SPI parameters */
#define STM32_READ_CMD         0x80
#define STM32_WRITE_CMD        0x00
#define MAX_SPI_BUFF_LEN      32768
#define AUDIO_BUFF_LENGTH     32000
#define ACK_RX_CMP             0x01
#define ACK_TX_CMP             0x02
#define CMD_BYTE                  5
#define ACK_BYTE                  3

/* STM Hardware pins */
#define STM32_EN                 31

extern uint16_t stm_wake_word_count;

extern bool ota_updating;
extern bool OTA_update_pending;
extern bool check_for_ota_update;
extern bool download_the_ota_update;
extern bool send_and_flash_ota_update;
extern bool dump_audio_file_to_server;
extern uint16_t ww_from_server;

struct stm_ota_update_status_t
{
    bool is_stm_ota_update_available;
    bool is_stm_update_downloaded;
    bool is_stm_update_send;
    char stm_fw_ver[15];
    char stm_old_fw_ver[15];
    char stm_audio_model_ver[15];
    char stm_motion_model_ver[15];
};

enum{
		STM_SOFT_RESTART = 1,
		STM_OLD_FW,
		STM_NEW_FW,
		STM_UPD_FW,
		STM_ERR_FW,
		STM_UNKNOWN_FW,
		HEALTH_CHECK,
		HEALTH_CHECK_MAIN_APP,
		HEALTH_CHECK_BOOT_APP,
		GET_VERSION_NUMBERS,
		HEX_FILE_TRANSFER_START,
		HEX_FILE_TRANSFER_CONTINUE,
		HEX_FILE_TRANSFER_DONE,
		HEX_FILE_TRANSFER_STARTED,
		HEX_FILE_RECEIVED_OK,
		HEX_FILE_RECEIVED_GARBAGE,
		HEX_FILE_RECEIVED_CHECK_SUM_ERROR,
		HEX_FILE_FOUND,
		HEX_FILE_FLASHING,
		HEX_FILE_FLASH_DONE,
		HEX_FILE_FLASH_FAILED,

		AUDIO_DETECTION_WW = 30,
		AUDIO_SPI_DATA,
		COMMAND_SPI_DATA,
		ULTRASONIC_SPI_DATA,
	};

enum{
      UPDATE_FOUND = 1,
      UPDATE_DOWNLOADED,
      UPDATE_SEND_TO_STM,
      UPDATE_FLASHED_INTO_STM
};


enum
{                                             // added enum for ota update status to edge device 
  DOWNLOAD_STARTED= 1,
  DOWNLOAD_PROGRESS,
  DOWNLOAD_COMPLETED,
  DOWNLOAD_FAILED_DUE_TO_WIFI,
  ERROR_IN_DOWNLOAD_HEX_FILE,
  DOWNLOAD_INTERRUPTED,

  INSTALLATION_STARTED,
  INSTALLATION_PROGRESS,
  INSTALLATION_COMPLETED,
  INSTALLATION_FAILED,
  
  UNKNOWN_ERROR,
  OTA_SUCCESS, 
  OTA_FAILED
};



#define STM32_GPIO_INRPT_NODE	DT_ALIAS(stm_interpt)
#if DT_NODE_HAS_STATUS(STM32_GPIO_INRPT_NODE, okay)
#define STM32_INRPT_GPIO_LABEL	DT_GPIO_LABEL(STM32_GPIO_INRPT_NODE, gpios)
#define STM32_INRPT_GPIO_PIN	DT_GPIO_PIN(STM32_GPIO_INRPT_NODE, gpios)
#define STM32_INRPT_GPIO_FLAGS	(GPIO_INPUT | DT_GPIO_FLAGS(STM32_GPIO_INRPT_NODE, gpios))
#else
#error "Unsupported board: sw0 devicetree alias is not defined"
#define SW0_GPIO_LABEL	""
#define SW0_GPIO_PIN	0
#define SW0_GPIO_FLAGS	0
#endif



int8_t Send_Data_to_STM32(uint8_t *out, uint16_t length);
int8_t Send_CMD_to_STM32(uint8_t message_type, uint16_t length, uint8_t read_write);
int8_t Read_from_STM32(uint8_t *in, uint16_t *read_data_lenght);
int8_t Send_ACK_to_STM32();
void init_STM32H7();
uint8_t check_stm32_ww();
uint8_t stm32_audio_files_read();
uint8_t stm32_version_numbers_read();
int8_t STM32_sleep_control(bool on);

/* added for downloading STM Hex file */
bool Check_stm_OTA_update_availble();
int8_t Download_stm_hex_file();
int8_t validate_stm_new_hex_file_chunk(char *hex_buffer, uint16_t size);
int8_t validate_stm_new_hex_file();
int8_t send_nex_hex_file_to_stm32();
int8_t flash_nex_hex_file_to_stm32();
int8_t store_stm32_hex_file(char *hex_buffer, uint16_t size);
int8_t store_stm_ota_update_status(uint8_t update_stage, bool state);
int8_t read_stm32_hex_file(uint8_t *hex_buffer);

void stm_old_new_fw_version_to_edge_device(uint8_t *stm_old_new_ver_numbers);


#endif  // STM32_WAKE_WORD_MODULE_H_
