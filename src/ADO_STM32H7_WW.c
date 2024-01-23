#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <errno.h>
#include <hal/nrf_gpio.h>
#include <inttypes.h>
#include <zephyr/kernel.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "ESP8266_WiFi_lib/WiFiSpi.h"
#include "Includes/ADO_Audio_files.h"
#include "Includes/ADO_PIR_Module.h"
#include "Includes/ADO_STM32H7_WW.h"
#include "Includes/Command_Manager_Module.h"
#include "Includes/EEPROM_Module.h"
#include "Includes/Intent_Module.h"
#include "Includes/SPI_Module.h"

#include "Includes/ADO_Flash_Module.h"
#include "Includes/ADO_WiFi_Module.h"
#include "Includes/Display_Module.h"

struct stm_ota_update_status_t stm_ota_update_status;

static struct gpio_callback stm32_inrpt_cb_data;
const struct device *STM32_interrupt_pin;
uint16_t ww_from_server = 0;

uint8_t spi_read_buffer[MAX_SPI_BUFF_LEN];
uint16_t receive_bytes = MAX_SPI_BUFF_LEN;
uint8_t set_ack_flag = 0;

bool stm32_interrupt_flag = false;


// OTA update flags
bool ota_updating = false;
bool OTA_update_pending = false;
bool check_for_ota_update = false;
bool download_the_ota_update = false;
bool send_and_flash_ota_update = false;
bool dump_audio_file_to_server = false;

uint16_t stm_wake_word_count = 0;
uint8_t stm32_present_audio_file_read_count = 0;

uint32_t store_stm32_hex_file_address = STM32_HEX_FILE_START_ADDRESS;

int8_t read_stm32_hex_file(uint8_t *hex_buffer);

int8_t Read_from_STM32(uint8_t *in, uint16_t *read_data_lenght) {
  int err = 0;
  uint16_t buffer_rx_length_stm = 0; // buffer length stm wants to send
  uint8_t rx_cmd_buffer[CMD_BYTE] = {0};

  err = stm_spi_read(rx_cmd_buffer, CMD_BYTE);
  buffer_rx_length_stm = (((rx_cmd_buffer[1] & 0x7F) << 8) | rx_cmd_buffer[0]);
  *read_data_lenght = buffer_rx_length_stm;
  printk("buffer length = %d read data from STM\n", buffer_rx_length_stm);

  if (buffer_rx_length_stm) {
    err = stm_spi_read(in, buffer_rx_length_stm);
  }
  return err;
}

int8_t Send_ACK_to_STM32() {
  int err = 0;
  uint8_t rx_ack_buff[ACK_BYTE];

  rx_ack_buff[0] = 0;
  rx_ack_buff[1] = 0;
  rx_ack_buff[2] = ACK_RX_CMP;
  err = stm_spi_send(rx_ack_buff, ACK_BYTE);
  return err;
}

int8_t Send_Data_to_STM32(uint8_t *out, uint16_t length) {
  int err;
  err = stm_spi_send(out, length);
  return err;
}

int8_t Send_CMD_to_STM32(uint8_t message_type, uint16_t length, uint8_t read_write) {
  int err;
  uint8_t data_chunk = 0;
  uint8_t chunk_num = 0;
  uint8_t tx_cmd_buffer[CMD_BYTE]; // command buffer

  tx_cmd_buffer[0] = (length & 0xFF);
  tx_cmd_buffer[1] = ((length >> 8) & 0xFF);
  tx_cmd_buffer[1] = ((tx_cmd_buffer[1] & 0x7F) | read_write);
  tx_cmd_buffer[2] = data_chunk;
  tx_cmd_buffer[3] = chunk_num;
  tx_cmd_buffer[4] = message_type;

  err = stm_spi_send(tx_cmd_buffer, CMD_BYTE);
  return err;
}

/*
   brief: It is a callback function triggered on an stm32 GPIO interrupt,
          when there is a wake word detected.
*/
void STM32_wake_word_detected(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
  // Debug print
  printk("STM32 interrupt Detected........................\n");
  stm32_interrupt_flag = true;
  stop_wifi_send = true; // uncommented this line
  //new_wifi_cred = false; // uncommented this line
  is_bot_staginated2 = false;
}

/* This function sets up the STM32 interrupt pin for GPIO, configures the
interrupt to trigger on a rising edge, associates a callback function with
the interrupt, and provides error handling in case any of the configurations fail.
*/

int8_t interrupt_gpio_stm32() {
  int ret;
  STM32_interrupt_pin = device_get_binding(STM32_INRPT_GPIO_LABEL);

  if (STM32_interrupt_pin == NULL) {
    printk("Error: didn't find %s device\n", STM32_INRPT_GPIO_LABEL);
    return -1;
  }

  ret = gpio_pin_configure(STM32_interrupt_pin, STM32_INRPT_GPIO_PIN, STM32_INRPT_GPIO_FLAGS);
  if (ret != 0) {
    printk("Error %d: failed to configure %s pin %d\n",
        ret, STM32_INRPT_GPIO_LABEL, STM32_INRPT_GPIO_PIN);
    return -2;
  }

  ret = gpio_pin_interrupt_configure(STM32_interrupt_pin, STM32_INRPT_GPIO_PIN, GPIO_INT_EDGE_RISING);
  if (ret != 0) {
    printk("Error %d: failed to configure interrupt on %s pin %d\n",
        ret, STM32_INRPT_GPIO_LABEL, STM32_INRPT_GPIO_PIN);
    return -3;
  }

  gpio_init_callback(&stm32_inrpt_cb_data, STM32_wake_word_detected, BIT(STM32_INRPT_GPIO_PIN));
  gpio_add_callback(STM32_interrupt_pin, &stm32_inrpt_cb_data);
  printk("Set up button at %s pin %d\n", STM32_INRPT_GPIO_LABEL, STM32_INRPT_GPIO_PIN);
  return 0;
}

/* This function initializes the STM32H7 microcontroller by configuring GPIO
pins for SPI chip select and enable signals, sets up an interrupt for a GPIO
pin, reads data from flash memory, and performs additional operations on the
retrieved data.
*/

void init_STM32H7() {
  nrf_gpio_cfg_output(STM32_SPI_CS);
  nrf_gpio_pin_set(STM32_SPI_CS);
  nrf_gpio_cfg_output(STM32_EN);
  nrf_gpio_pin_set(STM32_EN);
  // nrf_gpio_pin_clear(STM32_EN);

  interrupt_gpio_stm32();
  int8_t error = 0;

  error = external_flash_file_read(STM32_FILES_VERSION_STORE_ADDRESS, (uint8_t *)&stm_ota_update_status, sizeof(stm_ota_update_status));
  if (error) {
    printk("Error reading file from flash memory\n");
  }

  printk("-----------------------------------------------------\n");
  download_the_ota_update = stm_ota_update_status.is_stm_ota_update_available;
  send_and_flash_ota_update = stm_ota_update_status.is_stm_update_downloaded;
  if (download_the_ota_update && send_and_flash_ota_update) {
    download_the_ota_update = false;
    send_and_flash_ota_update = false;
  }
  printk("stm fw version = %s\n", stm_ota_update_status.stm_fw_ver);
  printk("-----------------------------------------------------\n");
}

/* This function is checking for wake words or specific patterns in the data
received from an STM32 microcontroller.
*/

uint8_t check_stm32_ww() {
  uint8_t in = 0;
  uint16_t received_data_length;
  // uint32_t start_time, end_time, exec_time;

  for (int i = 0; i < 5; i++) {
    if (stm32_interrupt_flag == true) {
      printk("######################################\n");
      stm32_interrupt_flag = false;
      printk("-----reading-----\n");
      Read_from_STM32(spi_read_buffer, &received_data_length);
      printk("----completed----\n");
      k_msleep(270); // wait until STM32 stores audio file into it's external flash.

      if (received_data_length == 2) {
        stm32_present_audio_file_read_count = spi_read_buffer[1];
        printk("--- %d files stored in STM\n", stm32_present_audio_file_read_count);
        printk(" Response = %x  \n", spi_read_buffer[0]); // Check response
        not_confident_sent = false;

        switch (spi_read_buffer[0]) {
        case 0x09:
        if(run_in_intent_mode){
          printk("Match -> Door_Open_LT\n");
          return NON_WAKE_WORD;
          }
          else printk(".");
          break;

        case 0x0A:
          if(run_in_intent_mode) {
          printk("Match -> Door_Open\n");
          return DOOR_OPEN;
          }
          else printk(".");
          break;

        case 0x0B:
          printk("Match -> Door_Close\n");
          return DOOR_CLOSE;
          break;

        case 0x0C:
          printk("Match -> Door_Stop\n");
          chg_display_page(DISP_DOOR_STOP);
          return DOOR_STOP;
          break;

        case 0x0D:
          printk("Match -> Door_Stop_LT\n");
          return NON_WAKE_WORD;
          break;

        case 0x0E:
          printk("Match -> Door_Close_LT\n");
          return NON_WAKE_WORD;
          break;

        default:
          break;
        }
      } else if (received_data_length == 32001) {
        printk(" Response = %x  \n", spi_read_buffer[0]); // Response 0x0F is comming properly.

        if (spi_read_buffer[0] == 0x0F) {
          printk("Match -> NOT_CONFIDENT\n");

          //if ((disp_status != DISP_DOOR_OPENING_PAGE) && (disp_status != DISP_DOOR_CLOSING_PAGE)) {
          //   chg_display_page(DISP_PLEASE_SAY_AGAIN);
          //}

          // start_time = k_cycle_get_32();
          // printk("%d\n", spi_read_buffer[0]);

          //Store Audio file in flash.

          //Store_audio_file(&spi_read_buffer[1], stm_wake_word_count);  //Uttam added for testing
          //printk("\n--------------------------- Store Not Confident file in Nordic Flash -------------------\n");
          //Store_not_confident_audio_file(&spi_read_buffer[1], stm_wake_word_count);
          //stored_audio_files_count();  //Print no of Stored Audio File.

          stop_wifi_send = false;
          int ret = Send_Raw_Audio_File_To_Server(&spi_read_buffer[1], received_data_length - 1, false);

          if (ret != SERVER_CONNECTION_FAILED) {

            if (stm32_interrupt_flag == true) {
              printf("\n----------------------- Break Not_Confident loop \n");
              continue;
            }

            printf("\n ===> Read not_confident response");
            int ret = Get_data_from_auto_ai(false);

            if (ww_from_server) {
              not_confident_sent = false;
              printf("\n===> Read not_confident ww_from_server %d \n", ww_from_server);
              break;
              //  return ww_from_server;
            }
          } else {
            ret = Send_Raw_Audio_File_To_Server(&spi_read_buffer[1], received_data_length - 1, false);

            if (ret != SERVER_CONNECTION_FAILED) {
              printf("\n ===> Read not_confident response");
              int ret = Get_data_from_auto_ai(false);

              if (ww_from_server) {
                not_confident_sent = false;
                printf("\n===> Read not_confident ww_from_server %d \n", ww_from_server);
                //   return ww_from_server;
                break;
              }
            } else {
              printk("Server retry also failed");
            }
          }

          // printf("\n===> Read not_confident ww_from_server %d \n",ww_from_server);
          // end_time = k_cycle_get_32();

          // Calculate the execution time in microseconds
          // exec_time = ((end_time - start_time) / (32768 / 1000));
          // printk("\nTotal Processing time = %d ms\n", exec_time);

          // for(int i=0;i<32000;i++)
          // {
          //   printk("%d ",spi_read_buffer[i+1]);
          // }
        }
      }
    }
    k_msleep(100U);
  }
  return in;
}

uint8_t stm32_audio_files_read() {
  uint8_t in = 0;
  uint16_t received_data_length;
  uint8_t audio_fil_count = 0;

  if (stm32_interrupt_flag == true) {
    return in;
  }

  while ((stm32_present_audio_file_read_count >= 1) && (stm32_present_audio_file_read_count <= 32)) {
    Send_CMD_to_STM32(AUDIO_SPI_DATA, 0, STM32_READ_CMD);
    k_msleep(200);

    if (stm32_interrupt_flag == true) {
      printk("stm intrupt detected\n");
      return in;
    }

    memset(spi_read_buffer, '\0', MAX_SPI_BUFF_LEN);
    printk("+++++reading+++++\n");
    Read_from_STM32(spi_read_buffer, &received_data_length);
    k_msleep(100);
    Send_ACK_to_STM32();
    printk("++++completed++++\n");

    stm32_present_audio_file_read_count = spi_read_buffer[0];
    audio_fil_count = stm32_present_audio_file_read_count;
    printk("-----Remainig audio files in STM32 = %d\n", stm32_present_audio_file_read_count);

    if ((stm32_present_audio_file_read_count < 1) || (received_data_length == 0)) {
      return 0;
    }

    if (received_data_length >= 32000) {
      switch (spi_read_buffer[1]) {
      case 0x09:
        printk("Read audio file Door_Open_LT --- %d\n", ++stm_wake_word_count);
        Store_audio_file(&spi_read_buffer[1], stm_wake_word_count);
        break;

      case 0x0A:
        printk("Read audio file Door_Open   --- %d\n", ++stm_wake_word_count);
        Store_audio_file(&spi_read_buffer[1], stm_wake_word_count);
        break;

      case 0x0B:
        printk("Read audio file Door_Close  --- %d\n", ++stm_wake_word_count);
        Store_audio_file(&spi_read_buffer[1], stm_wake_word_count);
        break;

      case 0x0C:
        printk("Read audio file Door_stop  --- %d\n", ++stm_wake_word_count);
        Store_audio_file(&spi_read_buffer[1], stm_wake_word_count);
        break;

      case 0x0D:
        printk("Read audio file Door_stop_LT  --- %d\n", ++stm_wake_word_count);
        Store_audio_file(&spi_read_buffer[1], stm_wake_word_count);
        break;

      case 0x0E:
        printk("Read audio file Door_close_LT  --- %d\n", ++stm_wake_word_count);
        Store_audio_file(&spi_read_buffer[1], stm_wake_word_count);
        break;

      default:
        break;
      }
      printk("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
    }
    stm32_present_audio_file_read_count = audio_fil_count;
  }
  return in;
}

uint8_t stm32_version_numbers_read() {
  uint16_t received_data_length;
  char *version;

  if (stm32_interrupt_flag == true) {
    return 0;
  }

  Send_CMD_to_STM32(GET_VERSION_NUMBERS, 0, STM32_READ_CMD);
  k_msleep(200);

  if (stm32_interrupt_flag == true) {
    return 0;
  }

  Read_from_STM32(spi_read_buffer, &received_data_length);
  k_msleep(100);
  Send_ACK_to_STM32();

  if (received_data_length >= 8) {
    version = &spi_read_buffer[0];
    printk("Version numbers are = %s\n", version);
  }
  return 0;
}

/*
 * brief: This function puts stm32 into sleep mode based on called argument command.
 */
int8_t STM32_sleep_control(bool on) {
  // Set the stm32 sleep GPIO
  if ((STM32_RUN_MODE) == on) {
    nrf_gpio_pin_clear(STM32_EN);
  } else {
    nrf_gpio_pin_set(STM32_EN);
  }
  return 0;
}

/* -------------------------- ALL functions related to OTA update added here --------------------- */

/* Added for downloading STM Hex file */
bool Check_stm_OTA_update_availble() {
  int8_t wifi_status = 0;
  bool ret = false;

  // stm_ota_update_status.stm_fw_ver="v0.0.0";    //added for  testing

  if (!wifi_module_active) {
    printk("WiFi module is off.\nTurning it on...\n");
    WiFi_module_on_off(WIFI_MODULE_ON);
    k_msleep(1500);
  }

  wifi_status = ADO_WiFi_status(NULL);

  if (WL_CONNECTED != wifi_status) {
    printk("WiFi connection status = %d\nTrying second time\n", wifi_status);
    wifi_status = WiFi_Network_Connect(NULL);

    if (WL_CONNECTED != wifi_status) {
      printk("check your wifi connection\n");
      return false;
    }
  }
  printk("\n--------------------------------------------------------------\n");
  printk("Current fw version = %s\n", stm_ota_update_status.stm_fw_ver);
  stm_ota_update_status.stm_fw_ver[14] = 0;
  strcpy(stm_ota_update_status.stm_old_fw_ver, stm_ota_update_status.stm_fw_ver); // ----------------added by ashok

  ret = request_and_cmp_stm_fw_version(stm_ota_update_status.stm_fw_ver, NULL);

  printk("OTA fw version = %s\n", stm_ota_update_status.stm_fw_ver);
  printk("\n-------------------------------------------------------------\n");
  return ret;
}

int8_t Download_stm_hex_file() {
  int8_t wifi_status = -1;

  if (!wifi_module_active) {
    printk("WiFi module is off for downloading the file.\nTurning it on...\n");
    WiFi_module_on_off(WIFI_MODULE_ON);
    k_msleep(1500);
  }
  wifi_status = ADO_WiFi_status(NULL);

  if (WL_CONNECTED != wifi_status) {
    printk("WiFi connection status = %d\nTrying second time\n", wifi_status);
    wifi_status = WiFi_Network_Connect(NULL);

    if (WL_CONNECTED != wifi_status) {
      printk("Check your wifi connection\n");
      return wifi_status;
    }
  }

  store_stm32_hex_file_address = STM32_HEX_FILE_START_ADDRESS;
  wifi_status = request_and_download_stm_hex_file_from_server(NULL);
  return wifi_status;
}

int8_t validate_stm_new_hex_file_chunk(char *hex_buffer, uint16_t size) {
  printk("Checking HEX file Chunk\n");

  hex_buffer[size + 2] = '\0';
  // printk("%s\n", hex_buffer+2);

  for (int i = 0; i < size; i++) {
    if ((hex_buffer[i + 2] == '\r') || (hex_buffer[i + 2] == '\n')) {
      continue;
    } else if ((hex_buffer[i + 2] >= 'A') && (hex_buffer[i + 2] <= 'F')) {
      continue;
    } else if ((hex_buffer[i + 2] < '0') || (hex_buffer[i + 2] > ':')) {
      printk("\n Error idex id = %d, %X\n", i, (uint8_t)hex_buffer[i + 2]);
      return -1;
    }
  }
  return 0;
}

int8_t validate_stm_new_hex_file() {
  uint16_t data_size = 0;
  store_stm32_hex_file_address = STM32_HEX_FILE_START_ADDRESS;
  printk("Checking HEX file\n");

  while (1) {
    memset(body, '\0', 32769);
    read_stm32_hex_file((uint8_t *)&body[0]);
    data_size = (uint16_t)((uint8_t)body[0] | ((uint8_t)body[1] << 8));

    body[data_size + 2] = '\0';
    // printk("%s\n", body+2);

    for (int i = 0; i < data_size; i++) {
      // printk("%c", body[i+2]);
      if ((body[i + 2] == '\r') || (body[i + 2] == '\n')) {
        continue;
      } else if ((body[i + 2] >= 'A') && (body[i + 2] <= 'F')) {
        continue;
      } else if ((body[i + 2] < '0') || (body[i + 2] > ':')) {
        printk("\n Error idex id = %d, %X\n", i, (uint8_t)body[i + 2]);
        return -1;
      }
    }
    k_msleep(10);

    if (data_size < MAX_LEN) {
      break;
    }
  }
  return 0;
}

int8_t stm_hex_file_ack_receive(uint32_t timeout) {
  uint16_t received_data_length;
  unsigned long int time_stamp = k_uptime_get();

  while (stm32_interrupt_flag != true) {
    k_msleep(50);
    if ((k_uptime_get() - time_stamp) >= timeout) {
      return -1;
    }
    printk("-");
  }
  stm32_interrupt_flag = false;
  k_msleep(50);
  Read_from_STM32(spi_read_buffer, &received_data_length);
  printk("Reply got reading = %d, %d\n", received_data_length, spi_read_buffer[0]);

  if (received_data_length == 2) {
    return spi_read_buffer[0];
  }
  return -1;
}

int8_t send_nex_hex_file_to_stm32() {
  int i = 0;
  printk("Hex file sending to STM\n");
  // Send_CMD_to_STM32(STM_SOFT_RESTART, 0, STM32_WRITE_CMD);
  Send_CMD_to_STM32(STM_NEW_FW, 0, STM32_WRITE_CMD);
  k_msleep(400);

  Send_CMD_to_STM32(HEX_FILE_TRANSFER_START, 0, STM32_WRITE_CMD);
  k_msleep(50);

  if (stm_hex_file_ack_receive(5000) != HEX_FILE_TRANSFER_STARTED) {
    printk("\nxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx.\n");
    return -1;
  }

  uint16_t data_size = 0;
  store_stm32_hex_file_address = STM32_HEX_FILE_START_ADDRESS;

  while (1) {
    memset(body, '\0', 32769);
    read_stm32_hex_file((uint8_t *)&body[0]);
    data_size = (uint16_t)((uint8_t)body[0] | ((uint8_t)body[1] << 8));

    body[data_size + 2] = '\0';

    Send_CMD_to_STM32(HEX_FILE_TRANSFER_CONTINUE, (data_size + 2), STM32_WRITE_CMD);
    k_msleep(50);
    if (stm_hex_file_ack_receive(2000) != HEX_FILE_TRANSFER_CONTINUE) {
      printk("Hex file transfer failed in middle\n");
      return -1;
    }

    k_msleep(50);
    Send_Data_to_STM32((uint8_t *)body, (data_size + 2));
    printk("Buffer len = %d\n", (data_size + 2));
    // printk("%s\n", body+2);
    k_msleep(350);

    if (data_size < MAX_LEN) {
      k_msleep(50);
      Send_CMD_to_STM32(HEX_FILE_TRANSFER_DONE, 0, STM32_WRITE_CMD);
      k_msleep(50);

      if (stm_hex_file_ack_receive(2000) != HEX_FILE_TRANSFER_DONE) {
        printk("Hex file transfer failed in middle\n");
        return -1;
      }
      printk("Data send completed\n");

      if (stm_hex_file_ack_receive(5000) == HEX_FILE_RECEIVED_OK) {
        printk("Hex file transfered to stm okay.\n");
      } else {
        printk("Hex file transfered to stm failed.\n");
        return -1;
      }
      k_msleep(50);
      break;
    }
  }
  return 0;
}

int8_t flash_nex_hex_file_to_stm32() {
  k_msleep(1000);
  while (1) {
    switch (stm_hex_file_ack_receive(5000)) {
    case HEX_FILE_FOUND:
      printk("Hex file found okay in bootloader code.\n");
      break;

    case HEX_FILE_RECEIVED_GARBAGE:
      printk("Hex file garbage in bootloader code.\n");
      return -1;
      break;

    case HEX_FILE_FLASHING:
      printk("Hex file flashing in bootloader code.\n");
      break;

    case HEX_FILE_FLASH_DONE:
      printk("Hex file flash done in bootloader code.\n");
      return 0;
      break;

    default:
      break;
    }
  }
  return 0;
}

int8_t store_stm32_hex_file(char *hex_buffer, uint16_t size) {
  int8_t error = 0;
  uint16_t data_size = (uint16_t)(hex_buffer[0] | (hex_buffer[1] << 8));
  printk("hex buffer size = %d, %d, %d, %d\n", size, data_size,
      (uint8_t)hex_buffer[0], (uint8_t)hex_buffer[1]);

  if ((store_stm32_hex_file_address % MAX_FLASH_ERASE_LEN) == 0) {
    printk("erasing hole block\n");
    error = external_flash_block_erase(store_stm32_hex_file_address, 0);
    if (error) {
      printk("Error erasing flash memory location\n");
      return error;
    }
  }

  printk("\nstore stm32 hex file address = 0x%08X\n", store_stm32_hex_file_address);
  error = external_flash_file_write(store_stm32_hex_file_address, hex_buffer, (uint32_t)size);

  if (error) {
    printk("Error saving file into flash memory\n");
    return error;
  }

  store_stm32_hex_file_address = store_stm32_hex_file_address + FLASH_AUDIO_ADDRESS_INCREMENT;
  return 0;
}

int8_t store_stm_ota_update_status(uint8_t update_stage, bool state) {
  int8_t error = 0;
  printk("erasing hole block\n");
  error = external_flash_block_erase(STM32_FILES_VERSION_STORE_ADDRESS, 0);

  if (error) {
    printk("Error erasing flash memory location\n");
    return error;
  }

  switch (update_stage) {
  case UPDATE_FOUND:
    stm_ota_update_status.is_stm_ota_update_available = state;
    stm_ota_update_status.is_stm_update_downloaded = false;
    stm_ota_update_status.is_stm_update_send = false;
    if(!state)
    {
      printk("OTA fw version = %s\n", stm_ota_update_status.stm_fw_ver);
      stm_ota_update_status.stm_old_fw_ver[14] = 0;
      strcpy(stm_ota_update_status.stm_fw_ver, stm_ota_update_status.stm_old_fw_ver); // --added by ashok
      printk("Current fw version = %s\n", stm_ota_update_status.stm_fw_ver);
    }
    break;

  case UPDATE_DOWNLOADED:
    // NOTE: if state == false store old fw string back into external flash.
    if (!state) {
      printk("\n--------------------------------------------------------------\n");
      printk("OTA tried for 10 times and failed.\n");
      printk("OTA fw version = %s\n", stm_ota_update_status.stm_fw_ver);
      stm_ota_update_status.stm_old_fw_ver[14] = 0;
      strcpy(stm_ota_update_status.stm_fw_ver, stm_ota_update_status.stm_old_fw_ver); // --added by ashok
      printk("Current fw version = %s\n", stm_ota_update_status.stm_fw_ver);
      printk("-------------------------------------------------------------\n");
    }
    stm_ota_update_status.is_stm_ota_update_available = false;
    stm_ota_update_status.is_stm_update_downloaded = state;
    stm_ota_update_status.is_stm_update_send = false;
    break;

  case UPDATE_SEND_TO_STM:
    // NOTE: if state == false store old fw string back into external flash.
    if (!state) {
      printk("\n--------------------------------------------------------------\n");
      printk("OTA tried for 10 times and failed.\n");
      printk("OTA fw version = %s\n", stm_ota_update_status.stm_fw_ver);
      stm_ota_update_status.stm_old_fw_ver[14] = 0;
      strcpy(stm_ota_update_status.stm_fw_ver, stm_ota_update_status.stm_old_fw_ver); // --added by ashok
      printk("Current fw version = %s\n", stm_ota_update_status.stm_fw_ver);
      printk("-------------------------------------------------------------\n");
    }

    stm_ota_update_status.is_stm_ota_update_available = false;
    stm_ota_update_status.is_stm_update_downloaded = false;
    stm_ota_update_status.is_stm_update_send = state;
    break;

  case UPDATE_FLASHED_INTO_STM:
    break;

  default:
    break;
  }

  error = external_flash_file_write(STM32_FILES_VERSION_STORE_ADDRESS,
      (uint8_t *)&stm_ota_update_status, sizeof(stm_ota_update_status));
  if (error) {
    printk("Error saving file into flash memory\n");
    return error;
  }

  printk("after storing fw_version in qspi is  = %s", stm_ota_update_status.stm_fw_ver);
  return 0;
}

int8_t read_stm32_hex_file(uint8_t *hex_buffer) {
  int8_t error = 0;
  error = external_flash_file_read(store_stm32_hex_file_address,
      (uint8_t *)body, FLASH_AUDIO_ADDRESS_INCREMENT);
  
  if (error) {
    printk("Error reading file from flash memory\n");
    // return error;
  }
  store_stm32_hex_file_address = store_stm32_hex_file_address + FLASH_AUDIO_ADDRESS_INCREMENT;
  return 0;
}

void stm_old_new_fw_version_to_edge_device(uint8_t *stm_old_new_ver_numbers) {
  char *version_str = NULL;
  char var[3] = {0};
  int i = 0;
  version_str = stm_ota_update_status.stm_old_fw_ver;
  memset(var, 0, sizeof(var));
  version_str++;

  while (*version_str != '.') {
    var[i++] = *(version_str++);
  }

  stm_old_new_ver_numbers[0] = atoi(var);
  memset(var, 0, sizeof(var));
  i = 0;
  version_str++;

  while (*version_str != '.') {
    var[i++] = *(version_str++);
  }

  stm_old_new_ver_numbers[1] = atoi(var);
  memset(var, 0, sizeof(var));
  i = 0;
  version_str++;

  while (*version_str != '\0') {
    var[i++] = *(version_str++);
  }

  stm_old_new_ver_numbers[2] = atoi(var);
  version_str = stm_ota_update_status.stm_fw_ver;
  memset(var, 0, sizeof(var));
  i = 0;
  version_str++;

  while (*version_str != '.') {
    var[i++] = *(version_str++);
  }

  stm_old_new_ver_numbers[3] = atoi(var);
  memset(var, 0, sizeof(var));
  i = 0;
  version_str++;

  while (*version_str != '.') {
    var[i++] = *(version_str++);
  }

  stm_old_new_ver_numbers[4] = atoi(var);
  memset(var, 0, sizeof(var));
  i = 0;
  version_str++;

  while (*version_str != '\0') {
    var[i++] = *(version_str++);
  }
  stm_old_new_ver_numbers[5] = atoi(var);
  printk("new fw is %s and old fw is %s", stm_ota_update_status.stm_fw_ver,
      stm_ota_update_status.stm_old_fw_ver);
}