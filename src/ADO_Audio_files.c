#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <errno.h>
#include <hal/nrf_gpio.h>
#include <inttypes.h>
#include <zephyr/kernel.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "ESP8266_WiFi_lib/WiFiSpi.h"
#include "Includes/ADO_Audio_files.h"
#include "Includes/ADO_Flash_Module.h"
#include "Includes/ADO_STM32H7_WW.h"
#include "Includes/ADO_WiFi_Module.h"
#include "Includes/Command_Manager_Module.h"
#include "Includes/EEPROM_Module.h"
#include "Includes/Intent_Module.h"

#define DUMPING_SWITCH 1

bool is_update_to_auto_ai = false, not_confident_sent = false;

uint32_t erasable_block_number = 0;
uint8_t audio_file_retry_attempts = 0;
uint32_t stm_store_audio_file_address = FLASH_AUDIO_DATA_START_REGION;
uint32_t server_update_audio_file_address = FLASH_AUDIO_DATA_START_REGION;

audio_file_store_format_t audio_file;
audio_auto_ai_parameters_t audio_auto_ai_params;
extern audio_auto_ai_parameters_t audio_data_for_ai;

uint16_t stored_audio_files_count() {
  uint16_t count = 0;

  // stm_store_audio_file_address = server_update_audio_file_address;
  if (stm_store_audio_file_address >= server_update_audio_file_address) {
    count = (uint16_t)(((stm_store_audio_file_address - server_update_audio_file_address) / FLASH_AUDIO_ADDRESS_INCREMENT));
  } else {
    count = (uint16_t)(((FLASH_AUDIO_DATA_REGION_MAX - server_update_audio_file_address) / FLASH_AUDIO_ADDRESS_INCREMENT));
    count += (uint16_t)(stm_store_audio_file_address / FLASH_AUDIO_ADDRESS_INCREMENT);
  }
  printf("No.files stored = %d, Auto-AI send count = %d\n", count, START_AUTO_AI_SERVER);
  return count;
}

int8_t load_audio_file_address() {
  int8_t error = 0;
  uint8_t audio_file_address[12];

  error = external_flash_file_read(0x0000, audio_file_address, sizeof(audio_file_address));
  if(error)
  {
    printk("Error loading audio file address from flash memory\n");
    return error;
  }

  memcpy(&stm_store_audio_file_address, &audio_file_address[0], 4);
  memcpy(&server_update_audio_file_address, &audio_file_address[4], 4);
  memcpy(&stm_wake_word_count, &audio_file_address[8], 2);

  printk("------------------------------------------------------------------------------\n");
  printk("reading stm address = 0x%X\n", stm_store_audio_file_address);
  printk("server address = 0x%X\n", server_update_audio_file_address);
  printk("Wake Words count = %d\n", stm_wake_word_count);

  if((stm_store_audio_file_address >= FLASH_AUDIO_DATA_REGION_MAX) || (stm_store_audio_file_address <= FLASH_AUDIO_DATA_START_REGION))
  {
    stm_store_audio_file_address = FLASH_AUDIO_DATA_START_REGION;
    printf("New reading stm address = 0x%X\n", stm_store_audio_file_address);
  }

  if((server_update_audio_file_address >= FLASH_AUDIO_DATA_REGION_MAX) || (server_update_audio_file_address <= FLASH_AUDIO_DATA_START_REGION))
  {
    server_update_audio_file_address = FLASH_AUDIO_DATA_START_REGION;
    printf("New server address = 0x%X\n", server_update_audio_file_address);
  }

  stored_audio_files_count();
  printk("------------------------------------------------------------------------------\n");
  return 0;
}

int8_t save_audio_file_address() {
  int8_t error = 0;
  uint8_t audio_file_address[12] = {0};

  //printk("saving stm address = 0x%X,  server address = 0x%X\n", stm_store_audio_file_address, server_update_audio_file_address);
  memcpy(&audio_file_address[0], &stm_store_audio_file_address, 4);
  memcpy(&audio_file_address[4], &server_update_audio_file_address, 4);
  memcpy(&audio_file_address[8], &stm_wake_word_count, 2);

  error = external_flash_block_erase(0x0000, erasable_block_number);
  if(error)
  {
    printk("Error erasing flash memory location\n");
    return error;
  }

  error = external_flash_file_write(0x0000, audio_file_address, sizeof(audio_file_address));
  if(error)
  {
    printk("Error saving audio file address into flash memory\n");
    return error;
  }
  return 0;
}

/* Store only Not confident file to the flash memory of Nordic. */
int8_t Store_not_confident_audio_file(uint8_t *audio_buff, uint16_t wake_word_count) {
  int8_t error = 0;

  if (stm_store_audio_file_address >= (FLASH_AUDIO_DATA_REGION_MAX-FLASH_AUDIO_ADDRESS_INCREMENT)) {
    printk("\n-------- Reached to max memory limit of audio files while storing -------\n");
    stm_store_audio_file_address = FLASH_AUDIO_DATA_START_REGION;
  }

  if ((stm_store_audio_file_address % MAX_FLASH_ERASE_LEN) == 0) {
    printk("Erasing hole block\n");
    error = external_flash_block_erase(stm_store_audio_file_address, erasable_block_number);

    if (error) {
      printk("Error erasing flash memory location\n");
      return error;
    }
  }

  audio_file.lable = audio_buff[0];
  audio_file.no_of_classes = audio_buff[1];
  memcpy(&audio_file.confidence_val[0], &audio_buff[2], 40);

  // NOTE: added temp
  uint32_t file_num = 0;
  memcpy(&file_num, &audio_buff[38], 4);
  printk("File number = %d\n", file_num);
  audio_file.file_number = (uint16_t)(file_num);
  printk("File number = %d\n", audio_file.file_number);
  audio_file.no_of_classes -= 1;

  memcpy(&audio_file.file_header[0], 0, 44);
  memcpy(&audio_file.data[0], &audio_buff[42], 32000);

  audio_file.ver_str_len = audio_buff[32042];
  printf("Ver str len = %d\n", audio_buff[32042]);

  if (audio_buff[32042] >= 13) {
    audio_buff[32042] = 13;
  }

  memcpy(&audio_file.audio_model_ver[0], &audio_buff[32043], audio_buff[32042]);
  audio_file.audio_model_ver[audio_buff[32042] + 1] = '\0';
  printf("CF = %d\n", audio_buff[1]);

  printk("\nModel version  before writing into the flash = %s\n", audio_file.audio_model_ver);
  // printk("\nstm store file address = 0x%X\n", stm_store_audio_file_address);
  error = external_flash_file_write(stm_store_audio_file_address, (uint8_t *)&audio_file, sizeof(audio_file));

  if (error) {
    printk("Error saving not confident file into flash memory\n");
    return error;
  }

  stm_store_audio_file_address = stm_store_audio_file_address + FLASH_AUDIO_ADDRESS_INCREMENT;
  error = save_audio_file_address();

  if (error) {
    return error;
  }
  return 0;
}

int8_t Store_audio_file(uint8_t *audio_buff, uint16_t wake_word_count) {
  int8_t error = 0;
  if (stm_store_audio_file_address >= (FLASH_AUDIO_DATA_REGION_MAX-FLASH_AUDIO_ADDRESS_INCREMENT)) {
    printk("\n-------- Reached to max memory limit of audio files while storing -------\n");
    stm_store_audio_file_address = FLASH_AUDIO_DATA_START_REGION;
  }

  if ((stm_store_audio_file_address % MAX_FLASH_ERASE_LEN) == 0) {
    printk("erasing hole block\n");
    error = external_flash_block_erase(stm_store_audio_file_address, erasable_block_number);

    if (error) {
      printk("Error erasing flash memory location\n");
      return error;
    }
  }
#ifdef DUMPING_SWITCH
  if((stored_audio_files_count()>10)&&(!run_in_intent_mode))
    {
      current_power_mode = DUMPING;
      dump_audio_file_to_server = true;
    }
  else if((stored_audio_files_count()==0)||(run_in_intent_mode))
    {
      current_power_mode = DUMPING_STOP;
      dump_audio_file_to_server = false;
    }
#endif


  // audio_file.file_number = wake_word_count;
  audio_file.lable = audio_buff[0];
  printk("File Name: %d\n" , audio_buff[0]);
  audio_file.no_of_classes = audio_buff[1];
  memcpy(&audio_file.confidence_val[0], &audio_buff[2], 40);

  // NOTE: added temp
  uint32_t file_num = 0;
  // memcpy(&file_num, &audio_buff[2 + ((audio_file.no_of_classes - 1)*4)], 4);
  memcpy(&file_num, &audio_buff[38], 4);
  printk("file number = %d\n", file_num);
  audio_file.file_number = (uint16_t)(file_num);
  printk("file number = %d\n", audio_file.file_number);
  audio_file.no_of_classes -= 1;

  memcpy(&audio_file.file_header[0], 0, 44);
  memcpy(&audio_file.data[0], &audio_buff[42], 32000);

  audio_file.ver_str_len = audio_buff[32042];
  printf("Ver str len = %d\n", audio_buff[32042]);

  if (audio_buff[32042] >= 13) {
    audio_buff[32042] = 13;
  }

  memcpy(&audio_file.audio_model_ver[0], &audio_buff[32043], audio_buff[32042]);
  audio_file.audio_model_ver[audio_buff[32042] + 1] = '\0';
  printf("CF = %d\n", audio_buff[1]);

  for (int i = 0; i < audio_buff[1]; i++) {
    printf("%f, ", audio_file.confidence_val[i]);
  }
  printf("\n");

  printk("\nmodel version  before writing into the flash = %s\n", audio_file.audio_model_ver);
  // printk("\nstm store file address = 0x%X\n", stm_store_audio_file_address);
  error = external_flash_file_write(stm_store_audio_file_address, (uint8_t *)&audio_file, sizeof(audio_file));

  if (error) {
    printk("Error saving file into flash memory\n");
    return error;
  }

  stm_store_audio_file_address = stm_store_audio_file_address + FLASH_AUDIO_ADDRESS_INCREMENT;
  error = save_audio_file_address();

  if (error) {
    return error;
  }
  return 0;
}

int8_t Read_audio_file(audio_file_store_format_t *qspi_audio_file) {
  int8_t error = 0;

  if (stm_store_audio_file_address >= (FLASH_AUDIO_DATA_REGION_MAX-FLASH_AUDIO_ADDRESS_INCREMENT)) {
    printk("\n-------- Reached to max memory limit of audio files while reading -------\n");
    server_update_audio_file_address = FLASH_AUDIO_DATA_START_REGION;
  }
  audio_file.file_number = 0;
  memset(&audio_file.audio_model_ver[0], '\0', 6);
  error = external_flash_file_read(server_update_audio_file_address, (uint8_t *)&audio_file, sizeof(audio_file));

  if (error) {
    printk("Error reading file from flash memory\n");
    return error;
  }

  printk("audio file count = %d\n", audio_file.file_number);
  printk("model version = %s\n", audio_file.audio_model_ver);
  return 0;
}

int8_t Update_server_audio_file_address() {
  server_update_audio_file_address = server_update_audio_file_address + FLASH_AUDIO_ADDRESS_INCREMENT;
  return save_audio_file_address();
}

int8_t Check_Audio_files_update_status() {
  uint16_t audio_files_count = stored_audio_files_count();

  if (is_update_to_auto_ai == false) {
    if (audio_files_count < START_AUTO_AI_SERVER) {
      printk("Less Audio files to send Auto-AI\n");
      return 0;
    }

    is_update_to_auto_ai = true;
    WiFi_module_on_off(WIFI_MODULE_ON);
  } else {
    printk("Audio files left to send Auto-AI\n");
    if (!wifi_module_active) {
      printk("WiFi Module Enabled\n");
      WiFi_module_on_off(WIFI_MODULE_ON);
    }
  }
  return 0;
}

/* Send live Raw Audio file to server without store it in the local memory.
   This API will send the Audio file to Not_Confident server.
*/

int8_t Send_Raw_Audio_File_To_Server(uint8_t *audio_buff, uint16_t buff_len, bool *stop_flag) {
  // uint32_t start_time, middle_time1, middle_time2, end_time, exec_time; //For calculate all time
  // start_time = k_cycle_get_32();
  // printk("\nStart time of NOT_CONFIDENT= %d ms\n", start_time);

  // Define local variable.
  int8_t wifi_status = 0;
  // char body[32769] = {0};

  uint8_t wav_format_header_array[] = {0x52, 0x49, 0x46, 0x46, 0x24, 0x7D, 0x00,
      0x00, 0x57, 0x41, 0x56, 0x45, 0x66, 0x6D, 0x74, 0x20, 0x10, 0x00, 0x00, 0x00,
      0x01, 0x00, 0x01, 0x00, 0x80, 0x3E, 0x00, 0x00, 0x00, 0x7D, 0x00, 0x00, 0x02,
      0x00, 0x10, 0x00, 0x64, 0x61, 0x74, 0x61, 0x00, 0x7D, 0x00, 0x00};

  // Step 2: Check WiFi Status.....................................................................................
  wifi_status = ADO_WiFi_status(stop_flag);
  printk("\nWiFi Status =  %d", wifi_status);

  // Calculate the execution time in miliseconds++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // middle_time1 = k_cycle_get_32();
  // exec_time = ((middle_time1 - start_time) / (32768 / 1000));
  // printk("\n Time for Nordic to ESP = %d ms\n", exec_time);

  // Step 3: Check WiFi connection status..........................................................................
  if (WL_CONNECTED == wifi_status) {
    int k = 0;
    int ret = 0;
    int length = 0;
    char Slength[5];
    int size = 0;

    // Step 4: Prepare post http body..............................................................................
    IPAddress server = {34, 93, 212, 15};

    strcpy(audio_auto_ai_params.csv, "Door Close=00.000%<br>Door Open=0.000%<br>Door Stop=0.000%<br>Unknown=100.000%");
    audio_auto_ai_params.label = 1;
    strcpy(audio_auto_ai_params.tag, "v4_25_73");
    strcpy(audio_auto_ai_params.ww_confidence_score, "0");
    strcpy(audio_auto_ai_params.file_name, "EVO2CABINONE33701.wav");

    audio_auto_ai_params.params_str_length = (strlen(audio_auto_ai_params.ww_confidence_score) +
                                              strlen(audio_auto_ai_params.file_name) +
                                              strlen(audio_auto_ai_params.csv) +
                                              strlen(audio_auto_ai_params.label) +
                                              strlen(audio_auto_ai_params.tag));

    length = 996 + audio_auto_ai_params.params_str_length + buff_len + WAV_AUDIO_HEADER_LENGTH;
    printk("\nLength of total content = %d\n", length);

    itoa(length, Slength, 10);
    body[0] = 0;

    prepare_data("POST /aihelp HTTP/1.1\r\nHost: 34.93.212.15:5001\r\n"
                 "Content-Type: multipart/form-data; boundary=------------------------d74496d66958873e\r\n"
                 "Content-Length: ",
        &size);

    prepare_data(Slength, &size);

    prepare_data("\r\nExpect: 100-continue\r\n"
                 "\r\n--------------------------d74496d66958873e\r\n"      // 46
                 "Content-Disposition: form-data; name=\"status\"\r\n\r\n" // 49
                 "backlog"                                                 // 7
                 "\r\n--------------------------d74496d66958873e\r\n"      // 46
                 "Content-Disposition: form-data; name=\"csv\"\r\n\r\n",
        &size);
    prepare_data(audio_auto_ai_params.csv, &size);

    prepare_data("\r\n--------------------------d74496d66958873e\r\n"     // 46
                 "Content-Disposition: form-data; name=\"model\"\r\n\r\n" // 48
                 "61ef9d4324e24310d102c5c7"                               // 24
                 "\r\n--------------------------d74496d66958873e\r\n"     // 46
                 "Content-Disposition: form-data; name=\"label\"\r\n\r\n",
        &size);
    prepare_data(audio_auto_ai_params.label, &size);

    prepare_data("\r\n--------------------------d74496d66958873e\r\n" // 46
                 "Content-Disposition: form-data; name=\"tag\"\r\n\r\n",
        &size); // 46
    prepare_data(audio_auto_ai_params.tag, &size);

    prepare_data("\r\n--------------------------d74496d66958873e\r\n" // 46
                 "Content-Disposition: form-data; name=\"confidence_score\"\r\n\r\n",
        &size); // 59
    prepare_data(audio_auto_ai_params.ww_confidence_score, &size);

    prepare_data("\r\n--------------------------d74496d66958873e\r\n"          // 46
                 "Content-Disposition: form-data; name=\"prediction\"\r\n\r\n" // 53
                 "predicted"                                                   // 9
                 "\r\n--------------------------d74496d66958873e\r\n"          // 46
                 "Content-Disposition: form-data; name=\"model_type\"\r\n\r\n" // 53
                 "audio"                                                       // 5
                 "\r\n--------------------------d74496d66958873e\r\n"          // 46
                 "Content-Disposition: form-data; name=\"resource\"; filename=\"",
        &size); // 59
    prepare_data(audio_auto_ai_params.file_name, &size);
    prepare_data("\"\r\nContent-Type: audio/wav\r\n\r\n", &size); // 30

    body[size] = 0;
    printk("\n -----size = %d,-----\n", size);

    // Step 5: Connect server using ip............................................................................

    printk("\nStarting connection to server...\n");

    if (WiFiSpiClient_connect_With_Ip(server, 5001, stop_flag)) {
      printk("\nconnected to server\n");

      ret = WiFiSpiClient_write(body, size, stop_flag);
      printk("size 1 = %d \n", ret);

      if ((stop_flag != NULL) && (*stop_flag)) {
        return WIFI_DATA_SEND_PAUSED;
      }

      ret = WiFiSpiClient_write(wav_format_header_array, WAV_AUDIO_HEADER_LENGTH, stop_flag);
      printk("size 2 = %d \n", ret);

      if ((stop_flag != NULL) && (*stop_flag)) {
        return WIFI_DATA_SEND_PAUSED;
      }

      // printk("\nSending data to cloud: ");
      // for (int z = 1; z < 32001; z++) {
      // printk("%d, ", audio_buff[z]);
      // }

      // Step 6: Send the body.................................................................
      // Step 7: Send the Audio Buffer.......................................................

      while (k < buff_len) {
        if ((buff_len - k) < WIFI_AUDIO_TRANSFER_BYTES) {
          ret = WiFiSpiClient_write(audio_buff + k, (buff_len - k), stop_flag);
          printk("size %d = %d \n", (3 + k), ret);

          if ((stop_flag != NULL) && (*stop_flag)) {
            return WIFI_DATA_SEND_PAUSED;
          }
        } else {
          ret = WiFiSpiClient_write(audio_buff + k, WIFI_AUDIO_TRANSFER_BYTES, stop_flag);
          printk("size %d = %d \n", (3 + k), ret);

          if ((stop_flag != NULL) && (*stop_flag)) {
            return WIFI_DATA_SEND_PAUSED;
          }
        }
        k += WIFI_AUDIO_TRANSFER_BYTES;

        if (stm32_interrupt_flag == true) {
          printf("\n----------------------- Break Not_Confident loop \n");
          return 0;
        }
      }

      // Step 8: Return from WiFiSpiClient..........................................................

      ret = WiFiSpiClient_write_in("\r\n--------------------------d74496d66958873e--\r\n\r\n", stop_flag);
      printk("size last = %d \n", ret);

      if ((stop_flag != NULL) && (*stop_flag)) {
        return WIFI_DATA_SEND_PAUSED;
      }

      printk("\n =====> not_confident file sent successfully");
      not_confident_sent = true;
    }

    // Calculate the execution time in miliseconds++++++++++++++++++++++++++++++++
    // middle_time2 = k_cycle_get_32();
    // exec_time = ((middle_time2 - middle_time1) / (32768 / 1000));
    // printk("\n Time for ESP to Cloud = %d ms\n", exec_time);

    // Calculate the execution time in miliseconds+++++++++++++++++++++++++++++++++
    // end_time = k_cycle_get_32();
    // exec_time = ((end_time - middle_time2) / (32768 / 1000));
    // printk("\n Time for Cloud to Nordic = %d ms\n", exec_time);
    memset(audio_buff, 0, sizeof(audio_buff));
  } else {
    printk("\nfailed to connect server\n");
    return SERVER_CONNECTION_FAILED;
  }

  // Calculate the execution time in miliseconds+++++++++++++++++++++++++++++++++++++
  // exec_time = ((end_time - start_time) / (32768 / 1000));
  // printk("\n Total time = %d ms\n", exec_time);
  return 0;
}

int8_t Send_Audio_files_to_Server(bool *stop_flag) {
  if (!is_update_to_auto_ai) {
    return 0;
  }

  if (!wifi_module_active) {
    return 0;
  }

  uint8_t reset_wifi_module = 0;
  int8_t wifi_status = 0;
  char Sdetect_count[5];
  unsigned long int time_stamp;

  while (1) {
    if (!((is_doorbot_uninstalled_uncalibrated() == true) || (ADO_get_restart_state() == true))) {
      door_close_to_open_percentage();

      // if door is in b/w close and open wait for autoclose to send files
      if (current_door_percentage > 5 && is_door_in_open != 2) {
        // printf("\n===================>Send_Audio_files_to_Server loop break");
        return 0;
      }
    }

    // break while loop if no audio file to send.
    if (stored_audio_files_count() == 0) {
      printk("All audio files are uploaded to Auto-AI server\n");
      is_update_to_auto_ai = false;
      // WiFi_module_on_off(WIFI_MODULE_OFF);
      break;
    }

    // check wifi connection.
    wifi_status = ADO_WiFi_status(stop_flag);

    if ((stop_flag != NULL) && (*stop_flag)) {
      printk("\n Halting wifi data sending \n");
      // return from function and execute ADO command if there.
      break;
    }

    if (WL_CONNECTED == wifi_status) {
      server_connected = true;
      printk("\n____________________________________________________\n");
      // read audio files from External flash memory ic.
      audio_auto_ai_params.tag[0] = '\0';
      // Read_audio_file(audio_file_buff, &audio_file_number, confidence_scores, &label, m_ver);
      Read_audio_file(&audio_file);
      audio_auto_ai_params.csv[0] = '\0';

      audio_file.no_of_classes = (audio_file.no_of_classes > 10) ? 10 : audio_file.no_of_classes;
      printk("no of classes-----%d\n", audio_file.no_of_classes);

      //This case is for 4 classes.
      if (audio_file.no_of_classes == 4) {
        for (int i = 0; i < audio_file.no_of_classes; i++) {
          audio_file.confidence_val[i] = (audio_file.confidence_val[i] * 100.0);
          if (audio_file.confidence_val[i] <= 0.0f) { // 0.1 to 0.0
            audio_file.confidence_val[i] = 0.00f;
          } else if (audio_file.confidence_val[i] >= 100.0f) {
            audio_file.confidence_val[i] = 100.00f;
          }
          printf("%f, ", audio_file.confidence_val[i]);

          if (i == 0) {
            strcat(audio_auto_ai_params.csv, "Door Close=");
          } else if (i == 1) {
            strcat(audio_auto_ai_params.csv, "Door Open=");
          } else if (i == 2) {
            strcat(audio_auto_ai_params.csv, "Door Stop=");
          } else if (i == 3) {
            strcat(audio_auto_ai_params.csv, "Unknown=");
          }

          if (audio_file.confidence_val[i] == 0.00) {
            sprintf(audio_auto_ai_params.ww_confidence_score, "0.000");
          } else {
            gcvt(audio_file.confidence_val[i], 6, audio_auto_ai_params.ww_confidence_score); // 3
          }

          // gcvt(audio_file.confidence_val[i], 3, audio_auto_ai_params.ww_confidence_score);
          strcat(audio_auto_ai_params.csv, audio_auto_ai_params.ww_confidence_score);
          if (i == (audio_file.no_of_classes - 1)) {
            strcat(audio_auto_ai_params.csv, "%");
            continue;
          }
          strcat(audio_auto_ai_params.csv, "%<br>");
        }
        //Add Live User response.................................................................
        strcat(audio_auto_ai_params.csv, "<br>");
        strcat(audio_auto_ai_params.csv, "User Response=");

        if (strlen(audio_data_for_ai.live_user_response)>0) {
          strcpy(audio_auto_ai_params.live_user_response,audio_data_for_ai.live_user_response);
        }
        strcat(audio_auto_ai_params.csv, audio_auto_ai_params.live_user_response);  //Add user response on csv file.
        strcat(audio_auto_ai_params.csv, "<br>");
        printf("\nLive user response sending to server 1 = %s \n", audio_auto_ai_params.live_user_response);
        //Upto this..............................................................................
      } else {
        //This case is for 7 classes.
        for (int i = 0; i < audio_file.no_of_classes; i++) {
          audio_file.confidence_val[i] = (audio_file.confidence_val[i] * 100.0);

          if (audio_file.confidence_val[i] <= 0.1f) {
            audio_file.confidence_val[i] = 0.1f;
          } else if (audio_file.confidence_val[i] >= 100.0f) {
            audio_file.confidence_val[i] = 100.0f;
          }
          printf("%f, ", audio_file.confidence_val[i]);

          if (i == 0) {
            strcat(audio_auto_ai_params.csv, "Silence=");
          } else if (i == 1) {
            strcat(audio_auto_ai_params.csv, "Unknown=");
          } else if (i == 2) {
            strcat(audio_auto_ai_params.csv, "Door Open=");
          } else if (i == 3) {
            strcat(audio_auto_ai_params.csv, "Door=");
          } else if (i == 4) {
            strcat(audio_auto_ai_params.csv, "Open=");
          } else if (i == 5) {
            strcat(audio_auto_ai_params.csv, "Door Stop=");
          } else if (i == 6) {
            strcat(audio_auto_ai_params.csv, "Door Close=");
          } else if (i == 7) {
            strcat(audio_auto_ai_params.csv, "Noise=");
          }
          gcvt(audio_file.confidence_val[i], 3, audio_auto_ai_params.ww_confidence_score);
          strcat(audio_auto_ai_params.csv, audio_auto_ai_params.ww_confidence_score);

          if (i == (audio_file.no_of_classes - 1)) {
            strcat(audio_auto_ai_params.csv, "%");
            continue;
          }
          strcat(audio_auto_ai_params.csv, "%<br>");
        }
        //Add Live User response.................................................................
        strcat(audio_auto_ai_params.csv, "<br>");
        strcat(audio_auto_ai_params.csv, "User Response=");

        if(strlen(audio_data_for_ai.live_user_response)>0) {
          strcpy(audio_auto_ai_params.live_user_response,audio_data_for_ai.live_user_response);
        }
        strcat(audio_auto_ai_params.csv, audio_auto_ai_params.live_user_response);  //Add user response on csv file.
        strcat(audio_auto_ai_params.csv, "<br>");
        printf("\nLive user response sending to server 2 = %s \n", audio_auto_ai_params.live_user_response);
        //Upto this..............................................................................
      }

      strcat(audio_auto_ai_params.csv, "\0");
      printf("\n");

      // printf("\ncsv = %s\n",audio_auto_ai_params.csv);

      itoa(audio_file.file_number, Sdetect_count, 10);
      printf("%s, %d \n", Sdetect_count, audio_file.file_number);
      printf("Label = 0x%X\n", audio_file.lable);

      if (audio_file.no_of_classes == 4) {
        switch (audio_file.lable) {
        case 0x0A:
          audio_auto_ai_params.label = LABEL_DOOR_OPEN;
          gcvt(audio_file.confidence_val[1], 3, audio_auto_ai_params.ww_confidence_score);
          printf("---audio_auto_ai_params.ww_confidence_score for door open = %s --\n",
              audio_auto_ai_params.ww_confidence_score);
          break;

        case 0x0B:
          audio_auto_ai_params.label = LABEL_DOOR_CLOSE;
          gcvt(audio_file.confidence_val[0], 3, audio_auto_ai_params.ww_confidence_score);
          printf("---audio_auto_ai_params.ww_confidence_score for door close = %s --\n",
              audio_auto_ai_params.ww_confidence_score);
          break;

        case 0x0C:
          audio_auto_ai_params.label = LABEL_DOOR_STOP;
          gcvt(audio_file.confidence_val[2], 3, audio_auto_ai_params.ww_confidence_score);
          printf("---audio_auto_ai_params.ww_confidence_score for door stop = %s --\n",
              audio_auto_ai_params.ww_confidence_score);
          break;

        case 0x0D:
          audio_auto_ai_params.label = LABEL_DOOR_STOP_LT;
          gcvt(audio_file.confidence_val[2], 3, audio_auto_ai_params.ww_confidence_score);
          printf("---audio_auto_ai_params.ww_confidence_score for door stop = %s --\n",
              audio_auto_ai_params.ww_confidence_score);
          break;

        case 0x0E:
          audio_auto_ai_params.label = LABEL_DOOR_CLOSE_LT;
          gcvt(audio_file.confidence_val[0], 3, audio_auto_ai_params.ww_confidence_score);
          printf("---audio_auto_ai_params.ww_confidence_score for door stop = %s --\n",
              audio_auto_ai_params.ww_confidence_score);
          break;

        case 0x09:
          audio_auto_ai_params.label = LABEL_DOOR_OPEN_LT;
          gcvt(audio_file.confidence_val[1], 3, audio_auto_ai_params.ww_confidence_score);
          printf("---audio_auto_ai_params.ww_confidence_score for door stop = %s --\n",
              audio_auto_ai_params.ww_confidence_score);
          break;

          //Uttam added for testing
        case 0x0F:
          audio_auto_ai_params.label = LABEL_NOT_CONFIDENT;
          //gcvt(audio_file.confidence_val[3], 3, audio_auto_ai_params.ww_confidence_score);
          //printf("---audio_auto_ai_params.ww_confidence_score for Not Confident = %s --\n",
          //    audio_auto_ai_params.ww_confidence_score);
          break;
          //Upto this.

        default:
          break;
        }
      } else {
        switch (audio_file.lable) {
        case 0x09:
          audio_auto_ai_params.label = LABEL_DOOR_OPEN_LT;
          gcvt(audio_file.confidence_val[2], 3, audio_auto_ai_params.ww_confidence_score);
          break;

        case 0x0A:
          audio_auto_ai_params.label = LABEL_DOOR_OPEN;
          gcvt(audio_file.confidence_val[2], 3, audio_auto_ai_params.ww_confidence_score);
          break;

        case 0x0B:
          audio_auto_ai_params.label = LABEL_DOOR_CLOSE;
          gcvt(audio_file.confidence_val[6], 3, audio_auto_ai_params.ww_confidence_score);
          break;

        case 0x0C:
          audio_auto_ai_params.label = LABEL_DOOR_STOP;
          gcvt(audio_file.confidence_val[5], 3, audio_auto_ai_params.ww_confidence_score);
          break;

        case 0x0D:
          audio_auto_ai_params.label = LABEL_DOOR_STOP_LT;
          gcvt(audio_file.confidence_val[5], 3, audio_auto_ai_params.ww_confidence_score);
          break;

        case 0x0E:
          audio_auto_ai_params.label = LABEL_DOOR_CLOSE_LT;
          gcvt(audio_file.confidence_val[6], 3, audio_auto_ai_params.ww_confidence_score);
          break;

        default:
          break;
        }
      }

      // printf("---%s---\n", audio_auto_ai_params.label);
      printf("audio_auto_ai_params.label = ---%s---\n", audio_auto_ai_params.label);

      audio_auto_ai_params.tag[0] = '\0';
      strcpy(audio_auto_ai_params.tag, audio_file.audio_model_ver);
      printk("\nAudio_Model_ver = %s\n", audio_auto_ai_params.tag);

      audio_auto_ai_params.device_name = ADO_NAME;
      audio_auto_ai_params.file_name[0] = '\0';
      strcat(audio_auto_ai_params.file_name, audio_auto_ai_params.device_name);
      strcat(audio_auto_ai_params.file_name, Sdetect_count);
      strcat(audio_auto_ai_params.file_name, ".wav");

      printk("file name = %s\n", audio_auto_ai_params.file_name);
      printk("csv = %s\n", audio_auto_ai_params.csv);

      audio_auto_ai_params.params_str_length = (strlen(audio_auto_ai_params.ww_confidence_score) +
                                                strlen(audio_auto_ai_params.file_name) +
                                                strlen(audio_auto_ai_params.csv) +
                                                strlen(audio_auto_ai_params.label) +
                                                strlen(audio_auto_ai_params.tag));

      if ((stop_flag != NULL) && (*stop_flag)) {
        printk("\n Halting wifi data sending \n");
        // return from function and execute ADO command if there.
        break;
      }

      // send file to server.
      time_stamp = k_uptime_get();
      wifi_status = Send_data_to_auto_ai(audio_auto_ai_params, &audio_file.data[0],
          AUDIO_BUFF_LENGTH, stop_flag);

      if (wifi_status) {
        // check for wifi send paused or not due to commands.
        if (WIFI_DATA_SEND_PAUSED == wifi_status) {
          printk("\n Halting wifi data sending \n");
          // return from function and execute ADO command if there.
          break;
        } else if (SERVER_CONNECTION_FAILED != wifi_status) {
          printk("no response from server or failed to send server\n");
          audio_file_retry_attempts++;

          // if updating faild more than 3 times skip audio file and send other one.
          if (3 == audio_file_retry_attempts) {
            audio_file_retry_attempts = 0;
            Update_server_audio_file_address();
          }
        } else {
          reset_wifi_module++;

          if (reset_wifi_module == 3) {
            // printk("WiFi Hardware reseting...\n");
            // WiFi_module_hard_reset();
            wifi_status = WiFi_Network_Connect(stop_flag);
            // check for wifi send paused or not due to commands.

            if (WIFI_DATA_SEND_PAUSED == wifi_status) {
              printk("\n Halting wifi data sending \n");
              // return from function and execute ADO command if there.
              break;
            }
            k_msleep(50U);
          } else if (reset_wifi_module == 6) {
            printk("WiFi Hardware reseting again...\n");
            WiFi_module_hard_reset();
            wifi_status = WiFi_Network_Connect(stop_flag);

            // check for wifi send paused or not due to commands.
            if (WIFI_DATA_SEND_PAUSED == wifi_status) {
              printk("\n Halting wifi data sending \n");
              // return from function and execute ADO command if there.
              break;
            }
            k_msleep(50U);
          } else if (reset_wifi_module > 6) {
            wifi_module_active = false;
            printk("\n Please check wifi module\n");
            break;
          }
        }
      } else {
        printk("WiFi send Time in mills = %ld, %ld\n", time_stamp, (k_uptime_get() - time_stamp));
        Update_server_audio_file_address();
        reset_wifi_module = 0;
        wifi_module_active = true;
      }
    } else {
      printk("WiFi Network not connected status = %d\n", wifi_status);

      // reconnect to wifi network again.
      wifi_status = WiFi_Network_Connect(stop_flag);

      if (WIFI_DATA_SEND_PAUSED == wifi_status) {
        printk("\n Halting wifi data sending \n");
        // return from function and execute ADO command if there.
        break;
      }

      // still not working reset wifi module and reconnect again with wifi network.
      else if (WL_CONNECTED != wifi_status) {
        printk("Not able to connect with WiFi Network Please check your SSID and Password.\n");
        WiFi_module_hard_reset();
        wifi_module_active = false;
      }
    }
  }
  return 0;
}