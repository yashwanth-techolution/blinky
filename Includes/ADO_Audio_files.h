#ifndef AUDIO_FILES_MODULE_H_
#define AUDIO_FILES_MODULE_H_

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <stdbool.h>
#include <stdint.h>

#define START_AUTO_AI_SERVER 10

//#define ADO_NAME  "ADO_TECHO_MAINDOOR_";
//#define ADO_NAME  "ADO_TECHO_MAINCONF_";
//#define ADO_NAME  "ADO_TECHO_MINICONF_";
//#define ADO_NAME  "ADO_TECHO_MECHLAB_";

#define ADO_NAME "EVO2MECHLAB";
//#define ADO_NAME  "EVO2MAINCONF";
//#define ADO_NAME  "EVO2CABINONE";
//#define ADO_NAME  "EVO2MAINDOOR";

//#define ADO_NAME  "adominiconfdev_";
//#define ADO_NAME  "techolutionindiaprod1_";
//#define ADO_NAME  "techolutionllc_";
//#define ADO_NAME  "adocabinqa_";
//#define ADO_NAME  "ADO_TECHO_CABIN1_";
//#define ADO_NAME  "mainconferenceINDpreprod_";
//#define ADO_NAME  "adodatacoll_";

#define LABEL_DOOR_OPEN "door_open";
#define LABEL_DOOR_OPEN_LT "door_open_LT";
#define LABEL_DOOR_CLOSE "door_close";
#define LABEL_DOOR_CLOSE_LT "door_close_LT";
#define LABEL_DOOR_STOP "door_stop";
#define LABEL_DOOR_STOP_LT "door_stop_LT";
#define LABEL_LOWER_THRESHOLD "lower_threshold";
#define LABEL_NOT_CONFIDENT "Not_Confident";

typedef struct audio_file_store_format {
  uint16_t file_number;
  uint8_t lable;
  uint8_t no_of_classes;
  float confidence_val[10];
  uint8_t file_header[44];
  uint8_t data[32000];
  uint8_t ver_str_len;
  char audio_model_ver[13];
} audio_file_store_format_t;

typedef struct {
  char csv[360];
  char tag[15];
  char file_name[50];
  // char ww_confidence_score[4];
  char ww_confidence_score[6];
  char *label;
  char *device_name;
  uint16_t params_str_length;
  char live_user_response[10];
} audio_auto_ai_parameters_t;

int8_t load_audio_file_address();
int8_t Store_not_confident_audio_file(uint8_t *audio_buff, uint16_t wake_word_count);
int8_t Store_audio_file(uint8_t *write_buff, uint16_t wake_word_count);
int8_t Check_Audio_files_update_status();
int8_t Send_Raw_Audio_File_To_Server(uint8_t *audio_buff, uint16_t buff_len, bool *stop_flag);
int8_t Send_Audio_files_to_Server(bool *stop_flag);
extern bool not_confident_sent;

#endif // AUDIO_FILES_MODULE_H_