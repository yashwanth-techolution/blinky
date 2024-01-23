#ifndef __BLE_MODULE_H_
#define __BLE_MODULE_H_

#include <zephyr/types.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <bluetooth/services/bms.h> // for Bond management service
#include "Includes/Intent_Module.h"

#ifdef __cplusplus
extern "C" {
#endif  

#define EM_LOCK_REQUEST 0x7E
#define EM_LOCK_RESPONSE 0x7F
#define NUM_CONN_MAX    CONFIG_BT_MAX_CONN

static struct bt_conn *curr_conn[NUM_CONN_MAX];

typedef enum lock_status {
  BALL_SWITCH_PRESSED = 1,
  EM_LOCKED_TRUE,
  EM_LOCKED_FALSE,
  EM_LOCK_ENABLE,
  EM_LOCK_DISABLE,
  EM_LOCK_ENABLE_RESPONSE,
  EM_LOCK_DISABLE_RESPONSE,
  IS_DOORBOT_ENABLED,
  DOORBOT_ENABLED,
  DOORBOT_DISABLED,
} lock_status;

extern lock_status enum_lock_status;
 
typedef struct emlock_evt {
  char cmd_type;
  char data;
} emlockmsg;

extern emlockmsg emlock_evnt_msg;
extern bool is_door_locked;
extern bool is_lock_config, ball_switch_on;
//-------------------------------------------------------------------------------------

#define BT_UUID_READ_WRITE_vALUE \
  BT_UUID_128_ENCODE(0x0e085574, 0xc584, 0x11ed, 0xafa1, 0x0242ac120002)
#define BT_UUID_INPUT_vALUE \
  BT_UUID_128_ENCODE(0x0e0858b2, 0xc584, 0x11ed, 0xafa1, 0x0242ac120002)
#define BT_UUID_OUTPUT_vALUE \
  BT_UUID_128_ENCODE(0x0e085d12, 0xc584, 0x11ed, 0xafa1, 0x0242ac120002)


#define BT_UUID_READ_WRITE_SERVICE \
  BT_UUID_DECLARE_128(BT_UUID_READ_WRITE_vALUE)
// input to the peripheral device
#define BT_UUID_INPUT \
  BT_UUID_DECLARE_128(BT_UUID_INPUT_vALUE)
// output from the peripheral device
#define BT_UUID_OUTPUT \
  BT_UUID_DECLARE_128(BT_UUID_OUTPUT_vALUE)

  //----------------------------------------------------
#define BLE_Init_Succcess          0
#define BLE_PassKey_NotSET        -1
#define BLE_Init_Fail             -2
#define Fail_BMS_init             -3
#define Advertising_Fail_ToStart  -4

// Temporary define since we moved from SDk version 1.4.2 to 1.8.0, this macro is not defined in new version
#define CONFIG_BT_L2CAP_RX_MTU CONFIG_BT_BUF_ACL_RX_SIZE

// Device name to be advertised as mentioned in proj.cnf file in project directory
#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME       
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

/** @brief ADO Service UUID. */
#define ADO_SERVICE_UUID_VAL \
	BT_UUID_128_ENCODE(0xf0ad0100, 0x6661, 0x6365, 0x6f70, 0x656e6f626f74)

/** @brief ADO command receive characteristic UUID. */
#define ADO_RX_CHAR_UUID_VAL \
	BT_UUID_128_ENCODE(0xf0ad0101, 0x6661, 0x6365, 0x6f70, 0x656e6f626f74)

/** @brief ADO command send Characteristic UUID. */
#define ADO_TX_CHAR_UUID_VAL \
	BT_UUID_128_ENCODE(0xf0ad0102, 0x6661, 0x6365, 0x6f70, 0x656e6f626f74)

#define ADO_SERVICE_UUID       BT_UUID_DECLARE_128(ADO_SERVICE_UUID_VAL)
#define ADO_RX_CHAR_UUID       BT_UUID_DECLARE_128(ADO_RX_CHAR_UUID_VAL)
#define ADO_TX_CHAR_UUID       BT_UUID_DECLARE_128(ADO_TX_CHAR_UUID_VAL)


/** @brief Service UUID for "uuid_service_nearby". */
#define SERVICE_UUID_NEARBY_VAL \
    BT_UUID_128_ENCODE(0x48fe3e40, 0x0817, 0x4bb2, 0x8633, 0x3073689c2dba)

/** @brief Characteristic UUID for "uuid_nearby_AccessoryData". */
#define CHAR_UUID_NEARBY_ACCESSORY_DATA_VAL \
    BT_UUID_128_ENCODE(0x95e8d9d5, 0xd8ef, 0x4721, 0x9a4e, 0x807375f53328)

#define SERVICE_UUID_NEARBY BT_UUID_DECLARE_128(SERVICE_UUID_NEARBY_VAL)
#define CHAR_UUID_NEARBY_ACCESSORY_DATA BT_UUID_DECLARE_128(CHAR_UUID_NEARBY_ACCESSORY_DATA_VAL)


#define MAX_BLE_PCKT_MSG_LEN    CONFIG_BT_L2CAP_RX_MTU - 3U
#define MAX_BLE_DATA_PACKETS    10U    // Fixed the max number of BLE packets to be received 
#define BLE_CMD_PREAMBLE_LEN    8U     // Total number of NON-Message bytes in 1st BLE packet
#define REG_EVT_NOTIFY_MSG_LEN  3U     // Event notification message len without extra data bytes

extern struct bt_conn_info conn_info;

// Enum containing the ADO response values
typedef enum {
  SUCCESS = 1,
  INVALID_SIGNATURE,    // For future use
  EXCEED_PRIVILEGES,    // For future use
  FRAGMENT_ERROR,
} ado_response_status_t;

// Structure to contain the incoming command packet data
typedef struct {  
  uint8_t               pckt_num;
  uint8_t               msg_id;
  uint8_t               msg_byte[MAX_BLE_PCKT_MSG_LEN];
} ble_cmd_packet_t;

// Structure depicting the incoming packet info
typedef struct {
  // Track keeping variables
  uint8_t         curr_packet_idx;
  uint16_t        rem_msg_len;

  // Packet variables
  uint8_t         total_pckt_count;
  uint8_t         cmd_type;
  uint16_t        client_id;
  uint16_t        total_msg_length;
} ble_cmd_packet_info_t;


// Structure depicting the response packet info
typedef struct {
  // Packet variables
  uint8_t               total_pckt_cnt;
  uint8_t               failed_pckt_cnt;
  uint8_t               req_id;
  uint16_t              client_id;
  uint16_t              total_msg_len;
  uint8_t               res_type;
  uint8_t               res_of_cmd;
  ado_response_status_t res_status;
  uint8_t               failed_pckt_idxs[MAX_BLE_DATA_PACKETS];
} ble_res_packet_t;


// Structure containing status values received from command handlers
typedef struct {
  uint8_t               cmd;
  uint8_t               cmd_type;
  ado_response_status_t res_status;
  uint8_t               evt_msg[48];
  uint8_t               evt_msg_len;
} ado_event_status_t;

// Structure depicting the event response packet info
typedef struct {
  uint8_t               pckt_num;
  uint8_t               req_id;
  uint8_t               total_pckt_cnt;  
  uint16_t              client_id;
  uint16_t              total_msg_len;
} ado_event_header_t;

// Variable declarations
extern ado_event_status_t      evt_status;
extern ado_event_header_t      evt_header;

extern uint8_t          evt_buffer[MAX_BLE_PCKT_MSG_LEN];
extern ble_cmd_packet_t cmd_packets[MAX_BLE_DATA_PACKETS];
extern const k_tid_t    ble_notify_thrd_id; 


// Function Declarations
/**********************************************************************************************************
 * Function name  :   ADO_recv_data()
 *
 * Description    :   1. It is a callback function registered to the BLE stack which will be called when
 *                       there is any data is received at the registered characteristics, for ADO it is 
 *                       ADO RX Characteristics.
 *
 *                    2. This function expects the received data to be a byte array and it will further pass
 *                       the received data buffer to translate the received command and act accordingly.
 *                       Presently, this function notifies the same received data over ADo TX characteristic
 *                       as an acknowledgement, which will change in future to the response of the command
 *                       execution.
 *
 * Params         :   1. <in> struct bt_conn *conn :  
 *                            Pointer to a structure containing the handle to the Live BLE connection over
 *                            which the data is received.
 *                            
 *                    2. <in> const struct bt_gatt_attr *attr :  
 *                            Pointer to a structure containing the handle of the attribute of the service.
 *                            to which the data has been written.
 *
 *                    3. <in> const void *buf :   
 *                            Pointer to the buffer holding the received data which has been allocated by the
 *                            BLE Stack. Since our data will be in the form of a byte array, we can type cast
 *                            this buffer to (uint8_t) to read the required values.
 *
 *                    4. <in> uint16_t len :   
 *                            A variable containing the length of received data in buf.
 * 
 *                    5. <in> uint16_t offset :   
 *                            A variable containing the offset if any, in our case it is unused.
 *
 *                    6. <in> uint8_t flags :
 **                            A variable containing the flags if any, in our case it is unused.
 *            
 *
 * Return         :     <ssize_t>  lenghth of data received on success, 
 ***********************************************************************************************************/
ssize_t ADO_recv_data(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);


/**********************************************************************************************************
 * Function name  :   ADO_cccd_cfg_changed()
 *
 * Description    :   1. It is a callback function registered to the BLE stack which will be called when
 *                       the "Client Configurable Characteristic Descriptor" (CCCD) attriute of registered 
 *                       characteristics, is written by the BLE controller to enable/disable the ADO 
 *                       Notifications for ADO TX Characteristics.
 *
 *                    2. This function just sets a global boolean variable to depict whether notifications
 *                       are enabled or disabled.
 *
 * Params         :   1. <in> const struct bt_gatt_attr *attr :  
 *                            Pointer to a structure containing the handle of the attribute of the service.
 *                            to which the data has been written.
 *
 *                    2. <in> uint16_t value :   
 *                            The value written to the CCCD Attribute by the BLE Controller.
 *
 * Return         :   None
 ***********************************************************************************************************/
void ADO_cccd_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);



/**********************************************************************************************************
 * Function name  :   initBLE()
 *
 * Description    :   1. It is a function to initialise the BLE stack, ADO BLE services, register the required 
 *                       callbacks with the BLE Subsystem and start advertising. It should be called once prior
 *                       starting any BLE activity.
 *
 * Params         :   None.
 *
 * Return         :   <int>  0 on success, non-zero on failure. 
 ***********************************************************************************************************/
int initBLE(void);

/**********************************************************************************************************
 * Function name  :   ADO_sendCmdACK()
 *
 * Description    :   1. Function to send back the acknowledgement of command packet receipt, it should be
 *                       supplied with a valid response buffer pre-initialised with the respective ACK values.
 *
 * Params         :   1. <in>  uint8_t* res_buf: Pointer to response buffer
 *                    2. <in>  uint8_t  len    : length of response buffer                   
 *
 * Return         :   <int8_t>  0 on success, negative error code on failure. 
 ***********************************************************************************************************/
int8_t ADO_sendCmdACK(uint8_t* res_buf, uint8_t len);

/**********************************************************************************************************
 * Function name  :   translate_ble_cmd()
 *
 * Description    :   1. It receives an array of bytes (uint8_t) and is responsible to translate it into 
 *                       respective ADO command as per the byte packet structure we have devised.
 *
 *                    2. Depending upon the received command, it will prepare send a mailbox message to 
 *                       command manager thread which will delegate the command to the respective thread
 *
 * Params         :   1. <in> uint8_t * data_array :  
 *                            Pointer to an array of bytes received by over BLE, this will be our command
 *                            to be translated and Must have the value of individual bytes as per the set
 *                            packet format. Presently, we have limited it to 8 bytes later it could vary.
 *
 *                    2. <in> uint16_t data_length :  
 *                            It will contain the unsigned value of the length of total bytes in data_array
 *                            It is unused presently but may be required later when the data_array size will
 *                            be variable.
 *
 * Return         :     <int32_t>  0 on success, non-zero on failure.
 ***********************************************************************************************************/
int32_t translate_ble_cmd(uint8_t * data_array, uint16_t data_length);


/**********************************************************************************************************
 * Function name  :   packetizeDataBuffer()
 *
 * Description    :   1. This function takes the received chunk over BLE and appends in a global array of
 *                       structure of type ble_cmd_packet_t. Therefore, if there are multiple chunks of 
 *                       BLE data in a complete data transfer, this function must always be called every 
 *                       time a new chunk is received. After complete data is transferred in multiple calls,
 *                       the data so received will be found in the global structure. 
 *
 * Params         :   1. <in> uint8_t * data_array :  
 *                            Pointer to an array of bytes received by over BLE, this will be our command
 *                            to be translated and Must have the value of individual bytes as per the set
 *                            packet format.    
 *
 * Return         :   <int32_t>  remaining length to be read on success, zero if no more data to be read,
 *                              negative value on failure.
 ***********************************************************************************************************/
int32_t packetizeDataBuffer(uint8_t * data_array);


/**********************************************************************************************************
 * Function name  :   bufferizeResPacket()
 *
 * Description    :   1. This function reads the global response packet structure and prepares a buffer of
 *                       bytes to be sent as an acknowledgement of packet receipt to the BLE central. The 
 *                       global response packet structure MUST be pre-initialised with the respective values
 *                       of member variables prior making a call to this function. After the buffer is ready
 *                       it can be sent back to the BLE central as a notification by calling ADO_sendCmdACK()
 *
 * Params         :   1. <out> uint8_t * res_buf :  
 *                            Pointer to an array of bytes created by the caller function to be filled with
 *                            packetised values of global response structure in predefined format. It must
 *                            be of length equal to the maximum response packet size allowed. In our case,
 *                            it can be determined by MAX_BLE_PCKT_MSG_LEN.
 *
 * Return         :   <uint8_t> actual length of bufferised response buffer.
 ***********************************************************************************************************/
uint8_t bufferizeResPacket(uint8_t *res_buf);


/**********************************************************************************************************
 * Function name  :   bufferizeEvtPacket()
 *
 * Description    :   1. This function reads the global event response packet structure and prepares a buffer
 *                       of bytes to be sent as an event notification of ADO events or present action status
 *                       to the BLE central. The global event response structure MUST be pre-initialised with
 *                       the respective values of member variables prior making a call to this function. After
 *                       the buffer is ready, it can be sent back to the BLE central as a notification by
 *                       calling ADO_sendCmdACK().
 *
 * Params         :   1. <in>  ado_event_status_t evt_status:
                              A structure variable containing the event status values to be notified over BLE

 *                    2. <out> uint8_t * res_buf :  
 *                            Pointer to an array of bytes created by the caller function to be filled with
 *                            packetised values of global event response structure in predefined format. It
 *                            must be of length equal to the maximum event response packet size allowed. In
 *                            our case, it can be determined by MAX_BLE_PCKT_MSG_LEN.
 *
 * Return         :   <uint8_t> actual length of bufferised event response buffer.
 ***********************************************************************************************************/
uint8_t bufferizeEvtPacket(ado_event_status_t evt_status, uint8_t *res_buf);

/**********************************************************************************************************
 * Function name  :   ADO_BleNotify_Thread()
 *
 * Description    :   1. This thread is going to be used to notify any data over BLE to ED. This threads 
 *                       keeps waiting to receive any msg in prescribed structure format from cmd manager 
 *                       thread                       
 *
 * Params         :   None.
 *
 * Return         :   Nothing
 ***********************************************************************************************************/
void ADO_BleNotify_Thread(void);

/**********************************************************************************************************
 * Function name  :   ADO_notify_ble()
 *
 * Description    :   1. This function takes the required parameters to be notified over BLE from any thread
 *                       and prepares a message to the destination thread and post it to that thread's mailbox. 
 *
 *                    2. The destination thread as per our architecture is command manager, but in case if the 
 *                       message is being sent from cmd_mgr thread itself, the destination thread will be 
 *                       ble_notify_thread.
 *
 * Params         :   1. <in> k_tid_t dest_thrd_id :  
 *                            The destination thread id to whose mailbox the data is to be written. Presently
 *                            it is expected to be command_mgr_thrd_id if we send from any thread. Whereas, if
 *                            we are calling this function from inside the command manager thread, the thread id
 *                            must be provided as ble_notify_thrd_id, to send he notification directly.                            
 *                            
 *                    2. <in> uint8_t cmd_type :  
 *                            The type of event notification i.e. the event is in response to to what type of
 *                            command. For example, for calibration events, it should be ADO_CALIBRATION_RESPONSE.
 *
 *                    3. <in> uint8_t cmd :   
 *                            The event is in response to which command received? i.e in case of calibration commands,
 *                            one of the value of this variable could be CAL_START.
 *
 *                    4. <in>  uint8_t status :   
 *                            The status of the command or operation that needs to be notified to ED. As in our example
 *                            of CAL_START command received, the value of this variable could be CAL_STARTED.
 *        
 *                    5. <in> uint8_t *evt_msg :   
 *                            A pointer to byte array of extra bytes that needs to be sent as a part of BLE event notification.
 *                            This pointer must be passed with an address of a valid evt_msg[] buffer with respective values.
 *                            As in case of installation running, we need to keep notifying the percentage of clamp movement
 *                            as an extra byte. it can be of a maximum MAX_BLE_DATA_PACKETS length.
 *
 *                    6. <in> uint8_t evt_msg_len :
 *                            A variable containing the number of extra bytes in evt_msg, as in case we send clamp percent as
 *                            a single extra byte, the evt_msg_len will be 1 (byte). 
 *            
 *
 * Return         :    Nothing. 
 ***********************************************************************************************************/
void ADO_notify_ble(k_tid_t dest_thrd_id, uint8_t cmd_type, uint8_t cmd, uint8_t status, uint8_t *evt_msg, uint8_t evt_msg_len);


// following functions have been provided by the Nordic SDK, and can better be referred in their documentation.
void connected(struct bt_conn *conn, uint8_t err);
void disconnected(struct bt_conn *conn, uint8_t reason);

#if defined(CONFIG_BT_LBS_SECURITY_ENABLED)
void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err);
void auth_passkey_display(struct bt_conn *conn, unsigned int passkey);
void auth_cancel(struct bt_conn *conn);
void pairing_confirm(struct bt_conn *conn);
void pairing_complete(struct bt_conn *conn, bool bonded);
void pairing_failed(struct bt_conn *conn, enum bt_security_err reason);
int8_t Lock_sendCmdACK(uint8_t *res_buf, uint8_t len);
int8_t uwb_sendCmdACK(uint8_t *res_buf, uint8_t len);
#endif

#ifdef __cplusplus
}
#endif


#endif //__BLE_MODULE_H_
