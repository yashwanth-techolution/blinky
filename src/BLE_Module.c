//#include <dk_buttons_and_leds.h>
#include <logging/log.h>
#include <settings/settings.h>

// ADO custom includes
#include "Includes/ADO_Operations_Module.h"
#include "Includes/BLE_Module.h"
#include "Includes/Calibration_Module.h"
#include "Includes/Command_Manager_Module.h"
#include "Includes/Display_Module.h"
#include "Includes/IPC_Manager_Module.h"
#include "Includes/Install_ADO.h"

// BLE Notify thread definitions
#define BLE_NTFY_THRD_PRIORITY 8
#define BLE_NTFY_THRD_STACK_SIZE 1024
#define CONFIG_BT_LBS_SECURITY_ENABLED 1
bool is_door_locked = false;
bool is_lock_config = false, ball_switch_on = false;
lock_status enum_lock_status;
uint8_t my_data[37];

// Define the BLE Notify thread at compile time,
// so that it needs not to be invoked separately by any
// function but by scheduler immediately
K_THREAD_DEFINE(ble_notify_thrd_id, BLE_NTFY_THRD_STACK_SIZE,
    ADO_BleNotify_Thread, NULL, NULL, NULL, BLE_NTFY_THRD_PRIORITY, K_ESSENTIAL, 0);

// Register the module to logger for log output
LOG_MODULE_DECLARE(ble_module, CONFIG_SETTINGS_LOG_LEVEL);

static bool notify_enabled;
static const unsigned int BT_fixed_passkey = 123456;
static bool notify_enabled, notify_enabled2;

// Contains current connection info
struct bt_conn_info conn_info;
int count = 0;

// Create an array of packets of total_pckt_count length.
ble_cmd_packet_t cmd_packets[MAX_BLE_DATA_PACKETS];
ble_cmd_packet_info_t cmd_info;

// Structure to contain response message items
ble_res_packet_t res_pckt;

// Buffer to contain the event response packet values
uint8_t evt_buffer[MAX_BLE_PCKT_MSG_LEN];
ado_event_header_t evt_header;

// event update flag initalization.
bool event_update = false;

// mail box declaration
struct k_mbox ble_notify_mb;

// Authorization code for Bond Management Service
// In hex: {0x41, 0x42, 0x43, 0x44}
static const uint8_t bms_auth_code[] = {'A', 'B', 'C', 'D'};

// Prepare the Advertising packet
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    //BT_DATA(BT_DATA_NAME_SHORTENED, "em_lock", 7),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)};

// Addtional advertising data if security is enabled
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, ADO_SERVICE_UUID_VAL),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x1e, 0x18),
};

// Structure containing the connection callbacks
static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
#ifdef CONFIG_BT_LBS_SECURITY_ENABLED
    .security_changed = security_changed,
#endif
};

// Structure containing additional callbacks when security is enabled
#ifdef CONFIG_BT_LBS_SECURITY_ENABLED
static struct bt_conn_auth_cb conn_auth_callbacks = {
    .passkey_display = auth_passkey_display,
    .cancel = auth_cancel,
    .pairing_confirm = pairing_confirm, // -> Commented to enable fixed passkey pairing
    .pairing_complete = pairing_complete,
    .pairing_failed = pairing_failed};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
#endif

/* ADO Service Declaration */
BT_GATT_SERVICE_DEFINE(ADO_Service,
    BT_GATT_PRIMARY_SERVICE(ADO_SERVICE_UUID),
    BT_GATT_CHARACTERISTIC(ADO_TX_CHAR_UUID, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT, NULL, NULL, NULL),
    BT_GATT_CCC(ADO_cccd_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(ADO_RX_CHAR_UUID, BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_WRITE_ENCRYPT, NULL, ADO_recv_data, NULL), );

int8_t uwb_sendCmdACK(uint8_t *res_buf, uint8_t len) {
  memcpy(my_data, res_buf, len);
}
static ssize_t read_my_char(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, my_data, sizeof(my_data));
}

BT_GATT_SERVICE_DEFINE(ADO_Nearby_Service,
    BT_GATT_PRIMARY_SERVICE(SERVICE_UUID_NEARBY),
    BT_GATT_CHARACTERISTIC(CHAR_UUID_NEARBY_ACCESSORY_DATA, BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ_ENCRYPT, read_my_char, NULL, my_data), );

uint8_t a_number = 0;

ssize_t write_input(struct bt_conn *conn, const struct bt_gatt_attr *attr,
    const void *buf, unsigned short len, unsigned short offset, uint8_t flags) {
  const uint8_t *lockbuf = buf;

  if (!len) {
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }

  // if (*lockbuf >= 0 && *lockbuf <= 10) {
  //   a_number = *lockbuf;
  // } else {
  //   return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
  // }

  // printk("write_input [%d]\n", a_number);
  printk("\nReceived data From LOCK:");

  if (len == 1) {
    a_number = lockbuf[0];
    // printk("%d, ", lockbuf[i]);
    if (BALL_SWITCH_PRESSED == a_number) {
      printk("  BALL_SWITCH_PRESSED ");
      ball_switch_on = true;

    } else if (EM_LOCKED_TRUE == a_number) {
      printk("  EM_LOCKED_TRUE ");
      is_door_locked = true;

    } else if (EM_LOCKED_FALSE == a_number) {
      printk("  EM_LOCKED_FALSE ");
      ball_switch_on = false;
      is_door_locked = false;

    } else if (EM_LOCK_ENABLE_RESPONSE == a_number) {
      printk("  EM_LOCK_ENABLE_RESPONSE ");

      is_lock_config = true;

    } else if (EM_LOCK_DISABLE_RESPONSE == a_number) {
      printk("  EM_LOCK_DISABLE_RESPONSE ");

      is_lock_config = false;

    } else if (IS_DOORBOT_ENABLED == a_number) {
      // Check if the doorbot is installed and calibrated
      if (!((is_doorbot_uninstalled_uncalibrated() == true) || (ADO_get_restart_state() == true))) {
        enum_lock_status = DOORBOT_ENABLED;
        Lock_sendCmdACK(&enum_lock_status, 1);
      } else {
        enum_lock_status = DOORBOT_DISABLED;
        Lock_sendCmdACK(&enum_lock_status, 1);
      }
    } else {
      printk("  INVALID RESPONSE FROM LOCK len %d %d %X", len, lockbuf[0], lockbuf[1]);
    }
  } else if (len == 2) {
    if (lockbuf[0] == EM_LOCK_RESPONSE && lockbuf[1] == EM_LOCKED_FALSE) {
      ADO_notify_cmd_mgr(COMMAND, ADO_OPERATIONS, OPEN, NULL, 0);
    } else {
      printk("  INVALID RESPONSE FROM LOCK len %d %d %X", len, lockbuf[0], lockbuf[1]);
    }
  }
  printk("\n");
  return len;
}

ssize_t read_output(struct bt_conn *conn, const struct bt_gatt_attr *attr,
    void *buf, unsigned short len, unsigned short offset) {
  printk("read_output\n");
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &a_number,
      sizeof(a_number));
}

/*
 * Brief: Callback to be called when the notifications are enabled/disabled for ADO Tx Characteristic
 */
void ADO_cccd_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
  notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

void Lock_cccd_cfg_changed(const struct bt_gatt_attr *attr,
    uint16_t value) {
  notify_enabled = notify_enabled2 = (value == BT_GATT_CCC_NOTIFY);
}

BT_GATT_SERVICE_DEFINE(Lockservice,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_READ_WRITE_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_INPUT, BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_WRITE, NULL, write_input, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_OUTPUT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ, read_output, NULL, NULL),
    BT_GATT_CCC(Lock_cccd_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

);

int8_t Lock_sendCmdACK(uint8_t *res_buf, uint8_t len) {
  int8_t err = -1;
  if (notify_enabled2) {
    err = bt_gatt_notify(NULL, &Lockservice.attrs[3], res_buf, 1);
    // err = bt_gatt_write(conn_info,);
  }
  return err;
}

/*
 * Brief: Callback to be called when there is any data received for ADO Rx Characteristic
 */
ssize_t ADO_recv_data(struct bt_conn *conn, const struct bt_gatt_attr *attr,
    const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
  int32_t err_code;
  uint8_t *data_array = (uint8_t *)buf;

  //// Debug prints
  printk("\nReceived data From Android :  ");

  for (int i = 0; i < len; ++i) {
    printk("%d, ", data_array[i]);
  }
  printk("\n");

  // 1. Translate the command if client ID validation succeeds
  err_code = translate_ble_cmd(data_array, len); //, &cmd_info);

  // TODO: Handle Errors

  // Get the connection info
  if (bt_conn_get_info(conn, &conn_info) == 0) {
    // printk("Conection ID: %d\n", conn_info.id);
  }
  return len;
}

static struct bt_conn **find_curr_conn(struct bt_conn *conn) {
  int i;
  struct bt_conn **conn_found = NULL;

  for (i = 0; i < NUM_CONN_MAX; i++) {
    if (curr_conn[i] == conn) {
      conn_found = curr_conn + i;
      break;
    }
  }
  return conn_found;
}

/*
 * Brief: Callback to be called when a BLE Controller device is connected to ADO
 */
void connected(struct bt_conn *conn, uint8_t err) {
  char addr[BT_ADDR_LE_STR_LEN];
  struct bt_conn **conn_room;

  if (err) {
    printk("Connection failed (err %u)\n", err);
    return;
  } else {
    power_optimization(CONNECTED);
    conn_room = find_curr_conn(NULL);

    if (conn_room) {
      *conn_room = bt_conn_ref(conn);
      bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
      printk("Connected:  %s %d\n", addr, ++count);
    } else {
      printk("No room to trace connection!\n");
    }
  }

  // Show Start page
  // chg_display_page(DISP_START_PAGE);
  // chg_display_page(DISP_SAY_DOOR_OPEN_PAGE);

  if (!(battery_low)) {
    chg_display_page(DISP_SAY_DOOR_OPEN_PAGE);
  }
}

/*
 * Brief: Callback to be called when a connnected BLE Controller device is disconnected from ADO
 */
void disconnected(struct bt_conn *conn, uint8_t reason) {
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  struct bt_conn **conn_found;
  char *thread_state = NULL;
  printk("Disconnected (reason %u)\n", reason);
  printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);
  --count;
  conn_found = find_curr_conn(conn);

  if (conn_found) {
    bt_conn_unref(*conn_found);
    *conn_found = NULL;
  }

  abort_all_thread();

  // Show Waiting for connection page
  // chg_display_page(DISP_WAIT_CONN_PAGE);        //commented by ashok (state chage)

  if (!(battery_low)) {
    chg_display_page(DISP_WAIT_CONN_PAGE);
  }

  // Check the status of Calibration thread, if it is running, tell cmd_mgr to STOP it
  thread_state = k_thread_state_str(ado_calib_thrd_id,0,0);

  // if(strcmp(thread_state, "unknown") != 0)
  if ((strcmp(thread_state, "dead") != 0) && (strcmp(thread_state, "unknown") != 0)) {
    ADO_notify_cmd_mgr(COMMAND, ADO_CALIBRATION, STOP, NULL, 0);
    // Debug print
    printk("Cal_Stopped\n");
  }

  // Check the status of Installation thread, if it is running,
  // tell cmd_mgr to STOP any install operation
  thread_state = k_thread_state_str(my_tid_install,0,0);

  // if(strcmp(thread_state, "unknown") != 0)
  if ((strcmp(thread_state, "dead") != 0) && (strcmp(thread_state, "unknown") != 0)) {
    ADO_notify_cmd_mgr(COMMAND, ADO_INSTALLATION, STOP_CLAMP, NULL, 0);
    // Debug print
    printk("Ins_Stopped\n");
  }

  power_optimization(DISCONNECTED);

  // uninitialise the track keeping variables
  cmd_info.rem_msg_len = 0;
  cmd_info.curr_packet_idx = 0;
  cmd_info.total_msg_length = 0;
  cmd_info.total_pckt_count = 0;
}

/*
 * Brief: Callback to be called when the BLE controller device needs to upgrade/downgrade
 *        the security level of BLE connection via Bonding/Pairing
 */
#ifdef CONFIG_BT_LBS_SECURITY_ENABLED
void security_changed(struct bt_conn *conn, bt_security_t level,
    enum bt_security_err err) {
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  if (!err) {
    printk("Security changed: %s level %u\n", addr, level);
  } else {
    printk("Security failed: %s level %u err %d\n", addr, level, err);
  }
}
#endif

#if defined(CONFIG_BT_LBS_SECURITY_ENABLED)
/*
 * Brief: Callback that prints a random passkey on serial terminal when there is a Bonding
 *        request received
 */
void auth_passkey_display(struct bt_conn *conn, unsigned int passkey) {
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  printk("Passkey for %s: %06u\n", addr, passkey);
}

/*
 * Brief: Callback to cancel the Bonding request
 */
void auth_cancel(struct bt_conn *conn) {
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  printk("Pairing cancelled: %s\n", addr);
}

/*
 * Brief: Callback to confirm the Bonding request if the entered key at BLE controller
 *        matches or not with the generated random key by ADO
 */
void pairing_confirm(struct bt_conn *conn) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  bt_conn_auth_pairing_confirm(conn);
  printk("Pairing confirmed: %s\n", addr);
}

/*
 * Brief: Callback which is called when the Bonding is successfully completed.
 */
void pairing_complete(struct bt_conn *conn, bool bonded) {
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  printk("Pairing completed: %s, bonded: %d\n", addr, bonded);

  // Show Bonding completed page
  // chg_display_page(DISP_BT_PAIRED_PAGE);      //commented by ashok (state change)
  // printk("\n------------------ Display from bluetooth = bluetooth paired  -------------------------\n");

  if (!(battery_low)) {
    chg_display_page(DISP_BT_PAIRED_PAGE); // added by ashok
  }
}

/*
 * Brief: Callback which is called when the Bonding is failed.
 */
void pairing_failed(struct bt_conn *conn, enum bt_security_err reason) {
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  printk("Pairing failed conn: %s, reason %d\n", addr, reason);
}

static bool bms_authorize(struct bt_conn *conn,
    struct bt_bms_authorize_params *params) {
  if ((params->code_len == sizeof(bms_auth_code)) &&
      (memcmp(bms_auth_code, params->code, sizeof(bms_auth_code)) == 0)) {
    printk("Authorization of BMS operation is successful\n");
    return true;
  }
  printk("Authorization of BMS operation has failed\n");
  return false;
}

static struct bt_bms_cb bms_callbacks = {
    .authorize = bms_authorize,
};

/*
 * Brief: Function to be called to initialise the Bond Management BLE services,
 *        and intialise them to the desired parameters.
 */
static int bms_init(void) {
  struct bt_bms_init_params init_params = {0};

  /* Enable all possible operation codes */
  init_params.features.delete_requesting.supported = true;
  init_params.features.delete_rest.supported = true;
  init_params.features.delete_all.supported = true;

  /* Require authorization code for operations that
   * also delete bonding information for other devices
   * than the requesting client.
   */
  init_params.features.delete_rest.authorize = true;
  init_params.features.delete_all.authorize = true;
  init_params.cbs = &bms_callbacks;
  return bt_bms_init(&init_params);
}

/*
 * Brief: Function to be called to initialise the BLE stack, ADO BLE services,
 *        register the required callbacks annd start advertising.
 */
int initBLE(void) {
  int err;
  bt_conn_cb_register(&conn_callbacks);

  if (IS_ENABLED(CONFIG_BT_LBS_SECURITY_ENABLED)) {
    bt_conn_auth_cb_register(&conn_auth_callbacks);
  }

  // Set Fixed Passkey for authorisation
  err = bt_passkey_set(BT_fixed_passkey);
  if (err) {
    printk("Bluetooth Passkey is not set\r\n");
    return BLE_PassKey_NotSET;
  }

  // Enable BLE
  err = bt_enable(NULL);
  if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
    return BLE_Init_Fail;
  }

  printk("Bluetooth initialized\n");

  if (IS_ENABLED(CONFIG_SETTINGS)) {
    settings_load();
  }

  err = bms_init();
  if (err) {
    printk("Failed to init BMS (err:%d)\n", err);
    return Fail_BMS_init;
  }

  // Start advertising
  err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
      sd, ARRAY_SIZE(sd));

  if (err) {
    printk("Advertising failed to start (err %d)\n", err);
    return Advertising_Fail_ToStart;
  }

  printk("Advertising successfully started\n");

  // Show Startup page on display
  // chg_display_page(DISP_WAIT_CONN_PAGE);    //commented by ashok (state change)
  // printk("\n------------------ Display from bluetooth = waiting for connection  -------------------------\n");

  if (!(battery_low)) {
    chg_display_page(DISP_WAIT_CONN_PAGE);
  }
  return BLE_Init_Succcess;
}

/*
 * Brief: Function to send back he acknowledgement of command recieved, It
          must be supplied valid buffer and length
 */
int8_t ADO_sendCmdACK(uint8_t *res_buf, uint8_t len) {
  int8_t err = -1;
  if (notify_enabled) {
    err = bt_gatt_notify(NULL, &ADO_Service.attrs[2], res_buf, len);
  }
  return err;
}

/*
 * brief: This function is used to append a command packet into a global array of all the received packets in a
 *        single transfer. If the overall data to be received over BLE, is larger than the allowed BLE MTU size,
 *        we divide the whole data into several packets and transfer them in sequence. In such multi-packet Txn,
 *        this function needs to be called per packet received.
 */
int32_t packetizeDataBuffer(uint8_t *data_array) {
  uint8_t curr_pckt_idx = cmd_info.curr_packet_idx;
  uint8_t msg_len = MAX_BLE_PCKT_MSG_LEN - 2; // msg length for pckt_idx > 0, i.e MTU_LEN - sizeof(non-msg-bytes)
  uint8_t msg_index = 2;                      // msg index in non-first message

  // For preparing response message in case of packet loss
  uint8_t res_buf[MAX_BLE_PCKT_MSG_LEN] = {0};
  uint8_t len;

  // Input validations
  if (data_array == NULL) {
    // Log message
    return -1;
  }

  // Check if max pckt index is reached
  if (curr_pckt_idx >= MAX_BLE_DATA_PACKETS) {
    printk("Error: Reached maximum BLE data Rx limit\r\n");
    return -2;
  }

  cmd_packets[curr_pckt_idx].pckt_num = data_array[0];
  cmd_packets[curr_pckt_idx].msg_id = data_array[1];

  if (curr_pckt_idx == 0) {
    // 1. Retrieve the command specific (one time) info.
    cmd_info.client_id = (data_array[2] << 8) + data_array[3];
    cmd_info.total_pckt_count = data_array[4];
    cmd_info.total_msg_length = (data_array[5] << 8) + data_array[6];
    cmd_info.cmd_type = data_array[7];

    // Set the track keeping variables
    cmd_info.rem_msg_len = cmd_info.total_msg_length;
    msg_len = MAX_BLE_PCKT_MSG_LEN - BLE_CMD_PREAMBLE_LEN;
    msg_index = BLE_CMD_PREAMBLE_LEN;
  }

  // check msg_length for last chunk
  if (cmd_info.rem_msg_len < msg_len) {
    msg_len = cmd_info.rem_msg_len;
  }

  // copy the data chunk into a byte array in structure
  memcpy(&(cmd_packets[curr_pckt_idx].msg_byte), &(data_array[msg_index]), msg_len);

  // Check if we received an unexpected packet sequence
  if (cmd_packets[curr_pckt_idx].pckt_num != curr_pckt_idx + 1) { // since pckt_idx + 1 = pckt_num
    // Set the respective response structure variables
    res_pckt.req_id = cmd_packets[0].msg_id;
    res_pckt.client_id = 1U; // To-Do: Modify the client ID value as per the client identification logic TBD later
    res_pckt.total_pckt_cnt = 1U;
    res_pckt.total_msg_len = 5U; // Only valid for sending single packet failure
    res_pckt.res_type = ADO_COMMAND_RESPONSE;
    res_pckt.res_of_cmd = cmd_info.cmd_type;
    res_pckt.res_status = FRAGMENT_ERROR;
    res_pckt.failed_pckt_cnt = 1U; // Only valid for sending single packet failure
    res_pckt.failed_pckt_idxs[res_pckt.failed_pckt_cnt] = cmd_packets[curr_pckt_idx].pckt_num;

    // Debug print
    printk("Out of order packet rcvd: req_id:%d, res_type:%d, res_of_cmd:%d\n", res_pckt.req_id, res_pckt.res_type, res_pckt.res_of_cmd);
    // Bufferize response packet and send the notification
    len = bufferizeResPacket(res_buf);
    if (len > 0) {
      if (ADO_sendCmdACK(res_buf, len) == 0) {
        // TODO: Clear the response structure in case of multiple packet failure
      }
    }
  }

  // Do the required track keeping before returning
  // 1. Increment the current packet index for next packet
  cmd_info.curr_packet_idx = cmd_info.curr_packet_idx + 1;

  // 2. Decrement the remaining message length
  cmd_info.rem_msg_len = cmd_info.rem_msg_len - msg_len;
  return cmd_info.rem_msg_len;
}

/*
 * brief: This function receives a structure of response packet
 *        and converts it to a byte array to be notified as an
 *        ACK to received command over BLE.
 */
uint8_t bufferizeResPacket(uint8_t *res_buf) {
  uint8_t res_pckt_num = 1U; // first packet index
  uint8_t rem_packet_len = MAX_BLE_PCKT_MSG_LEN;
  uint8_t res_len = 0U;

  // Prepare the first buffer from packet1 in following sequence
  // [pckt_no(1), req_id(1), client_id(2), total_pckt(1), msg_len(2), [msg_type(1), cmd_type(1), res_status(1), failed_count(1), failed_buf[]]]
  res_buf[0] = res_pckt_num;
  res_buf[1] = res_pckt.req_id;
  res_buf[2] = (uint8_t)(res_pckt.client_id >> 8); // First byte of client_id
  res_buf[3] = (uint8_t)res_pckt.client_id;        // Second byte of client_id
  res_buf[4] = res_pckt.total_pckt_cnt;
  res_buf[5] = (uint8_t)(res_pckt.total_msg_len >> 8); // First byte of msg_len
  res_buf[6] = (uint8_t)res_pckt.total_msg_len;        // Second byte of msg_len
  res_buf[7] = res_pckt.res_type;
  res_buf[8] = res_pckt.res_of_cmd;
  res_buf[9] = res_pckt.res_status;
  res_buf[10] = res_pckt.failed_pckt_cnt;

  // Update the remaining length to be bufferised
  rem_packet_len = MAX_BLE_PCKT_MSG_LEN - 10; // since we have consumed 10 bytes of buf_1 so far
  res_len = 11;

  // If there are any missed packets, copy an array of their pckt_number's
  if (res_pckt.failed_pckt_cnt) {
    // Now check if the current buf is sufficient or else create new buffers and fill them
    if (res_pckt.failed_pckt_cnt <= rem_packet_len) {
      memcpy(&(res_buf[11]), res_pckt.failed_pckt_idxs, res_pckt.failed_pckt_cnt);
      res_len = res_len + res_pckt.failed_pckt_cnt;
    } else {
      // TODO: Looping Logic to add more buffers
      // 1. increment the packet number,
      // 2. assign the request_id
      // 3. Copy the buffer
      // 4. Update pckt_num
      // 5. update the res_len[pckt_num]
    }
  }

  //// Debug prints
  // printk("Prepared res_buf \n");
  // for (int l = 0; l < res_len; l++)
  // printk("%d, ", res_buf[l]);
  // printk("\n");
  return res_len;
}

/*
 * brief: This function receives a structure of response packet
 *        and converts it to a byte array to be notified as an
 *        ACK to received command over BLE.
 */
uint8_t bufferizeEvtPacket(ado_event_status_t evt_status, uint8_t *res_buf) {
  uint8_t evt_pckt_num = 1U; // first packet index
  uint8_t evt_len = 0U;

  // Calculate the total event message length
  evt_header.total_msg_len = evt_status.evt_msg_len + REG_EVT_NOTIFY_MSG_LEN;

  // Prepare the first buffer from packet1 in following sequence
  // [pckt_no(1), req_id(1), client_id(2), total_pckt(1), msg_len(2), [msg_type(1), cmd_type(1), res_status(1), failed_count(1), failed_buf[]]]
  res_buf[0] = evt_pckt_num;
  res_buf[1] = res_pckt.req_id;
  res_buf[2] = conn_info.id >> 8;             // First byte of client_id
  res_buf[3] = conn_info.id;                  // Second byte of client_id, contains the BT client ID of connected client
  res_buf[4] = 1U;                            // we are sending only one packet so far
  res_buf[5] = evt_header.total_msg_len >> 8; // First byte of msg_len
  res_buf[6] = evt_header.total_msg_len;      // Second byte of msg_len
  res_buf[7] = evt_status.cmd_type;
  res_buf[8] = evt_status.cmd;
  res_buf[9] = evt_status.res_status;

  evt_len = 10;

  if (evt_status.evt_msg_len > 0) {
    memcpy(&res_buf[10], &evt_status.evt_msg[0], evt_status.evt_msg_len);
    evt_len = evt_len + evt_status.evt_msg_len;
  }

  // Debug prints
  // printk("Prepared evt_buf \n");
  // for (int l = 0; l < evt_len; l++)
  // printk("%d, ", res_buf[l]);
  // printk("\n");
  return evt_len;
}

/*
 * brief: This function receives a byte array with its length
 *        and converts it into a global array of multiple packets
 *        if there are any. If more than one packet, this function
 *        needs to be called always, whenever there is a new data
 *        buffer has been received in sequence to previous buffer.
 *        Once received all packets, this function notifies the
 *        command manager thread for further action.
 */
int translate_ble_cmd(uint8_t *data_array, uint16_t data_length) {
  int32_t ret;
  uint8_t res_buf[MAX_BLE_PCKT_MSG_LEN] = {0};
  uint8_t len, err = 0;
  struct k_mbox_msg ble_send_msg;
  ado_cmd_mgr_msg_t cmd_mgr_msg;

  // Fill in the structure with received values in buffer
  ret = packetizeDataBuffer(data_array);

  if (ret < 0) {
    printk("Error: Packetisation failed (err: %d)\r\n", ret);
    return ret;
  } else if (ret) {
    printk("Remaining length : %d)\r\n", ret);
    return ret;
  }
  // No more data to read, we can go ahead translating now,

  // Prepare the SUCCESS acknowledgement
  res_pckt.req_id = cmd_packets[0].msg_id;
  res_pckt.client_id = conn_info.id; // To-Do: Modify the client ID value as per the client identification logic TBD later
  res_pckt.total_pckt_cnt = 1U;      // To be filled later
  res_pckt.total_msg_len = 4U;
  res_pckt.res_type = ADO_COMMAND_RESPONSE;
  res_pckt.res_of_cmd = cmd_info.cmd_type;
  res_pckt.res_status = SUCCESS;
  res_pckt.failed_pckt_cnt = 0U;

  len = bufferizeResPacket(res_buf);

  if (len > 0) {
    ADO_sendCmdACK(res_buf, len);
  }

  // Prepare the cmd_mgr_msg structure
  cmd_mgr_msg.msg_type = COMMAND;
  cmd_mgr_msg.msg_cmd = cmd_info.cmd_type;

  // Copy the data buffer, for now we are sending the first packet data only
  if (cmd_info.total_msg_length <= MAX_BLE_PCKT_MSG_LEN) {
    cmd_mgr_msg.msg_buf_len = cmd_info.total_msg_length - 1; // msg_type + msg_cmd = 2 bytes
    memcpy(cmd_mgr_msg.msg_buf, cmd_packets[0].msg_byte, cmd_mgr_msg.msg_buf_len);
  } else {
    cmd_mgr_msg.msg_buf_len = MAX_BLE_PCKT_MSG_LEN;
    memcpy(cmd_mgr_msg.msg_buf, cmd_packets[0].msg_byte, cmd_mgr_msg.msg_buf_len);
    // TODO: if there are more than 1 packet in complete command, copy the complete
    // message in a buffer and send, may need to use dynamic memory

    // Debug printk
    printk("Only first packet data sent to cmd mgr\n");
  }

  // prepare a mailbox message to be sent to command manager
  ble_send_msg.info = 0;
  ble_send_msg.size = sizeof(cmd_mgr_msg);
  ble_send_msg.tx_data = &cmd_mgr_msg;
  ble_send_msg.tx_target_thread = cmd_mgr_thrd_id;

  // send the mailbox message and wait till the other thread receives the message
  // printk("\n k_mbox_put send");
  err = k_mbox_put(&cmd_mgr_mb, &ble_send_msg, MAX_MAILBOX_MSG_SEND_TIMEOUT);
  // TODO: add retry logic when the message sending fails due to timeout.
  // It could have failed since the cmd_mgr might be waiting to send some
  // message and might not receive the new message

  if (err == 0) {
    printk("\nmsg sent successfully");
  } else if (ENOMSG == err) {
    printk("\nReturned without waiting");
  } else if (EAGAIN == err) {
    printk("\nWaiting period timed out");
  } else {
    int numb_msgs;
    numb_msgs = k_msgq_num_used_get(&cmd_mgr_mb);
    printk("\nun known  error %d number of msgs in q %d", err, numb_msgs);

    // k_msgq_purge(&ble_send_msg);
    // ble_send_msg.info = 0;
    // ble_send_msg.size = sizeof(cmd_mgr_msg);
    // ble_send_msg.tx_data = &cmd_mgr_msg;
    // ble_send_msg.tx_target_thread = cmd_mgr_thrd_id;

    for (int i = 0; i <= 1000000; i++) {
      __NOP();
    }
    err = k_mbox_put(&cmd_mgr_mb, &ble_send_msg, MAX_MAILBOX_MSG_SEND_TIMEOUT);
    numb_msgs = k_msgq_num_used_get(&cmd_mgr_mb);
    printk("\nun known  error %d number of msgs in q %d", err, numb_msgs);
  }
  // uninitialise the track keeping variables
  cmd_info.rem_msg_len = 0;
  cmd_info.curr_packet_idx = 0;
  cmd_info.total_msg_length = 0;
  cmd_info.total_pckt_count = 0;
  return ret;
}

/*
 * brief: This thread is going to be used to notify any data over
 *        BLE to ED. This threads keeps waiting to receive any msg
 *        in prescribed structure format from cmd manager thread
 */
void ADO_BleNotify_Thread(void) {
  // Mailbox variables
  struct k_mbox_msg recv_msg;
  ado_cmd_mgr_msg_t ble_notify_msg;
  uint8_t res_type = 0U;

  // Initialize the mailbox
  k_mbox_init(&ble_notify_mb);

  while (1) {
    // prepare the mailbox receive buffer
    recv_msg.info = 0;
    recv_msg.size = sizeof(ado_cmd_mgr_msg_t);
    recv_msg.rx_source_thread = cmd_mgr_thrd_id;

    // retrieve and delete the message if recieved
    k_mbox_get(&ble_notify_mb, &recv_msg, &ble_notify_msg, K_FOREVER);

    // debug prints
    // printk("Recvd MB message in ble_notify:");
    // for(int i = 0; i < ble_notify_msg.msg_buf_len; ++i)
    //    printk("%d, ", ble_notify_msg.msg_buf[i]);
    //    printk("\n");

    res_type = ble_notify_msg.msg_cmd;
    switch (res_type) {
    case BLE_NOTIFY:
      // Notify the received message buf over BLE
      if (ble_notify_msg.msg_buf_len > 0) {
        ADO_sendCmdACK(ble_notify_msg.msg_buf, ble_notify_msg.msg_buf_len);
        // TODO: Handle errors
      }
      res_type = 0U;
      break;

    default:
      res_type = 0U;
      break;
    }
  }
}

/*
 * brief: This function takes the required parameters to be notified
 *        over BLE from any thread and prepares a message to the dest-
 *        ination thread and post it to that thread's mailbox. The
 *        destination thread as per our architecture is command manager,
 *        but in case if the message is being sent from cmd_mgr thread,
 *        itself, the destination thread will be ble_notify_thread.
 */
void ADO_notify_ble(k_tid_t dest_thrd_id, uint8_t cmd_type, uint8_t cmd, uint8_t status, uint8_t *evt_msg, uint8_t evt_msg_len) {
  // Structure to hold the received parameter values
  ado_event_status_t evt_stat;
  uint8_t len = 0;
  int err = 0;
  struct k_mbox *dest_mbox = NULL;

  // Mailbox variables
  struct k_mbox_msg ble_send_msg = {0};
  ado_cmd_mgr_msg_t cmd_mgr_msg = {0};

  // prepare evt_status structure
  evt_stat.cmd_type = cmd_type;
  evt_stat.cmd = cmd;
  evt_stat.res_status = (ado_response_status_t)status;
  evt_stat.evt_msg_len = evt_msg_len;

  if (evt_msg_len > 0) {
    memcpy(evt_stat.evt_msg, evt_msg, evt_msg_len);
  }

  // Prepare a message for the command manager thread
  cmd_mgr_msg.msg_type = RESPONSE;
  cmd_mgr_msg.msg_cmd = BLE_NOTIFY;

  // Bufferise the event status
  len = bufferizeEvtPacket(evt_stat, cmd_mgr_msg.msg_buf);
  if (len > 0) {
    cmd_mgr_msg.msg_buf_len = len;
  }

  // prepare a mailbox message to be sent to command manager
  ble_send_msg.info = 0;
  ble_send_msg.size = sizeof(cmd_mgr_msg);
  ble_send_msg.tx_data = &cmd_mgr_msg;
  ble_send_msg.tx_target_thread = cmd_mgr_thrd_id;
  dest_mbox = &cmd_mgr_mb;

  // check if the destination thread is ble_notify and change the dest_mbox and thrd_id if yes
  if (dest_thrd_id == ble_notify_thrd_id) {
    ble_send_msg.tx_target_thread = ble_notify_thrd_id;
    dest_mbox = &ble_notify_mb;
  }

  // debug prints
  printk("Sent MB message:");
  for (int i = 0; i < cmd_mgr_msg.msg_buf_len; ++i) {
    printk("%d, ", cmd_mgr_msg.msg_buf[i]);
    // printk("\n");
  }

  // send the mailbox message and wait till the other thread receives the message or MAX_MAILBOX_MSG_SEND_TIMEOUT whichever earlier
  err = k_mbox_put(dest_mbox, &ble_send_msg, MAX_MAILBOX_MSG_SEND_TIMEOUT);
  if (err < 0) {
    printk("MB send failed, err %d\n", err);
  }
}

#endif