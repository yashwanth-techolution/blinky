#ifndef __IPC_MANAGER_MODULE_H_
#define __IPC_MANAGER_MODULE_H_

#include <zephyr/types.h>

// wait for 50 ms for the sent message to be retrieved by other thread otherwise discard it.
#define MAX_MAILBOX_MSG_SEND_TIMEOUT    K_MSEC(50) 
//#define MAX_MAILBOX_MSG_RECV_TIMEOUT    K_MSEC(50) //K_MSEC(100)
#define MAX_MAILBOX_MSG_RECV_TIMEOUT    K_MSEC(100)


// Declare a mailbox, must initialise before use.
extern struct k_mbox    ble_cmd_mb;
extern struct k_mbox    ble_notify_mb;

#endif  // __IPC_MANAGER_MODULE_H_
