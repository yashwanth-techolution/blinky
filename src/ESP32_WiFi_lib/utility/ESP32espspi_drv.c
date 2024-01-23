#include "ESP32espspi_drv.h"
#include "Includes/ADO_WiFi_Module.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <errno.h>
#include <hal/nrf_gpio.h>
#include <inttypes.h>
#include <zephyr/kernel.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define CMD_BYTES 4
#define DATA_BUFF_SIZE 16384

// Message indicators
#define MESSAGE_FINISHED 0xDF
#define MESSAGE_CONTINUES 0xDC

uint8_t _ss_pin;
uint8_t buffer[DATA_BUFF_SIZE] = {0};
uint16_t buflen;
uint16_t bufpos;
extern bool wifi_connection_begin_flag;

// Read and check one byte from the input

#define READ_AND_CHECK_BYTE(c, err)                      \
  do {                                                   \
    uint8_t _b = readByte();                             \
    if (_b != (c)) {                                     \
      printk("%s expected: %X, got: %X \n", err, c, _b); \
      return 0;                                          \
    }                                                    \
  } while (false)

/*
    Sends a command to ESP. If numParam == 0 ends the command otherwise keeps it open.

    Cmd Struct Message
   _______________________________________________________________________
  | START CMD | C/R  | CMD  | N.PARAM | PARAM LEN | PARAM  | .. | END CMD |
  |___________|______|______|_________|___________|________|____|_________|
  |   8 bit   | 1bit | 7bit |  8bit   |   8bit    | nbytes | .. |   8bit  |
  |___________|______|______|_________|___________|________|____|_________|

  The last byte (position 31) is crc8.
*/
void EspSpiDrv_sendCmd(const uint8_t cmd, const uint8_t numParam) {
  // waitForSlaveRxReady();

  // Send Spi START CMD
  writeByte(START_CMD);

  // Send Spi C + cmd
  writeByte(cmd & ~(REPLY_FLAG));

  // Send Spi numParam
  writeByte(numParam);

  // If numParam == 0 send END CMD and flush
  if (numParam == 0) {
    EspSpiDrv_endCmd();
  }
}

/*
   Ends a command and flushes the buffer
 */
void EspSpiDrv_endCmd() {
  writeByte(END_CMD);
  flush(MESSAGE_FINISHED);
}

/*
    Sends a parameter.
    param ... parameter value
    param_len ... length of the parameter
 */
void EspSpiDrv_sendParam(const uint8_t *param, const uint8_t param_len) {
  // Send paramLen
  writeByte(param_len);

  // Send param data
  for (int i = 0; i < param_len; ++i) {
    writeByte(param[i]);
  }
}

/*
    Sends a 8 bit integer parameter. Sends high byte first.
    param ... parameter value
 */
void EspSpiDrv_sendParam_1byte(const uint8_t param) {
  // Send paramLen
  writeByte(1);

  // Send param data
  writeByte(param);
}

/*
    Sends a buffer as a parameter.
    Parameter length is 16 bit.
 */
void EspSpiDrv_sendBuffer(const uint8_t *param, uint16_t param_len) {
  // Send paramLen
  writeByte(param_len & 0xff);
  writeByte(param_len >> 8);
  writeByte_buf(param, param_len);
}

/*
    Gets a response from the ESP
    cmd ... command id
    numParam ... number of parameters - currently supported 0 or 1
    param  ... pointer to space for the first parameter
    param_len ... max length of the first parameter, returns actual length
 */
uint8_t EspSpiDrv_waitResponseCmd(const uint8_t cmd, uint8_t numParam,
    uint8_t *param, uint8_t *param_len, bool *stop_flag) {
  int8_t err = 0;
  err = waitForSlaveTxReady(stop_flag);

  if (err == -3) {
    // TODO: add proper return type.
    return 0;
  } else if (err) {
    // TODO: add proper return type.
    return 1;
  }

  READ_AND_CHECK_BYTE(START_CMD, "Start");
  READ_AND_CHECK_BYTE(cmd | REPLY_FLAG, "Cmd");
  READ_AND_CHECK_BYTE(numParam, "Param");

  if (numParam == 1) {
    // Reads the length of the first parameter
    uint8_t len = readByte();

    // Reads the parameter, checks buffer overrun
    for (uint8_t ii = 0; ii < len; ++ii) {
      if (ii < *param_len) {
        param[ii] = readByte();
      }
    }

    // Sets the actual length of the parameter
    if (len < *param_len) {
      *param_len = len;
    }
  } else if (numParam != 0) {
    return 0; // Bad number of parameters
  }
  READ_AND_CHECK_BYTE(END_CMD, "End");
  return 1;
}

/*
    Gets a response from the ESP
    cmd ... command id
    numParam ... number of parameters - currently supported 0 or 1
    param  ... pointer to space for the first parameter
    param_len ... max length of the first parameter (16 bit integer), returns actual length
 */
uint8_t EspSpiDrv_waitResponseCmd16(const uint8_t cmd, uint8_t numParam,
    uint8_t *param, uint16_t *param_len, bool *stop_flag) {
  int8_t err = 0;
  err = waitForSlaveTxReady(stop_flag);

  if (err == -3) {
    // TODO: add proper return type.
    return 0;
  } else if (err) {
    // TODO: add proper return type.
    return 1;
  }

  READ_AND_CHECK_BYTE(START_CMD, "Start");
  READ_AND_CHECK_BYTE(cmd | REPLY_FLAG, "Cmd");
  READ_AND_CHECK_BYTE(numParam, "Param");

  if (numParam == 1) {
    // Reads the length of the first parameter
    uint16_t len = readByte() << 8;
    len |= readByte();

    if (len < *param_len) {
      readByte_buff(param, *param_len);
      *param_len = len;
    } else {
      readByte_buff(param, len);
    }
  } else if (numParam != 0) {
    return 0; // Bad number of parameters
  }
  READ_AND_CHECK_BYTE(END_CMD, "End");
  return 1;
}

/*
    Reads a response from the ESP. Decodes parameters and puts them into a return structure
 */
int8_t EspSpiDrv_waitResponseParams(const uint8_t cmd, uint8_t numParam,
    tParam *params, bool *stop_flag) {
  int8_t err = 0;
  err = waitForSlaveTxReady(stop_flag);

  if (err == -3) {
    // TODO: add proper return type.
    return 0;
  } else if (err) {
    // TODO: add proper return type.
    return 1;
  }

  READ_AND_CHECK_BYTE(START_CMD, "Start");
  READ_AND_CHECK_BYTE(cmd | REPLY_FLAG, "Cmd");
  READ_AND_CHECK_BYTE(numParam, "Param");

  if (numParam > 0) {
    for (uint8_t i = 0; i < numParam; ++i) {
      // Reads the length of the first parameter
      uint8_t len = readByte();

      // Reads the parameter, checks buffer overrun
      for (uint8_t ii = 0; ii < len; ++ii) {
        if (ii < params[i].paramLen) {
          params[i].param[ii] = readByte();
        }
      }

      // Sets the actual length of the parameter
      if (len < params[i].paramLen) {
        params[i].paramLen = len;
      }
    }
  }
  READ_AND_CHECK_BYTE(END_CMD, "End");
  return 1;
}

void _pulseSS(bool start) {
  if (_ss_pin >= 0) {
    if (start) {
      nrf_gpio_pin_clear(_ss_pin);
    } else {
      nrf_gpio_pin_set(_ss_pin);
    }
  }
}

void espSpiProxy_begin(uint8_t cs_pin, uint8_t hwresetPin) {
  _ss_pin = cs_pin;
  nrf_gpio_cfg_output(_ss_pin);
  nrf_gpio_cfg_output(hwresetPin);

  nrf_gpio_pin_set(_ss_pin);
  nrf_gpio_pin_set(hwresetPin);
}

void readData(uint8_t *buf, uint16_t *len) {
  uint8_t cmd_buff[CMD_BYTES] = {0xAA, 0, 0, 0};
  uint16_t *i = len;
  *i = 0;

  _pulseSS(true);
  spi_transfer_w_bytes(cmd_buff, CMD_BYTES); // the value is not important
  k_msleep(15);                              // TODO:  (tested and working time 20ms)check later to reduce time.
  spi_transfer_r_bytes(cmd_buff, CMD_BYTES); // the value is not important

  if (cmd_buff[0] == 0xAA) {
    *i = (cmd_buff[1] | (cmd_buff[2] << 8));

    if (*i) {
      memset(buf, 0, DATA_BUFF_SIZE);
      spi_transfer_r_bytes(buf, *i); // the value is not important
    }
  }
  _pulseSS(false);
}

void writeData(uint8_t *data, uint16_t len) {
  k_msleep(10); // wait untill spi buffer ready from slave side.
  uint8_t cmd_buff[CMD_BYTES] = {0, 0, 0, 0xA5};

  if (len % 4 != 0) {
    while (len % 4 != 0) {
      data[len] = 0;
      len++;
    }
  }
  cmd_buff[0] = 0x55;
  cmd_buff[1] = (len & 0xFF);
  cmd_buff[2] = ((len >> 8) & 0xFF);

  _pulseSS(true);
  spi_transfer_w_bytes(cmd_buff, CMD_BYTES);
  k_msleep(15); // TODO:  (tested and working time 40ms)check later to reduce time.

  spi_transfer_w_bytes(data, len);
  _pulseSS(false);
}

void flush(uint8_t indicator) {
  // Is buffer empty?
  if (buflen == 0) {
    return;
  }

  // Message state indicator
  buffer[0] = indicator;

  // Send the buffer
  writeData(buffer, (buflen + 1));
  buflen = 0;
}

void writeByte(uint8_t b) {
  bufpos = 0; // discard input data in the buffer

  if (buflen >= (DATA_BUFF_SIZE - 2)) {
    flush(MESSAGE_CONTINUES);
  }
  buffer[++buflen] = b;
}

void writeByte_buf(uint8_t *b, uint16_t len) {
  bufpos = 0; // discard input data in the buffer

  if ((buflen + len) >= (DATA_BUFF_SIZE - 2)) {
    // TODO: handle large data.
    flush(MESSAGE_CONTINUES);
  } else {
    buflen++;
    memcpy(&buffer[buflen], b, len);
    buflen += (len - 1);
  }
}

uint8_t readByte() {
  buflen = 0; // discard output data in the buffer

  if (bufpos >= (DATA_BUFF_SIZE - 1)) { // the buffer segment was read
    return -2;
  }

  if (bufpos == 0) { // buffer empty
    return -1;
  }
  return buffer[bufpos++];
}

uint8_t readByte_buff(uint8_t *b, uint16_t len) {
  buflen = 0; // discard output data in the buffer

  if (bufpos >= (DATA_BUFF_SIZE - 1)) { // the buffer segment was read
    return -2;
  }
  if (bufpos == 0) { // buffer empty
    return -1;
  }

  memcpy(b, &buffer[bufpos], len);
  bufpos += len;
  return 0;
}

/*
    Waits for slave transmitter ready status.
    Return: transmitter status
 */
int8_t waitForSlaveTxReady(bool *stop_flag) {
  buflen = 0; // discard output data in the buffer

  if (bufpos != 0) {
    return 0;
  }

  uint16_t size = 0;
  memset(buffer, 0, DATA_BUFF_SIZE);
  uint32_t thisTime = k_uptime_get();

  if (wifi_connection_begin_flag) {
    do {
      for (int k = 0; k < 2; k++) { // TODO: (tested value 3)
        if (stop_flag != NULL) {
          if (*stop_flag) {
            printk("xxxxxxxxxx WiFi Stop/break found xxxxxxxxxx\n");
            return -3;
          }
        }
        k_msleep(25U); // TODO:  (tested and working time 25ms)check later to reduce time.
      }

      readData(buffer, &size);
      // printk("size = %d\n", size);
      if (size != 0) {
        // for(int k = 0; k<size; k++) {
        //     printk("%02x ", buffer[k]);
        // }
        // printk("\n");
        break;
      }
    } while (k_uptime_get() - thisTime < SLAVE_TX_READY_TIMEOUT_WIFI_BEGIN); // repeat until crc is ok or 200 ms elapsed
  } else {
    do {
      for (int k = 0; k < 2; k++) { // TODO: (tested value 3)
        if (stop_flag != NULL) {
          if (*stop_flag) {
            printk("xxxxxxxxxx WiFi Stop/break found xxxxxxxxxx\n");
            return -3;
          }
        }
        k_msleep(25U); // TODO:  (tested and working time 25ms)check later to reduce time.
      }

      readData(buffer, &size);
      // printk("size = %d\n", size);
      if (size != 0) {
        // for(int k = 0; k<size; k++) {
        //     printk("%02x ", buffer[k]);
        // }
        // printk("\n");
        break;
      }
    } while (k_uptime_get() - thisTime < SLAVE_TX_READY_TIMEOUT); // repeat until crc is ok or 200 ms elapsed
  }
  // TODO: add timeout error.
  if (size == 0) {
    // TODO: return time out.
    printk("\nSlave Response Time Out\n");
    return -1;
  }

  // Check the buffer for correctness
  if (buffer[0] != MESSAGE_FINISHED && buffer[0] != MESSAGE_CONTINUES) {
    printk("incorrect message = %d \n", buffer[0]);
    return -2; // incorrect message (should not happen)
  }

  bufpos = 1;
  return 0;
}

/*
 * Puts the SS low (required for successfull boot) and resets the ESP
 */
void espSpiProxy_hardReset(int8_t hwResetPin) {
  printk("resetting ESP32..... %d\n", hwResetPin);
  nrf_gpio_pin_set(_ss_pin);
  nrf_gpio_pin_clear(hwResetPin);
  k_msleep(50U);
  nrf_gpio_pin_set(hwResetPin);
  k_msleep(1000U);
}