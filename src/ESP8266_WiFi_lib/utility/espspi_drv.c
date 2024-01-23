#include <errno.h>
#include <stdio.h>
#include <device.h>
#include <kernel.h>
#include <stdlib.h>
#include <zephyr.h>
#include <inttypes.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <devicetree.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>
#include <hal/nrf_gpio.h>

///*
//  espspi_drv.cpp - Library for Arduino SPI connection with ESP8266
  
//  Copyright (c) 2017 Jiri Bilek. All right reserved.

//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.

//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.

//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//*/

#include "../config.h"
#include "espspi_drv.h"
#include "espspi_proxy.h"

// Read and check one byte from the input
#if defined(ESPSPI_DEBUG_OPTION_BAD_MESSAGE)
#define READ_AND_CHECK_BYTE(c, err)                         \
            do {                                       \
                uint8_t _b = espSpiProxy.readByte();   \
                if (_b != (c)) {                       \
                    Serial.print(err);                 \
                    Serial.print(" exp:");  Serial.print(c, HEX);    \
                    Serial.print(", got:");  Serial.print(_b, HEX);  \
                    return 0;                          \
                }                                      \
            } while (false)
#else
#define READ_AND_CHECK_BYTE(c, err)           \
            do {                              \
                uint8_t _b = readByte();      \
                if (_b != (c)) {              \
                  printk("%s expected: %X, got: %X \n", err, c, _b); \
                  return 0;    \
                  }                \
            } while (false)
#endif
/* cmd by samuel */
//#define READ_AND_CHECK_BYTE(c, err)                   \
//            do {                                     \
//                //if (espSpiProxy.readByte() != (c))   \
//                if (readByte() != (c))  // added by samuel \
//                  return 0;                        \
//            } while (false)
//#endif

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
int8_t EspSpiDrv_sendCmd(const uint8_t cmd, const uint8_t numParam)
{
    //WAIT_FOR_SLAVE_RX_READY();
    //espSpiProxy.waitForSlaveRxReady();
    waitForSlaveRxReady();  // added by samuel

    // Send Spi START CMD
    //espSpiProxy.writeByte(START_CMD);
    writeByte(START_CMD);  // added by samuel

    // Send Spi C + cmd
    //espSpiProxy.writeByte(cmd & ~(REPLY_FLAG));
    writeByte(cmd & ~(REPLY_FLAG));  // added by samuel

    // Send Spi numParam
    //espSpiProxy.writeByte(numParam);
    writeByte(numParam);  // added by samuel

    // If numParam == 0 send END CMD and flush
    if (numParam == 0)
    {
      EspSpiDrv_endCmd();
    }
}

/*
   Ends a command and flushes the buffer
 */
void EspSpiDrv_endCmd()
{
    //espSpiProxy.writeByte(END_CMD);
    writeByte(END_CMD);  // added by samuel
    //espSpiProxy.flush(MESSAGE_FINISHED);
    flush(MESSAGE_FINISHED);  // added by samuel
}

/*
    Sends a parameter.
    param ... parameter value
    param_len ... length of the parameter
 */
void EspSpiDrv_sendParam(const uint8_t* param, const uint8_t param_len)
{
    // Send paramLen
    //espSpiProxy.writeByte(param_len);
    writeByte(param_len);  // added by samuel

    // Send param data
    for (int i=0; i<param_len; ++i)
    {
      //espSpiProxy.writeByte(param[i]);
      writeByte(param[i]);  // added by samuel
    }
}

/*
    Sends a 8 bit integer parameter. Sends high byte first.
    param ... parameter value
 */
void EspSpiDrv_sendParam_1byte(const uint8_t param)
{
    // Send paramLen
    //espSpiProxy.writeByte(1);
    writeByte(1);  // added by samuel

    // Send param data
    //espSpiProxy.writeByte(param );
    writeByte(param);  // added by samuel
}


/*
    Sends a buffer as a parameter.
    Parameter length is 16 bit.
 */
void EspSpiDrv_sendBuffer(const uint8_t* param, uint16_t param_len)
{
    // Send paramLen
    //espSpiProxy.writeByte(param_len & 0xff);
    writeByte(param_len & 0xff);  // added by samuel

    //espSpiProxy.writeByte(param_len >> 8);
    writeByte(param_len >> 8);  // added by samuel

    // Send param data
    for (uint16_t i=0;  i<param_len;  ++i)
    {
      //espSpiProxy.writeByte(param[i]);
      writeByte(param[i]);  // added by samuel
    }
}


/*
    Gets a response from the ESP
    cmd ... command id
    numParam ... number of parameters - currently supported 0 or 1
    param  ... pointer to space for the first parameter
    param_len ... max length of the first parameter, returns actual length
 */
uint8_t EspSpiDrv_waitResponseCmd(const uint8_t cmd, uint8_t numParam, uint8_t* param, uint8_t* param_len)
{
    //WAIT_FOR_SLAVE_TX_READY();
    //espSpiProxy.waitForSlaveTxReady();
    waitForSlaveTxReady();  // added by samuel

    READ_AND_CHECK_BYTE(START_CMD, "Start");
    READ_AND_CHECK_BYTE(cmd | REPLY_FLAG, "Cmd");
    READ_AND_CHECK_BYTE(numParam, "Param");

    if (numParam == 1)
    {
      // Reads the length of the first parameter
      //uint8_t len = espSpiProxy.readByte();
      uint8_t len = readByte();  // added by samuel

      // Reads the parameter, checks buffer overrun
      for (uint8_t ii=0; ii<len; ++ii)
      {
        if (ii < *param_len)
        {
          //param[ii] = espSpiProxy.readByte();
          param[ii] = readByte();  // added by samuel
        }
      }
      // Sets the actual length of the parameter
      if (len < *param_len)
      {
        *param_len = len;
      }
    }
    else if (numParam != 0)
    {
      return 0;  // Bad number of parameters
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
uint8_t EspSpiDrv_waitResponseCmd16(const uint8_t cmd, uint8_t numParam, uint8_t* param, uint16_t* param_len)
{
    //WAIT_FOR_SLAVE_TX_READY();
    //espSpiProxy.waitForSlaveTxReady();
    waitForSlaveTxReady();  // added by samuel

    READ_AND_CHECK_BYTE(START_CMD, "Start");
    READ_AND_CHECK_BYTE(cmd | REPLY_FLAG, "Cmd");
    READ_AND_CHECK_BYTE(numParam, "Param");

    if (numParam == 1)
    {
        // Reads the length of the first parameter
        //uint16_t len = espSpiProxy.readByte() << 8;
        //len |= espSpiProxy.readByte();
        uint16_t len = readByte() << 8;
        len |= readByte();
        // Reads the parameter, checks buffer overrun
        for (uint16_t ii=0; ii<len; ++ii)
        {
            if (ii < *param_len)
                //param[ii] = espSpiProxy.readByte();
                param[ii] = readByte();
        }

        // Sets the actual length of the parameter
        if (len < *param_len)
            *param_len = len;
    }
    else if (numParam != 0)
        return 0;  // Bad number of parameters
  
    READ_AND_CHECK_BYTE(END_CMD, "End");

    return 1;
}

/*
    Reads a response from the ESP. Decodes parameters and puts them into a return structure
 */
int8_t EspSpiDrv_waitResponseParams(const uint8_t cmd, uint8_t numParam, tParam* params)
{
    //WAIT_FOR_SLAVE_TX_READY();
    //espSpiProxy.waitForSlaveTxReady();
    waitForSlaveTxReady();  // added by samuel

    READ_AND_CHECK_BYTE(START_CMD, "Start");
    READ_AND_CHECK_BYTE(cmd | REPLY_FLAG, "Cmd");
    READ_AND_CHECK_BYTE(numParam, "Param");

    if (numParam > 0)
    {
      for (uint8_t i=0;  i<numParam;  ++i)
      {
        // Reads the length of the first parameter
        //uint8_t len = espSpiProxy.readByte();
        uint8_t len = readByte();  // added by samuel

        // Reads the parameter, checks buffer overrun
        for (uint8_t ii=0; ii<len; ++ii)
        {
          if (ii < params[i].paramLen)
          {
            //params[i].param[ii] = espSpiProxy.readByte();
            params[i].param[ii] = readByte();  // added by samuel
          }
        }
        // Sets the actual length of the parameter
        if (len < params[i].paramLen)
        {
          params[i].paramLen = len;
        }
      }
    }
    READ_AND_CHECK_BYTE(END_CMD, "End");

    return 1;
}

