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

#include "utility/wl_definitions.h"
#include "utility/wl_types.h"
#include <string.h>

#include "ESP32WiFiSpi.h"
#include "ESP32WiFiSpiClient.h"
#include "utility/ESP32srvspi_drv.h"
#include "utility/ESP32wifispi_drv.h"
#include "utility/wifi_spi.h"

uint8_t _sock = SOCK_NOT_AVAIL;

/*
 *
 */
int WiFiSpiClient_connect(const char *host, uint16_t port, bool *stop_flag) {
  IPAddress remote_addr;
  if (WiFiSpi_hostByName(host, &remote_addr, stop_flag)) {
    printk("WiFiSpi_hostByName success %d.%d.%d.%d\n", remote_addr.first_byte,
        remote_addr.second_byte, remote_addr.third_byte, remote_addr.fourth_byte);
    return WiFiSpiClient__connect(remote_addr, port, false, stop_flag);
  }
  return 0;
}

/*
 *
 */
int WiFiSpiClient_connect_With_Ip(IPAddress ip, uint16_t port, bool *stop_flag) {
  return WiFiSpiClient__connect(ip, port, false, stop_flag);
}

int WiFiSpiClient_connect_With_URL(const char *url, uint16_t port, bool *stop_flag) {
  return WiFiSpiClient__connect_url(url, port, true, stop_flag);
}

/*
 *
 */
int WiFiSpiClient__connect(IPAddress ip, uint16_t port, bool isSSL, bool *stop_flag) {
  _sock = WiFiSpi_getSocket();

  uint32_t ipp = ((ip.first_byte) | (ip.second_byte << 8) |
                  (ip.third_byte << 16) | (ip.fourth_byte << 24));

  if (_sock != SOCK_NOT_AVAIL) {
    if (!ServerSpiDrv_startClient(ipp, port, _sock,
            (isSSL ? TCP_MODE_WITH_TLS : TCP_MODE), stop_flag)) {
      return 0; // unsuccessfull
    }
    _state[_sock] = _sock;
  } else {
    // printk("No Socket available\n");
    return 0;
  }
  return 1;
}

int WiFiSpiClient__connect_url(const char *url, uint16_t port, bool isSSL, bool *stop_flag) {
  _sock = WiFiSpi_getSocket();

  if (_sock != SOCK_NOT_AVAIL) {
    if (!ServerSpiDrv_startClientURL(url, port, _sock,
            (isSSL ? TCP_MODE_WITH_TLS : TCP_MODE), stop_flag)) {
      return 0; // unsuccessfull
    }
    _state[_sock] = _sock;
  } else {
    // printk("No Socket available\n");
    return 0;
  }
  return 1;
}
/*
 *
 */
size_t WiFiSpiClient_write(const uint8_t *buf, size_t size, bool *stop_flag) {
  if (_sock >= MAX_SOCK_NUM || size == 0) {
    return 0;
  }

  if (!ServerSpiDrv_sendData(_sock, buf, size, stop_flag)) {
    return 0;
  }
  return size;
}

/* extra function added by samuel */
size_t WiFiSpiClient_write_in(const char *str, bool *stop_flag) {
  return WiFiSpiClient_write((const uint8_t *)(str), strlen(str), stop_flag);
}

/*
 *
 */
int WiFiSpiClient_available(bool *stop_flag) {
  if (_sock == SOCK_NOT_AVAIL) {
    return 0;
  } else {
    return ServerSpiDrv_availData(_sock, stop_flag);
  }
}

/*
 *
 */
int WiFiSpiClient_read() {
  int16_t b;
  ServerSpiDrv_getData(_sock, &b, 0); // returns -1 when error
  return b;
}

/*
    Reads data into a buffer.
    Return: 0 = success, size bytes read
           -1 = error (either no data or communication error)
 */
int WiFiSpiClient_read_buff(uint8_t *buf, size_t size, bool *stop_flag) {
  // sizeof(size_t) is architecture dependent
  // but we need a 16 bit data type here
  uint16_t _size = size;
  if (!ServerSpiDrv_getDataBuf(_sock, buf, &_size, stop_flag)) {
    printk("--------- error either data or communication error WiFiSpiClient_read_buff ------\n");
    return -1;
  }
  // printk("--------- no error in reading buffer in WiFiSpiClient_read_buff ------\n");
  return 0;
}

/*
 *
 */
void WiFiSpiClient_stop(bool *stop_flag) {
  if (_sock == SOCK_NOT_AVAIL) {
    return;
  }

  ServerSpiDrv_stopClient(_sock, stop_flag);
  int count = 0;
  // wait maximum 5 secs for the connection to close
  while (WiFiSpiClient_status(stop_flag) != CLOSED && ++count < 5) {
    k_msleep(50U);
  }

  if (_server_port[_sock] == 0) {
    _state[_sock] = NA_STATE; // Close only if it isn't server connection
  }
  _sock = SOCK_NOT_AVAIL;
}

/*
 *
 */
uint8_t WiFiSpiClient_connected(bool *stop_flag) {
  if (_sock == SOCK_NOT_AVAIL) {
    return 0;
  } else {
    return (WiFiSpiClient_status(stop_flag) == ESTABLISHED);
  }
}

/*
 *
 */
uint8_t WiFiSpiClient_status(bool *stop_flag) {
  if (_sock == SOCK_NOT_AVAIL) {
    return CLOSED;
  } else {
    return ServerSpiDrv_getClientState(_sock, stop_flag);
  }
}