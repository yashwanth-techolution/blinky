#include <stdio.h>
#include <zephyr/device.h>
#include <string.h>
#include <nrfx_spi.h>
#include <zephyr/devicetree.h>
#include <hal/nrf_spi.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/flash.h>
#include <hal/nrf_common.h>
#include <zephyr/kernel.h>

#include "Includes/W25N01GV.h"
#include "Includes/ADO_Flash_Module.h"


flash_device_t flash_dev;
struct device *spi_flash;


struct spi_config spi_flash_cfg = {
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .frequency = 8000000,
};


void spi2_init(void)
{
  const char *const spiName = "SPI_2";
  spi_flash = device_get_binding(spiName);

  if (spi_flash == NULL) 
  {
    printk("Could not get %s device\n", spiName);
    return;
  }
  printk("device found\n");
}


/**
 * @brief SPI user event handler.
 * @param event
 */
//void spi_event_handler(nrfx_spi_evt_t const * p_event,void *  p_context)
//{
//    spi_xfer_done = true;
//}


static uint32_t nrf_spi_transfer_own(uint8_t* p_tx_buffer,uint8_t tx_buffer_length, uint8_t* p_rx_buffer,uint8_t rx_buffer_length)
{
  struct spi_buf tx_buf1 = {
          .buf = p_tx_buffer,
          .len = tx_buffer_length};
  struct spi_buf_set tx1 = {
          .buffers = &tx_buf1,
          .count = 1};
		 
                 
   struct spi_buf rx_buf = {
          .buf = p_rx_buffer,
          .len = rx_buffer_length};
   struct spi_buf_set rx1 = {
          .buffers = &rx_buf,
          .count = 1};

  return spi_transceive(spi_flash, &spi_flash_cfg, &tx1, &rx1); 
}


void external_flash_init()
{
    int8_t err_status = FLASH_SUCCESS;

    nrf_gpio_cfg_output(EXT_FLASH_SPI_CS);
    nrf_gpio_pin_set(EXT_FLASH_SPI_CS);

    nrf_gpio_cfg_output(EXT_FLASH_SPI_HLD);
    nrf_gpio_pin_set(EXT_FLASH_SPI_HLD);

    nrf_gpio_cfg_output(EXT_FLASH_SPI_WP);
    nrf_gpio_pin_clear(EXT_FLASH_SPI_WP);

    flash_dev.spi_txfer = nrf_spi_transfer_own;
    flash_dev.sleep = k_msleep(10);
    flash_dev.flash_size = 0x8000000;
    flash_dev.num_of_pages_per_block = 0x40;
    flash_dev.num_of_blocks = 0x400;
    flash_dev.page_size = 0x800;
    flash_dev.num_ecc_bytes_per_page = 0x40;
    
    /*initialization of external flash device*/
    err_status = w25n01gv_init(&flash_dev);
    if (err_status != FLASH_SUCCESS)
    {
      printk(" winbond flash init failed\n");
    }
    printk("winbond flash started\n");
    nrf_gpio_pin_set(EXT_FLASH_SPI_WP);
      
    //printk("winbond erase starts\n");
    /*erasing the 128kB of memory*/
    //err_status = w25n01gc_flash_erase(&flash_dev, FLASH_TEST_REGION_OFFSET, MAX_FLASH_ERASE_LEN);
    //if (err_status != FLASH_SUCCESS)
    //{
    //  printk("winbond erase fail\n");
    //}
    //printk("flash erase successful\n");
}


int8_t external_flash_block_erase(uint32_t start_add, uint32_t no_of_blocks)
{
  int8_t err_status = FLASH_SUCCESS;

  nrf_gpio_pin_clear(EXT_FLASH_SPI_WP);
  /*erasing memory before storing audio file*/
  err_status = w25n01gc_flash_erase(&flash_dev, start_add, MAX_FLASH_ERASE_LEN);
  nrf_gpio_pin_set(EXT_FLASH_SPI_WP);

  if (err_status != FLASH_SUCCESS)
  {
    printk("winbond erase fail\n");
    return -1;
  }
  return 0; 
}


int8_t external_flash_file_write(uint32_t start_add, uint8_t* write_buf, uint32_t write_len)
{
  int8_t err_status = FLASH_SUCCESS;
  
  /*erasing memory before storing audio file*/
  //err_status = w25n01gc_flash_erase(&flash_dev, start_add, AUDIO_FILE_FLASH_LENGTH);
  //if (err_status != FLASH_SUCCESS)
  //{
  //  printk("winbond erase fail\n");
  //}

  /*flash write*/
  nrf_gpio_pin_clear(EXT_FLASH_SPI_WP);
  err_status = w25n01gc_flash_write(&flash_dev, (start_add+1), write_buf, write_len);
  nrf_gpio_pin_set(EXT_FLASH_SPI_WP);
  if (err_status != FLASH_SUCCESS)
  {
    printk("winbond write fail\n");
    return -1;
  }
  return 0; 
}


int8_t external_flash_file_read(uint32_t start_add, uint8_t* read_buf, uint32_t read_len)
{
  int8_t err_status = FLASH_SUCCESS;

  nrf_gpio_pin_clear(EXT_FLASH_SPI_WP);
  err_status = w25n01gc_flash_read(&flash_dev, (start_add+1), read_buf, read_len);
  nrf_gpio_pin_set(EXT_FLASH_SPI_WP);
  if (err_status != FLASH_SUCCESS)
  {
    printk("winbond read fail\n");
    return -1;
  }
  return 0;
}
