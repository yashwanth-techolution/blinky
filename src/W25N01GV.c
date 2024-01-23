#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <hal/nrf_gpio.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>

#include "Includes/W25N01GV.h"

const flash_list_t flash_list[] = {{"winbond w25n01gv", {0xEF, 0xAA, 0x21}}};
volatile bool spi_xfer_done;
uint8_t spi_xfer_buf[MAX_SPI_BUFFER_SIZE];
uint8_t spi_rcv_buf[MAX_SPI_BUFFER_SIZE];

int8_t detect_w25n01gv(flash_device_t *w25n01gc_flash) {
  int8_t status = FLASH_SUCCESS;
  uint8_t jedec_id[W25N01GV_JEDEC_ID_SIZE];
  status = w25n01gv_read_jedec_id(w25n01gc_flash, jedec_id, W25N01GV_JEDEC_ID_SIZE);

  if (status != FLASH_SUCCESS) {
    printk("read id failed\n");
    return status;
  }

  if ((check_jedec_id(jedec_id, flash_list[0].flash_dev_id, W25N01GV_JEDEC_ID_SIZE)) != FLASH_SUCCESS) {
    printk(" flash id is not proper\n");
    return FLASH_DETECT_FAIL;
  }
  return FLASH_SUCCESS;
}

int8_t w25n01gv_init(flash_device_t *w25n01gc_flash) {
  if (w25n01gc_flash->spi_txfer == NULL) {
    printk("invalid parameters\n");
    return FLASH_INVALID_PARAMS;
  }

  if ((detect_w25n01gv(w25n01gc_flash)) != FLASH_SUCCESS) {
    printk(" flash detect fail\n");
    return FLASH_DETECT_FAIL;
  }

  if ((set_block_protect(w25n01gc_flash, BLOCK_PROTECT_NONE)) != FLASH_SUCCESS) {
    printk("set block protection failed\n");
    return FLASH_MISC_FAILURE;
  }
}

int8_t w25n01gc_flash_read(flash_device_t *w25n01gc_flash, uint32_t addr,
    uint8_t *read_buf, uint32_t read_len) {
  int8_t status = FLASH_SUCCESS;
  uint16_t page_addr;
  uint16_t col_addr;
  uint32_t rem_len = read_len;
  uint32_t read_len_page = 0;

  if ((addr + read_len) > w25n01gc_flash->flash_size) {
    printk("invalid read address\n");
    return FLASH_INVALID_PARAMS;
  }
  page_addr = addr / w25n01gc_flash->page_size;
  col_addr = addr % w25n01gc_flash->page_size;

  while (rem_len > 0) {
    if (rem_len > (w25n01gc_flash->page_size - col_addr)) {
      read_len_page = w25n01gc_flash->page_size - col_addr;
    } else {
      read_len_page = rem_len;
    }

    status = w25n01gv_page_data_read(w25n01gc_flash, page_addr);

    if (status != FLASH_SUCCESS) {
      printk("page read failed\n");
      return status;
    }

    status = w25n01gv_read_data(w25n01gc_flash, col_addr, read_buf + (read_len - rem_len), read_len_page);

    if (status != FLASH_SUCCESS) {
      printk("data read fail\n");
      return status;
    }

    status = check_ecc(w25n01gc_flash);

    if ((status != ECC_SUCCESS_NO_CORRECTION) && (status != ECC_SUCCESS_CORRECTION)) {
      printk(" read data ecc error\n");
      return status;
    }

    rem_len -= read_len_page;
    col_addr = 0;
    page_addr++;
  }
  return status;
}

int8_t w25n01gc_flash_write(flash_device_t *w25n01gc_flash, uint32_t addr,
    uint8_t *write_buf, uint32_t write_len) {
  int8_t status = FLASH_SUCCESS;
  uint16_t page_addr;
  uint16_t col_addr;
  uint32_t rem_len = write_len;
  uint32_t write_len_page = 0;

  if ((addr + write_len) > w25n01gc_flash->flash_size) {
    printk("invalid read address\n");
    return FLASH_INVALID_PARAMS;
  }

  /* Checking fail incase there was a previous error. Not returning from this error. */
  status = check_fail(w25n01gc_flash);

  if (status != ERASE_PROGRAM_SUCESS) {
    printk("previous operation of qspi flash error\n");
  }

  page_addr = addr / (w25n01gc_flash->page_size);
  col_addr = addr % (w25n01gc_flash->page_size);

  while (rem_len > 0) {
    if (rem_len > (w25n01gc_flash->page_size - col_addr)) {
      write_len_page = (w25n01gc_flash->page_size - col_addr);
    } else {
      write_len_page = rem_len;
    }

    status = w25n01gv_load_program_data(w25n01gc_flash, true, col_addr, write_buf + (write_len - rem_len), write_len_page);

    if (status != FLASH_SUCCESS) {
      printk("load program data fail\n");
      return status;
    }

    status = w25n01gv_program_execute(w25n01gc_flash, page_addr);

    if (status != FLASH_SUCCESS) {
      printk("program execute fail\n");
      return status;
    }
    /*
     * FIXME Double wait here and in program func, mostly it wont affect since we
     * wait here, the above wait is resolved immediately after single status reg read.
     */

    /* Wait for busy bit */
    status = wait_if_busy(w25n01gc_flash, W25N01GV_BUSY_DEFAULT_TIMEOUT_MS);

    if (status != FLASH_SUCCESS) {
      if (status == FLASH_TIMEOUT) {
        printk("wait if busy timeout\n");
      } else {
        printk("wait if busy fails\n");
      }
      return status;
    }

    status = check_fail(w25n01gc_flash);

    if (status != ERASE_PROGRAM_SUCESS) {
      printk("program data operation error\n");
      return status;
    }

    rem_len -= write_len_page;
    col_addr = 0;
    page_addr++;
  }
  return FLASH_SUCCESS;
}

int8_t w25n01gc_flash_erase(flash_device_t *w25n01gc_flash, uint32_t addr, uint32_t erase_len) {
  int8_t status = FLASH_SUCCESS;
  uint16_t page_addr;
  uint32_t rem_len = erase_len; // max erase len is 128kb one time
  uint32_t erase_len_page = 0;

  if ((addr + erase_len) > w25n01gc_flash->flash_size) {
    printk("invalid read address\n");
    return FLASH_INVALID_PARAMS;
  }

  /* Checking fail incase there was a previous error. Not returning from this error. */
  status = check_fail(w25n01gc_flash);

  if (status != ERASE_PROGRAM_SUCESS) {
    printk("previous operation error\n");
  }

  page_addr = addr / w25n01gc_flash->page_size;
  erase_len_page = w25n01gc_flash->page_size - (addr % w25n01gc_flash->page_size);

  while (rem_len > 0) {
    status = w25n01gv_block_erase(w25n01gc_flash, page_addr);

    if (status != FLASH_SUCCESS) {
      printk("block erase fail\n");
      return status;
    }

    status = wait_if_busy(w25n01gc_flash, W25N01GV_BUSY_ERASE_TIMEOUT_MS);

    if (status != FLASH_SUCCESS) {
      if (status == FLASH_TIMEOUT) {
        printf("wait if busy timeout\n");
      } else {
        printk("wait of busy fails\n");
      }
      return status;
    }

    status = check_fail(w25n01gc_flash);

    if (status != ERASE_PROGRAM_SUCESS) {
      printk("error while erase operation\n");
      return status;
    }

    if (rem_len < erase_len_page) {
      break;
    }

    rem_len -= erase_len_page;
    erase_len_page = w25n01gc_flash->page_size;
    page_addr++;
  }
  return status;
}

int8_t check_jedec_id(uint8_t *jedec_id, const uint8_t *cmp_id, uint16_t len) {
  for (int32_t i = 0; i < len; i++) {
    if (jedec_id[i] != cmp_id[i]) {
      printk("ID mismatch\n");
      return FLASH_DETECT_FAIL;
    }
  }
  return FLASH_SUCCESS;
}

int8_t w25n01gv_read_jedec_id(flash_device_t *w25n01gc_flash, uint8_t *jedec_id, uint32_t len) {
  spi_transfer_t spi_xfer_data;
  int8_t status;

  memset(&spi_xfer_data, 0, sizeof(spi_transfer_t));

  spi_xfer_data.opcode = W25N01GV_JEDEC_ID;
  spi_xfer_data.dummy_cycles = W25N01GV_JEDEC_ID_DUMMY_CYCLES;
  spi_xfer_data.dummy_cyles_pos = 0;
  spi_xfer_data.rx_buf = jedec_id;
  spi_xfer_data.rx_len = len;
  spi_xfer_data.tx_len = 0;

  status = flash_spi_transfer(w25n01gc_flash, &spi_xfer_data);

  if (status != FLASH_SUCCESS) {
    printk("spi transfer failed\n");
    return status;
  }
  return status;
}

int8_t w25n01gv_read_data(flash_device_t *w25n01gc_flash, uint16_t col_addr,
    uint8_t *buf, uint16_t buf_len) {
  spi_transfer_t spi_xfer_data;
  int8_t status = FLASH_SUCCESS;
  uint8_t send_addr[2];

  send_addr[0] = (col_addr >> 8) & 0xFF;
  send_addr[1] = col_addr & 0xFF;

  memset(&spi_xfer_data, 0, sizeof(spi_transfer_t));

  /* Wait for busy bit */
  status = wait_if_busy(w25n01gc_flash, W25N01GV_BUSY_DEFAULT_TIMEOUT_MS);

  if (status != FLASH_SUCCESS) {
    if (status == FLASH_TIMEOUT) {
      printk("wait busy timeout\n");
    } else {
      printk("wait if busy fails\n");
    }
    return status;
  }

  spi_xfer_data.opcode = W25N01GV_READ;
  spi_xfer_data.dummy_cycles = W25N01GV_READ_DUMMY_CYCLES;
  spi_xfer_data.dummy_cyles_pos = 1;
  spi_xfer_data.tx_len = 0;
  spi_xfer_data.rx_len = buf_len;
  spi_xfer_data.rx_buf = buf;
  spi_xfer_data.addr_len = W25N01GV_COLUMN_ADDR_SIZE;
  spi_xfer_data.addr = send_addr;

  status = flash_spi_transfer(w25n01gc_flash, &spi_xfer_data);
  if (status != FLASH_SUCCESS) {
    printk("spi transfer failed\n");
    return status;
  }
  return status;
}

int8_t w25n01gv_page_data_read(flash_device_t *w25n01gc_flash, uint16_t page_addr) {
  spi_transfer_t spi_xfer_data;
  int8_t status = FLASH_SUCCESS;
  uint8_t send_addr[2];

  send_addr[0] = (page_addr >> 8) & 0xFF;
  send_addr[1] = page_addr & 0xFF;

  memset(&spi_xfer_data, 0, sizeof(spi_transfer_t));

  /* Wait for busy bit */
  status = wait_if_busy(w25n01gc_flash, W25N01GV_BUSY_DEFAULT_TIMEOUT_MS);
  if (status != FLASH_SUCCESS) {
    if (status == FLASH_TIMEOUT) {
      printk("wait busy timeout\n");
    } else {
      printk("wait if busy fails\n");
    }
    return status;
  }

  spi_xfer_data.opcode = W25N01GV_PAGE_DATA_READ;
  spi_xfer_data.dummy_cycles = W25N01GV_PAGE_DATA_READ_DUMMY_CYCLES;
  spi_xfer_data.dummy_cyles_pos = 0;
  spi_xfer_data.rx_len = 0;
  spi_xfer_data.tx_len = 0;
  spi_xfer_data.addr_len = W25N01GV_COLUMN_ADDR_SIZE;
  spi_xfer_data.addr = send_addr;

  status = flash_spi_transfer(w25n01gc_flash, &spi_xfer_data);
  if (status != FLASH_SUCCESS) {
    printk("spi transfer failed\n");
    return status;
  }
  return status;
}

int8_t w25n01gv_program_execute(flash_device_t *w25n01gc_flash, uint16_t page_addr) {
  spi_transfer_t spi_xfer_data;
  int8_t status = FLASH_SUCCESS;
  uint8_t send_addr[2];

  send_addr[0] = (page_addr >> 8) & 0xFF;
  send_addr[1] = page_addr & 0xFF;

  memset(&spi_xfer_data, 0, sizeof(spi_transfer_t));

  /* Wait for busy bit */
  status = wait_if_busy(w25n01gc_flash, W25N01GV_BUSY_DEFAULT_TIMEOUT_MS);
  if (status != FLASH_SUCCESS) {
    if (status == FLASH_TIMEOUT) {
      printk("wait busy timeout\n");
    } else {
      printk("wait if busy fails\n");
    }
    return status;
  }

  spi_xfer_data.opcode = W25N01GV_PROGRAM_EXECUTE;
  spi_xfer_data.dummy_cycles = W25N01GV_PROGRAM_EXECUTE_DUMMY_CYCLES;
  spi_xfer_data.dummy_cyles_pos = 0;
  spi_xfer_data.rx_len = 0;
  spi_xfer_data.tx_len = 0;
  spi_xfer_data.addr_len = W25N01GV_PAGE_ADDR_SIZE;
  spi_xfer_data.addr = send_addr;

  status = flash_spi_transfer(w25n01gc_flash, &spi_xfer_data);
  if (status != FLASH_SUCCESS) {
    printk("spi transfer failed\n");
    return status;
  }
  return status;
}

int8_t w25n01gv_load_program_data(flash_device_t *w25n01gc_flash, bool random_load, uint16_t col_addr, uint8_t *buf, uint16_t buf_len) {
  spi_transfer_t spi_xfer_data;
  int8_t status = FLASH_SUCCESS;
  uint8_t send_addr[2];

  memset(&spi_xfer_data, 0, sizeof(spi_transfer_t));
  status = w25n01gv_write_enable(w25n01gc_flash);

  if (status != FLASH_SUCCESS) {
    return status;
  }

  send_addr[0] = (col_addr >> 8) & 0xFF;
  send_addr[1] = col_addr & 0xFF;

  /* Wait for busy bit */
  status = wait_if_busy(w25n01gc_flash, W25N01GV_BUSY_DEFAULT_TIMEOUT_MS);

  if (status != FLASH_SUCCESS) {
    if (status == FLASH_TIMEOUT) {
      printk("wait busy timeout\n");
    } else {
      printk("wait if busy fails\n");
    }
    return status;
  }

  if (random_load) {
    spi_xfer_data.opcode = W25N01GV_RANDOM_PROGRAM_DATA_LOAD;
  } else {
    spi_xfer_data.opcode = W25N01GV_PROGRAM_DATA_LOAD;
  }

  spi_xfer_data.dummy_cycles = W25N01GV_PROGRAM_DATA_LOAD_DUMMY_CYCLES;
  spi_xfer_data.rx_len = 0;
  spi_xfer_data.tx_len = buf_len;
  spi_xfer_data.tx_buf = buf;
  spi_xfer_data.addr_len = W25N01GV_COLUMN_ADDR_SIZE;
  spi_xfer_data.addr = send_addr;

  status = flash_spi_transfer(w25n01gc_flash, &spi_xfer_data);

  if (status != FLASH_SUCCESS) {
    printk("spi transfer failed\n");
    return status;
  }
  return status;
}

int8_t w25n01gv_block_erase(flash_device_t *w25n01gc_flash, uint16_t page_addr) {
  spi_transfer_t spi_xfer_data;
  int8_t status = FLASH_SUCCESS;
  uint8_t send_addr[2];

  memset(&spi_xfer_data, 0, sizeof(spi_transfer_t));
  status = w25n01gv_write_enable(w25n01gc_flash);

  if (status != FLASH_SUCCESS) {
    return status;
  }

  send_addr[0] = (page_addr >> 8) & 0xFF;
  send_addr[1] = page_addr & 0xFF;

  /* Wait for busy bit */
  status = wait_if_busy(w25n01gc_flash, W25N01GV_BUSY_DEFAULT_TIMEOUT_MS);

  if (status != FLASH_SUCCESS) {
    if (status == FLASH_TIMEOUT) {
      printk("wait busy timeout\n");
    } else {
      printk("wait if busy fails\n");
    }
    return status;
  }

  spi_xfer_data.opcode = W25N01GV_BLOCK_ERASE;
  spi_xfer_data.dummy_cycles = W25N01GV_BLOCK_ERASE_DUMMY_CYCLES;
  spi_xfer_data.dummy_cyles_pos = 0;
  spi_xfer_data.rx_len = 0;
  spi_xfer_data.tx_len = 0;
  spi_xfer_data.addr_len = W25N01GV_PAGE_ADDR_SIZE;
  spi_xfer_data.addr = send_addr;

  status = flash_spi_transfer(w25n01gc_flash, &spi_xfer_data);

  if (status != FLASH_SUCCESS) {
    printk("spi transfer failed\n");
    return status;
  }

  return status;
}

int8_t w25n01gv_write_status_reg(flash_device_t *w25n01gc_flash,
    uint8_t reg_addr, uint8_t *reg_write) {
  spi_transfer_t spi_xfer_data;
  int8_t status;

  memset(&spi_xfer_data, 0, sizeof(spi_transfer_t));

  spi_xfer_data.opcode = W25N01GV_WRITE_STATUS_REG;
  spi_xfer_data.dummy_cycles = W25N01GV_WRITE_STATUS_REG_DUMMY_CYCLES;
  spi_xfer_data.rx_len = 0;
  spi_xfer_data.tx_buf = reg_write;
  spi_xfer_data.tx_len = 1;
  spi_xfer_data.addr = &reg_addr;
  spi_xfer_data.addr_len = W25N01GV_SR_ADDR_SIZE;

  /* Wait for busy bit */
  status = wait_if_busy(w25n01gc_flash, W25N01GV_BUSY_DEFAULT_TIMEOUT_MS);

  if (status != FLASH_SUCCESS) {
    if (status == FLASH_TIMEOUT) {
      printk("wait busy timeout\n");
    } else {
      printk("wait if busy fails\n");
    }
    return status;
  }

  status = flash_spi_transfer(w25n01gc_flash, &spi_xfer_data);

  if (status != FLASH_SUCCESS) {
    printk("spi transfer failed\n");
    return status;
  }
  return status;
}

int8_t w25n01gv_read_status_reg(flash_device_t *w25n01gc_flash, uint8_t reg_addr, uint8_t *reg_read) {
  spi_transfer_t spi_xfer_data;
  int8_t status;

  memset(&spi_xfer_data, 0, sizeof(spi_transfer_t));

  spi_xfer_data.opcode = W25N01GV_READ_STATUS_REG;
  spi_xfer_data.dummy_cycles = W25N01GV_READ_STATUS_REG_DUMMY_CYCLES;
  spi_xfer_data.rx_buf = reg_read;
  spi_xfer_data.rx_len = 1;
  spi_xfer_data.tx_len = 0;
  spi_xfer_data.addr = &reg_addr;
  spi_xfer_data.addr_len = W25N01GV_SR_ADDR_SIZE;

  status = flash_spi_transfer(w25n01gc_flash, &spi_xfer_data);

  if (status != FLASH_SUCCESS) {
    printk("spi transfer failed\n");
    return status;
  }
  return status;
}

int8_t w25n01gv_write_disable(flash_device_t *w25n01gc_flash) {
  spi_transfer_t spi_xfer_data;
  int8_t status;

  memset(&spi_xfer_data, 0, sizeof(spi_transfer_t));

  spi_xfer_data.opcode = W25N01GV_WRITE_DISABLE;
  spi_xfer_data.dummy_cycles = W25N01GV_WRITE_DISABLE_DUMMY_CYCLES;
  spi_xfer_data.rx_len = 0;
  spi_xfer_data.tx_len = 0;

  /* Wait for busy bit */
  status = wait_if_busy(w25n01gc_flash, W25N01GV_BUSY_DEFAULT_TIMEOUT_MS);

  if (status != FLASH_SUCCESS) {
    if (status == FLASH_TIMEOUT) {
      printk("wait busy timeout\n");
    } else {
      printk("wait if busy fails\n");
    }
    return status;
  }

  status = flash_spi_transfer(w25n01gc_flash, &spi_xfer_data);

  if (status != FLASH_SUCCESS) {
    printk("spi transfer failed\n");
    return status;
  }
  return status;
}

int8_t w25n01gv_write_enable(flash_device_t *w25n01gc_flash) {
  spi_transfer_t spi_xfer_data;
  int8_t status;

  memset(&spi_xfer_data, 0, sizeof(spi_transfer_t));

  spi_xfer_data.opcode = W25N01GV_WRITE_ENABLE;
  spi_xfer_data.dummy_cycles = W25N01GV_WRITE_ENABLE_DUMMY_CYCLES;
  spi_xfer_data.rx_len = 0;
  spi_xfer_data.tx_len = 0;

  /* Wait for busy bit */
  status = wait_if_busy(w25n01gc_flash, W25N01GV_BUSY_DEFAULT_TIMEOUT_MS);

  if (status != FLASH_SUCCESS) {
    if (status == FLASH_TIMEOUT) {
      printk("wait busy timeout\n");
    } else {
      printk("wait if busy fails\n");
    }
    return status;
  }

  status = flash_spi_transfer(w25n01gc_flash, &spi_xfer_data);

  if (status != FLASH_SUCCESS) {
    printk("spi transfer failed\n");
    return status;
  }
  return status;
}

int8_t wait_if_busy(flash_device_t *w25n01gc_flash, uint32_t busy_timeout) {
  uint32_t iter = 0;
  int8_t status = FLASH_SUCCESS;

  while (status == 1) {
    if (iter == busy_timeout) {
      return FLASH_TIMEOUT;
    }

    w25n01gc_flash->sleep(W25N01GV_BUSY_SLEEP_TIME_MS);
    iter++;
    status = check_busy(w25n01gc_flash);
  }
  return status;
}

int8_t set_block_protect(flash_device_t *w25n01gc_flash, uint8_t block_protect_mode) {
  uint8_t reg_val;
  uint8_t reg_write;
  int8_t status = FLASH_SUCCESS;
  status = w25n01gv_read_status_reg(w25n01gc_flash, W25N01GV_PROTECTION_REG, &reg_val);

  if (status != FLASH_SUCCESS) {
    printk("read status register failed\n");
    return status;
  }

  reg_val &= ~W25N01GV_BP_MASK;
  reg_val &= ~W25N01GV_TB_MASK;

  reg_val |= ((block_protect_mode & 0x0F) << W25N01GV_BP_OFFSET);
  reg_val |= ((block_protect_mode & 0x10) << W25N01GV_TB_OFFSET);

  reg_write = reg_val;
  status = w25n01gv_write_status_reg(w25n01gc_flash, W25N01GV_PROTECTION_REG,
      &reg_val);

  if (status != FLASH_SUCCESS) {
    printk("write status register failed\n");
    return status;
  }

  /* Recheck if the value is correct */
  status = w25n01gv_read_status_reg(w25n01gc_flash, W25N01GV_PROTECTION_REG, &reg_val);

  if (status != FLASH_SUCCESS) {
    printk("read status register failed\n");
    return status;
  }

  if (reg_write != reg_val) {
    printk("written value mismatch\n");
    return status;
  }
  return FLASH_SUCCESS;
}

int8_t check_busy(flash_device_t *w25n01gc_flash) {
  uint8_t reg_val;
  int8_t status = FLASH_SUCCESS;
  status = w25n01gv_read_status_reg(w25n01gc_flash, W25N01GV_STATUS_REG, &reg_val);

  if (status != FLASH_SUCCESS) {
    printk("read status register failed\n");
    return status;
  }
  return ((reg_val & W25N01GV_BUSY_MASK) >> W25N01GV_BUSY_OFFSET);
}

int8_t check_fail(flash_device_t *w25n01gc_flash) {
  uint8_t reg_val;
  int8_t status = FLASH_SUCCESS;
  status = w25n01gv_read_status_reg(w25n01gc_flash, W25N01GV_STATUS_REG, &reg_val);

  if (status != FLASH_SUCCESS) {
    printk("read status registed failed\n");
    return status;
  }

  if (((reg_val & W25N01GV_EFAIL_MASK) >> W25N01GV_EFAIL_OFFSET) == 0x01) {
    printk("erase fail bit set\n");
    return ERASE_FAIL_CODE;
  } else if (((reg_val & W25N01GV_PFAIL_MASK) >> W25N01GV_PFAIL_OFFSET) == 0x01) {
    printk("program fail bit is set\n");
    return PROGRAM_FAIL_CODE;
  }
  return ERASE_PROGRAM_SUCESS;
}

int8_t check_ecc(flash_device_t *w25n01gc_flash) {
  uint8_t reg_val;
  int8_t status = FLASH_SUCCESS;
  status = w25n01gv_read_status_reg(w25n01gc_flash, W25N01GV_STATUS_REG, &reg_val);

  if (status != FLASH_SUCCESS) {
    printk("read status registed failed\n");
    return status;
  }
  return ((reg_val & W25N01GV_ECC_MASK) >> W25N01GV_ECC_OFFSET);
}

int8_t flash_spi_transfer(flash_device_t *flash_dev, spi_transfer_t *spi_xfer_data) {
  uint16_t len = 0;
  int32_t status = FLASH_SUCCESS;

  memset(spi_xfer_buf, 0, MAX_SPI_BUFFER_SIZE);
  memset(spi_rcv_buf, 0, MAX_SPI_BUFFER_SIZE);

  spi_xfer_buf[0] = spi_xfer_data->opcode;
  len++;

  if (spi_xfer_data->dummy_cyles_pos == 0) {
    len += spi_xfer_data->dummy_cycles;
  }

  if (spi_xfer_data->addr_len != 0) {
    memcpy(spi_xfer_buf + len, spi_xfer_data->addr, spi_xfer_data->addr_len);
    len += spi_xfer_data->addr_len;
  }

  if (spi_xfer_data->dummy_cyles_pos == 1) {
    len += spi_xfer_data->dummy_cycles;
  }

  if (spi_xfer_data->tx_len != 0) {
    memcpy(spi_xfer_buf + len, spi_xfer_data->tx_buf, spi_xfer_data->tx_len);
    len += spi_xfer_data->tx_len;
  }

  // printk("tx len=%d , rx_len=%d  \n", len, len+spi_xfer_data->rx_len);
  k_msleep(1);

  /* Transfer SPI data and receive */
  nrf_gpio_pin_clear(EXT_FLASH_SPI_CS);
  status = flash_dev->spi_txfer(spi_xfer_buf, len, spi_rcv_buf, (len + spi_xfer_data->rx_len));
  nrf_gpio_pin_set(EXT_FLASH_SPI_CS);

  if (status != FLASH_SUCCESS) {
    printk("spi transfer failed\n");
    return status;
  }

  if (spi_xfer_data->rx_len != 0) {
    // printk("rx_l= %d \n",spi_xfer_data->rx_len);
    memcpy(spi_xfer_data->rx_buf, spi_rcv_buf + len, spi_xfer_data->rx_len);
  }

  //  for(int i=0;i<spi_xfer_data->rx_len; i++)
  //    printk(" %X", *(spi_xfer_data->rx_buf+i));
  // printk("\n");
  return FLASH_SUCCESS;
}