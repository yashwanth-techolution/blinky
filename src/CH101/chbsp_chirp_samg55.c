///*
// Copyright � 2016-2019, Chirp Microsystems.  All rights reserved.

// Chirp Microsystems CONFIDENTIAL

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL CHIRP MICROSYSTEMS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// You can contact the authors of this program by email at support@chirpmicro.com
// or by mail at 2560 Ninth Street, Suite 220, Berkeley, CA 94710.
// */

///* Zephyr Includes for i2C */
//#include <device.h>
//#include <drivers/i2c.h>
//#include <errno.h>
//#include <math.h>
//#include <stdio.h>
//#include <sys/printk.h>
//#include <hal/nrf_gpio.h>
////#include <nrfx/hal/nrf_gpio.h>

//#include "chirp_smartsonic.h"		// header with board-specific defines
//#include "soniclib.h"				// Chirp SonicLib API definitions
//#include "chirp_bsp.h"
//#include "time.h"
//#include "chirp_board_config.h"
//#include "Includes/I2C_Module.h"
//#include "Includes/GPIO_Expander.h"

///* Used with Nordic */
////#define CHIRP_RST     31  // added by Samuel
////#define CHIRP_PROG_0  9 // added by Samuel
////#define CHIRP_INT     10 // added by Samuel

///** Reference voltage for ADC, in mV. */
//#define VOLT_REF              (3277)
///** The ADC maximal digital value */
//#define MAX_DIGITAL_12_BIT    (4095UL)
//#define ADC_MODE_NO_AUTOTEST  (0x00000000)
//#define ADC_ACR_REG_ADDR      (0x40038094)

//#define ADC_MEASURE_ODR_HZ         (100)
//#define ADC_NB_SAMPLES_PER_MEASURE (10) /* 10 samples per ODR cycle, it was for motion sensor */
//#define ADC_MEASURE_WAIT_TIME_US   ((1000000 / ADC_MEASURE_ODR_HZ / ADC_NB_SAMPLES_PER_MEASURE) - 30)
//#define SENSORS_CURRENT_NB_MEASURE (100)
//#define ADC_CONV_TIME_US           (30)
//#define ADC_MEAS_RAW_TO_UA_RATIO   (0.6171) /* 5.1R sense, x100 and 1.21 Op-amp gain. mA to uA */

//static uint8_t chirp_i2c_addrs[] = CHIRP_I2C_ADDRS;
//static uint8_t chirp_i2c_buses[] = CHIRP_I2C_BUSES;

///* Chirp sensor group pointer */
//ch_group_t	*sensor_group_ptr;

///* Callback function pointers */
//static ch_timer_callback_t  periodic_timer_callback_ptr = NULL;

////static uint16_t periodic_timer_interval_ms;

//static uint16_t ultrasound_timer_period_in_tick = 0xFFFF;
//static uint16_t ultrasound_prev_period_end_in_tick;

///* Counter used to decimate call to ultrasound timer callback from TC0 ISR in case decimation
//   factor is != 1 */
//static uint8_t decimation_counter = 0;

///* Used in case the timer resolution and range (16 bits HW counter) overflows */
//static uint8_t decimation_factor;

//extern void ext_MotionINT_handler(void);

///* Forward declaration */
//static void program_next_period(void);
//static uint32_t get_period_in_tick(uint32_t interval_us);
//static void find_sensors(void);


//static void find_sensors(void)
//{
//        uint8_t ret = 0;

//        uint8_t sig_bytes[2];

//	//nrf_gpio_cfg_output(CHIRP_RST); //reset=output  // changed by samuel
//	//nrf_gpio_pin_set(CHIRP_RST); //reset=H   // changed by samuel
//        gpio_expander_port_A_pin_set(CHIRP_RST);    // added by samuel Now controlled from GPIO Expander pins

//	/* Drive PROG low on all sensor ports */
//	//nrf_gpio_cfg_output(CHIRP_PROG_0); //PROG_0=output   // changed by samuel
//	//nrf_gpio_pin_clear(CHIRP_PROG_0); //PROG_0=L     // changed by samuel
//        gpio_expander_port_A_pin_clear(CHIRP_PROG_0);    // added by samuel Now controlled from GPIO Expander pins

//	/* check sensor 0 (on board chip or J6) */
//	//nrf_gpio_pin_set(CHIRP_PROG_0);   // changed by samuel
//        gpio_expander_port_A_pin_set(CHIRP_PROG_0);    // added by samuel Now controlled from GPIO Expander pins

//        //k_msleep(2000U);

//	sig_bytes[0] = 0;
//	sig_bytes[1] = 0;
//        uint8_t add = 0x00;
//        printk("I2C add = %X \n", CH_I2C_ADDR_PROG);
        
//        ret = i2cReadRegister(CH_I2C_ADDR_PROG, &add, 1, &sig_bytes, 2); // added by samuel for nordic 
//        if(ret != 0)
//        {
//          printk(" Error (%d) Reading USS\n", ret);
//        }

//        printk("Read = %X , %X \n", sig_bytes[0], sig_bytes[1]);
//	printf("Chirp sensor 0 ");
//	if ((sig_bytes[0] == CH_SIG_BYTE_0) && (sig_bytes[1] == CH_SIG_BYTE_1)) {
//		printf("found\n");
//		} else {
//		printf("not found\n");
//	}
//	//nrf_gpio_pin_clear(CHIRP_PROG_0);    // changed by samuel
//        gpio_expander_port_A_pin_clear(CHIRP_PROG_0);    // added by samuel Now controlled from GPIO Expander pins
//}


///*!
// * \brief Initialize board hardware
// *
// * \note This function performs all necessary initialization on the board.
// */
//void chbsp_board_init(ch_group_t *grp_ptr) {

//	/* Make local copy of group pointer */
//	sensor_group_ptr = grp_ptr;

//	/* Initialize group descriptor */
//	grp_ptr->num_ports = CHBSP_MAX_DEVICES;
//	grp_ptr->num_i2c_buses = CHBSP_NUM_I2C_BUSES;
//	grp_ptr->rtc_cal_pulse_ms = CHBSP_RTC_CAL_PULSE_MS;
        

//	/* Probe I2C bus to find connected sensor(s) */
//	find_sensors();   // Modifications Required For Nordic. By Samuel
//}


///*!
// * \brief Assert the reset pin
// *
// * This function drives the sensor reset pin low.
// */
//void chbsp_reset_assert(void) {

//        //nrf_gpio_pin_clear(CHIRP_RST);  // added by Samuel
//        gpio_expander_port_A_pin_clear(CHIRP_RST);    // added by samuel Now controlled from GPIO Expander pins
//}

///*!
// * \brief Deassert the reset pin
// *
// * This function drives the sensor reset pin high.
// */
//void chbsp_reset_release(void) {

//        //nrf_gpio_pin_set(CHIRP_RST);  // added by Samuel
//        gpio_expander_port_A_pin_set(CHIRP_RST);    // added by samuel Now controlled from GPIO Expander pins
//}

///*!
// * \brief Assert the PROG pin
// *
// * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
// *
// * This function drives the sensor PROG pin high on the specified port.
// */
//void chbsp_program_enable(ch_dev_t *dev_ptr) {
//        //nrf_gpio_pin_set(CHIRP_PROG_0);  // added by Samuel
//        gpio_expander_port_A_pin_set(CHIRP_PROG_0);    // added by samuel Now controlled from GPIO Expander pins
//}

///*!
// * \brief Deassert the PROG pin
// *
// * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
// *
// * This function drives the sensor PROG pin low on the specified port.
// */
//void chbsp_program_disable(ch_dev_t *dev_ptr) {
//	uint8_t dev_num = ch_get_dev_num(dev_ptr);

//        //nrf_gpio_pin_clear(CHIRP_PROG_0);  // added by Samuel
//        gpio_expander_port_A_pin_clear(CHIRP_PROG_0);    // added by samuel Now controlled from GPIO Expander pins
//}

///*!
// * \brief Configure the Chirp sensor INT pin as an output for one sensor.
// *
// * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
// *
// * This function configures the Chirp sensor INT pin as an output (from the perspective
// * of the host system).
// */
//void chbsp_set_io_dir_out(ch_dev_t *dev_ptr) {
//	uint8_t dev_num = ch_get_dev_num(dev_ptr);

//        //nrf_gpio_cfg_output(CHIRP_INT);  // added by samuel for product balancer  // added by Samuel
//}


///*!
// * \brief Configure the Chirp sensor INT pin as an input for one sensor.
// *
// * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
// *
// * This function configures the Chirp sensor INT pin as an input (from the perspective of
// * the host system).
// */
//void chbsp_set_io_dir_in(ch_dev_t *dev_ptr) {
//	uint8_t dev_num = ch_get_dev_num(dev_ptr);

//        //nrf_gpio_cfg_input(CHIRP_INT, GPIO_PIN_CNF_PULL_Pullup);
//}


///*!
// * \brief Configure the Chirp sensor INT pins as outputs for a group of sensors
// *
// * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
// *
// * This function configures each Chirp sensor's INT pin as an output (from the perspective
// * of the host system).
// */
//void chbsp_group_set_io_dir_out(ch_group_t *grp_ptr) {
//        //nrf_gpio_cfg_output(CHIRP_INT);  // added by samuel for product balancer  // added by Samuel
//}

///*!
// * \brief Configure the Chirp sensor INT pins as inputs for a group of sensors
// *
// * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
// *
// * \note This function assumes a bidirectional level shifter is interfacing the ICs.
// */
//void chbsp_group_set_io_dir_in(ch_group_t *grp_ptr) {
//        //nrf_gpio_cfg_input(CHIRP_INT, GPIO_PIN_CNF_PULL_Pullup);
//}


///*!
// * \brief Initialize the I/O pins.
// *
// * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
// *
// * Configure reset and program pins as outputs. Assert reset and program. Configure
// * sensor INT pin as input.
// */
//void chbsp_group_pin_init(ch_group_t *grp_ptr) {
//	uint8_t dev_num;
//	uint8_t port_num;

//        //nrf_gpio_cfg_output(CHIRP_PROG_0);

//        //nrf_gpio_pin_clear(CHIRP_PROG_0);
//        gpio_expander_port_A_pin_clear(CHIRP_PROG_0);    // added by samuel Now controlled from GPIO Expander pins

//        //nrf_gpio_cfg_output(CHIRP_RST);
//	chbsp_reset_assert();


//	for (dev_num = 0; dev_num < grp_ptr->num_ports; dev_num++) {
//		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

//		chbsp_program_enable(dev_ptr);
//	}

//	/* Initialize IO pins */
//	chbsp_group_set_io_dir_in(grp_ptr);
//}

///*!
// * \brief Set the INT pins low for a group of sensors.
// *
// * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
// *
// * This function drives the INT line low for each sensor in the group.
// */
//void chbsp_group_io_clear(ch_group_t *grp_ptr) {
//        //nrf_gpio_pin_clear(CHIRP_INT); // added by samuel
//        gpio_expander_port_A_pin_clear(CHIRP_INT);    // added by samuel Now controlled from GPIO Expander pins
//}

// /*!
// * \brief Set the INT pins high for a group of sensors.
// *
// * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
// *
// * This function drives the INT line high for each sensor in the group.
// */
//void chbsp_group_io_set(ch_group_t *grp_ptr) {
//        //nrf_gpio_pin_set(CHIRP_INT); // added by samuel
//        gpio_expander_port_A_pin_set(CHIRP_INT);    // added by samuel Now controlled from GPIO Expander pins
//}


///*!
// * \brief Disable interrupts for a group of sensors
// *
// * \param grp_ptr 	pointer to the ch_group_t config structure for a group of sensors
// *
// * For each sensor in the group, this function disables the host interrupt associated
// * with the Chirp sensor device's INT line.
// */
//void chbsp_group_io_interrupt_enable(ch_group_t *grp_ptr) {
//	uint8_t dev_num;
//}

///*!
// * \brief Enable the interrupt for one sensor
// *
// * \param dev_ptr	pointer to the ch_dev_t config structure for a sensor
// *
// * This function enables the host interrupt associated with the Chirp sensor device's
// * INT line.
// */
//void chbsp_io_interrupt_enable(ch_dev_t *dev_ptr) {
//}

///*!
// * \brief Disable interrupts for a group of sensors
// *
// * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
// *
// * For each sensor in the group, this function disables the host interrupt associated
// * with the Chirp sensor device's INT line.
// */
//void chbsp_group_io_interrupt_disable(ch_group_t *grp_ptr) {
//}

///*!
// * \brief Disable the interrupt for one sensor
// *
// * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
// *
// * This function disables the host interrupt associated with the Chirp sensor device's
// * INT line.
// */
//void chbsp_io_interrupt_disable(ch_dev_t *dev_ptr) {
//}

///*!
// * \brief Set the INT pins low for a one sensor.
// *
// * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
// *
// * This function drives the INT line low for one sensor.
// */
//void chbsp_io_clear(ch_dev_t *dev_ptr) {
//        //nrf_gpio_pin_clear(CHIRP_INT); // added by samuel
//        gpio_expander_port_A_pin_clear(CHIRP_INT);    // added by samuel Now controlled from GPIO Expander pins
//}

///*!
// * \brief Set the INT pins high for a one sensor.
// *
// * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
// *
// * This function drives the INT line high for one sensor.
// */
//void chbsp_io_set(ch_dev_t *dev_ptr) {
//        //nrf_gpio_pin_set(CHIRP_INT); // added by samuel
//        gpio_expander_port_A_pin_set(CHIRP_INT);    // added by samuel Now controlled from GPIO Expander pins
//}

///*!
// * \brief Set callback routine for Chirp sensor I/O interrupt
// *
// * \param callback_func_ptr 	pointer to application function to be called when interrupt occurs
// *
// * This function sets up the specified callback routine to be called whenever the interrupt
// * associated with the sensor's INT line occurs.  The callback routine address in stored in
// * a pointer variable that will later be accessed from within the interrupt handler to call
// * the function.
// *
// * The callback function will be called at interrupt level from the interrupt
// * service routine.
// */
//void chbsp_io_callback_set(ch_io_int_callback_t callback_func_ptr) {

//	io_int_callback_ptr = callback_func_ptr;
//}


///*!
// * \brief Delay for specified number of microseconds
// *
// * \param us  	number of microseconds to delay before returning
// *
// * This function waits for the specified number of microseconds before returning to
// * the caller.
// */
//void chbsp_delay_us(uint32_t us) {

//        k_usleep(us);
//}

///*!
// * \brief Delay for specified number of milliseconds.
// *
// * \param ms 	number of milliseconds to delay before returning
// *
// * This function waits for the specified number of milliseconds before returning to
// * the caller.
// */
//void chbsp_delay_ms(uint32_t ms) {

//        k_msleep(ms);
//}

///*!
// * \brief Initialize the host's I2C hardware.
// *
// * \return 0 if successful, 1 on error
// *
// * This function performs general I2C initialization on the host system.
// */
//int chbsp_i2c_init(void) {

//	return 0;
//}

///*!
// * \brief Return I2C information for a sensor port on the board.
// *
// * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
// * \param dev_num		device number within sensor group
// * \param info_ptr		pointer to structure to be filled with I2C config values
// *
// * \return 0 if successful, 1 if error
// *
// * This function returns I2C values in the ch_i2c_info_t structure specified by \a info_ptr.
// * The structure includes three fields.
// *  - The \a address field contains the I2C address for the sensor.
// *  - The \a bus_num field contains the I2C bus number (index).
// *  - The \a drv_flags field contains various bit flags through which the BSP can inform
// *  SonicLib driver functions to perform specific actions during I2C I/O operations.
// */
//uint8_t chbsp_i2c_get_info(ch_group_t __attribute__((unused)) *grp_ptr, uint8_t io_index, ch_i2c_info_t *info_ptr) {
//	uint8_t ret_val = 1;

//	if (io_index <= CHBSP_MAX_DEVICES) {
//		info_ptr->address = chirp_i2c_addrs[io_index];
//		info_ptr->bus_num = chirp_i2c_buses[io_index];

//		info_ptr->drv_flags = 0;	// no special I2C handling by SonicLib driver is needed

//		ret_val = 0;
//	}

//	return ret_val;
//}

///*!
// * \brief Write bytes to an I2C slave.
// *
// * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
// * \param data 			data to be transmitted
// * \param num_bytes 	length of data to be transmitted
// *
// * \return 0 if successful, 1 on error or NACK
// *
// * This function writes one or more bytes of data to an I2C slave device.
// * The I2C interface must have already been initialized using \a chbsp_i2c_init().
// */
//int chbsp_i2c_write(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes) {
//	int error = 0;

//        //printk("I2C Addreee = %X \n", dev_ptr->i2c_address);

//	if (dev_ptr->i2c_bus_index == 0) {
//                error = i2cSendRegister(data, num_bytes, dev_ptr->i2c_address);  // added for nordic by Samuel

//	} else if (dev_ptr->i2c_bus_index == 1) {   
//                error = i2cSendRegister(data, num_bytes, dev_ptr->i2c_address);  // added for nordic by Samuel
//	}

//        if(error != 0)
//        {
//          printk("Error (%d) Sending USS\n", error);
//        }
//	return error;
//}

///*!
// * \brief Write bytes to an I2C slave using memory addressing.
// *
// * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
// * \param mem_addr		internal memory or register address within device
// * \param data 			data to be transmitted
// * \param num_bytes 	length of data to be transmitted
// *
// * \return 0 if successful, 1 on error or NACK
// *
// * This function writes one or more bytes of data to an I2C slave device using an internal
// * memory or register address.  The remote device will write \a num_bytes bytes of
// * data starting at internal memory/register address \a mem_addr.
// * The I2C interface must have already been initialized using \a chbsp_i2c_init().
// */
//int chbsp_i2c_mem_write(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes) {
//	int error=0;
        
//        //printk("I2C Addreee = %X \n", dev_ptr->i2c_address);

//	if (dev_ptr->i2c_bus_index == 0) {
//		error = i2cSendBurst(dev_ptr->i2c_address, mem_addr, data, num_bytes);  // added for nordic by Samuel

//		} else if (dev_ptr->i2c_bus_index == 1) {
//		error = i2cSendBurst(dev_ptr->i2c_address, mem_addr, data, num_bytes);  // added for nordic by Samuel
//	}
//        if(error != 0)
//        {
//          printk("Error (%d) Sending USS\n", error);
//        }
//	return error;
//}

///*!
// * \brief Write bytes to an I2C slave, non-blocking.
// *
// * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
// * \param data 			pointer to the start of data to be transmitted
// * \param num_bytes		length of data to be transmitted
// *
// * \return 0 if successful, 1 on error or NACK
// *
// * This function initiates a non-blocking write of the specified number of bytes to an
// * I2C slave device.
// *
// * The I2C interface must have already been initialized using \a chbsp_i2c_init().
// */
//int chbsp_i2c_write_nb(ch_dev_t __attribute__((unused)) *dev_ptr, uint8_t __attribute__((unused)) *data, uint16_t __attribute__((unused)) num_bytes) {

//	// XXX not implemented
//	return 1;
//}

///*!
// * \brief Write bytes to an I2C slave using memory addressing, non-blocking.
// *
// * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
// * \param mem_addr		internal memory or register address within device
// * \param data 			pointer to the start of data to be transmitted
// * \param num_bytes		length of data to be transmitted
// *
// * \return 0 if successful, 1 on error or NACK
// *
// * This function initiates a non-blocking write of the specified number of bytes to an
// * I2C slave device, using an internal memory or register address.  The remote device will write
// * \a num_bytes bytes of data starting at internal memory/register address \a mem_addr.
// *
// * The I2C interface must have already been initialized using \a chbsp_i2c_init().
// */
//int chbsp_i2c_mem_write_nb(ch_dev_t __attribute__((unused)) *dev_ptr, uint16_t __attribute__((unused)) mem_addr, uint8_t __attribute__((unused)) *data, uint16_t __attribute__((unused)) num_bytes) {

//	// XXX not implemented
//	return 1;
//}

///*!
// * \brief Read bytes from an I2C slave.
// *
// * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
// * \param data 			pointer to receive data buffer
// * \param num_bytes 	number of bytes to read
// *
// * \return 0 if successful, 1 on error or NACK
// *
// * This function reads the specified number of bytes from an I2C slave device.
// * The I2C interface must have already been initialized using \a chbsp_i2c_init().
// */
//int chbsp_i2c_read(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes) {
//	int error = 1;		// default is error return
//	uint8_t i2c_addr = ch_get_i2c_address(dev_ptr);
//	uint8_t bus_num  = ch_get_i2c_bus(dev_ptr);

//        //printk("I2C Addreee = %X \n", i2c_addr);

//	if (bus_num == 0) {
//                error = i2cRead(i2c_addr, data, num_bytes);  // added by samuel for nordic

//	} else if (bus_num == 1) {
//                error = i2cRead(i2c_addr, data, num_bytes);  // added by samuel for nordic
//	}
//        if(error != 0)
//        {
//          printk("Error (%d) Reading USS\n", error);
//        }
//	return error;
//}

///*!
// * \brief Read bytes from an I2C slave using memory addressing.
// *
// * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
// * \param mem_addr		internal memory or register address within device
// * \param data 			pointer to receive data buffer
// * \param num_bytes 	number of bytes to read
// *
// * \return 0 if successful, 1 on error or NACK
// *
// * This function reads the specified number of bytes from an I2C slave device, using
// * an internal memory or register address.  The remote device will return \a num_bytes bytes
// * starting at internal memory/register address \a mem_addr.
// *
// * The I2C interface must have already been initialized using \a chbsp_i2c_init().
// */
//int chbsp_i2c_mem_read(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes) {
//	int error = 1;		// default is error return
//	uint8_t i2c_addr = ch_get_i2c_address(dev_ptr);
//	uint8_t bus_num  = ch_get_i2c_bus(dev_ptr);

//        //printk("I2C Address to read = %X \n", i2c_addr);
//        //printk("I2C mem = %X \n", mem_addr);
//        //printk("I2C length = %d \n", num_bytes);

//	if (bus_num == 0) {
//                error = i2cReadRegister(i2c_addr, &mem_addr, 1, data, num_bytes);  // added by samuel for nordic

//		} else if (bus_num == 1) {
//                error = i2cReadRegister(i2c_addr, &mem_addr, 1, data, num_bytes);  // added by samuel for nordic
//	}
//        //printk("data = %d \n", *data);
//        if(error != 0)
//        {
//          printk("Error (%d) Reading USS\n", error);
//        }
//	return error;
//}

/////*!
//// * \brief Read bytes from an I2C slave, non-blocking.
//// *
//// * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
//// * \param data 			pointer to receive data buffer
//// * \param num_bytes 	number of bytes to read
//// *
//// * \return 0 if successful, 1 on error or NACK
//// *
//// * This function initiates a non-blocking read of the specified number of bytes from
//// * an I2C slave.
//// *
//// * The I2C interface must have already been initialized using \a chbsp_i2c_init().
//// */
//int chbsp_i2c_read_nb(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes) {
//}

///*!
// * \brief Read bytes from an I2C slave using memory addressing, non-blocking.
// *
// * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
// * \param mem_addr		internal memory or register address within device
// * \param data 			pointer to receive data buffer
// * \param num_bytes 	number of bytes to read
// *
// * \return 0 if successful, 1 on error or NACK
// *
// * This function initiates a non-blocking read of the specified number of bytes from an I2C slave.
// *
// * The I2C interface must have already been initialized using \a chbsp_i2c_init().
// */
//int chbsp_i2c_mem_read_nb(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes) {
//}

///*!
// * \brief Reset I2C bus associated with device.
// *
// * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
// *
// * This function performs a reset of the I2C interface for the specified device.
// */
//void chbsp_i2c_reset(ch_dev_t * dev_ptr) {
//}

///*!
// * \brief Initialize periodic timer.
// *
// * \param interval_ms		timer interval, in milliseconds
// * \param callback_func_ptr	address of routine to be called every time the timer expires
// *
// * \return 0 if successful, 1 if error
// *
// * This function initializes a periodic timer on the board.  The timer is programmed
// * to generate an interrupt after every \a interval_ms milliseconds.
// *
// * The \a callback_func_ptr parameter specifies a callback routine that will be called when the
// * timer expires (and interrupt occurs).  The \a chbsp_periodic_timer_handler function
// * will call this function.
// */
//uint8_t chbsp_periodic_timer_init(uint16_t interval_ms, ch_timer_callback_t callback_func_ptr) {
//}

//void chbsp_periodic_timer_change_period(uint32_t new_interval_us)
//{
//    uint16_t prev_expiration = ultrasound_prev_period_end_in_tick - ultrasound_timer_period_in_tick;

//    ultrasound_timer_period_in_tick = get_period_in_tick(new_interval_us);

//    ultrasound_prev_period_end_in_tick = prev_expiration;

//    program_next_period();
//}

//uint32_t get_period_in_tick(uint32_t interval_us) {
//	uint64_t timer_period_in_tick = (uint64_t) ULTRASOUND_TIMER_FREQUENCY * interval_us / 1000000;

//	/* If the ODR is too slow to be handled then program a faster interrupt and decimate it */
//	if (timer_period_in_tick > UINT16_MAX)
//		decimation_factor = timer_period_in_tick / UINT16_MAX + 1;
//	else
//		decimation_factor = 1;

//	/* Calculate the final tick in case a decimation is needed */
//	return (uint32_t) (timer_period_in_tick / decimation_factor);
//}

//void program_next_period(void)
//{
//	uint32_t time = ultrasound_prev_period_end_in_tick + ultrasound_timer_period_in_tick;
//	ultrasound_prev_period_end_in_tick = time;
//}

///*!
// * \brief Enable periodic timer interrupt.
// *
// * This function enables the interrupt associated with the periodic timer initialized by
// * \a chbsp_periodic_timer_init().
// */
//void chbsp_periodic_timer_irq_enable(void) {
//}

///*!
// * \brief Disable periodic timer interrupt.
// *
// * This function enables the interrupt associated with the periodic timer initialized by
// * \a chbsp_periodic_timer_init().
// */
//void chbsp_periodic_timer_irq_disable(void) {

//}


///*!
// * \brief Start periodic timer.
// *
// * \return 0 if successful, 1 if error
// *
// * This function starts the periodic timer initialized by \a chbsp_periodic_timer_init().
// */
//uint8_t chbsp_periodic_timer_start(void) {

//	decimation_counter = 0;
//	/* The timer start done at the very end is resetting the counter */
//	ultrasound_prev_period_end_in_tick = 0;
//	program_next_period();

//	/* Start the HW counter (this resets the counter */

//	return 0;
//}


///*!
// * \brief Stop periodic timer.
// *
// * \return 0 if successful, 1 if error
// *
// * This function stops the periodic timer initialized by \a chbsp_periodic_timer_init().
// */
//uint8_t chbsp_periodic_timer_stop(void) {

//	return 0;
//}


///*!
// * \brief Periodic timer handler.
// *
// * \return 0 if successful, 1 if error
// *
// * This function handles the expiration of the periodic timer, re-arms it and any associated
// * interrupts for the next interval, and calls the callback routine that was registered using
// * \a chbsp_periodic_timer_init().
// */
//void chbsp_periodic_timer_handler(void) {
//	ch_timer_callback_t func_ptr = periodic_timer_callback_ptr;

//	decimation_counter++;
//	program_next_period();
//	if (decimation_counter >= decimation_factor) {
//		decimation_counter = 0;
//		if (func_ptr != NULL) {
//			(*func_ptr)();			// call application timer callback routine
//		}
//	}
//}


///*!
// * \brief Put the processor into low-power sleep state.
// *
// * This function puts the host processor (MCU) into a low-power sleep mode, to conserve energy.
// * The sleep state should be selected such that interrupts associated with the I2C, external
// * GPIO pins, and the periodic timer (if used) are able to wake up the device.
// */
//void chbsp_proc_sleep(void) {

//}

///*!
// * \brief Turn on an LED on the board.
// *
// * This function turns on an LED on the board.
// *
// * The \a dev_num parameter contains the device number of a specific sensor.  This routine
// * will turn on the LED on the Chirp sensor daughterboard that is next to the specified
// * sensor.
// */
//void chbsp_led_on(uint8_t led_num) {

//}

///*!
// * \brief Turn off an LED on the board.
// *
// * This function turns off an LED on the board.
// *
// * The \a dev_num parameter contains the device number of a specific sensor.  This routine
// * will turn off the LED on the Chirp sensor daughterboard that is next to the specified
// * sensor.
// */
//void chbsp_led_off(uint8_t led_num) {

//}

///*!
// * \brief Toggles an LED on the board.
// *
// * This function toggles an LED on the board.
// *
// * The \a dev_num parameter contains the device number of a specific sensor.  This routine
// * will toggles the LED on the Chirp sensor daughterboard that is next to the specified
// * sensor.
// */
//void chbsp_led_toggle(uint8_t led_num) {

//}

///*!
// * \brief Output a text string via serial interface
// *
// * \param str	pointer to a string of characters to be output
// *
// * This function prints debug information to the console.
// */
//void chbsp_print_str(char *str) {
//	printf(str);
//}

///*!
// * \brief Return the current time in ms
// *
// * This function returns the system current time in ms.
// */
//uint32_t chbsp_timestamp_ms(void) {
//       return k_uptime_get();
//}

///************* End of file chbsp_chirp_smartsonic.c  --   Copyright 2016-2019 Chirp Microsystems **********/











/*
 Copyright � 2016-2019, Chirp Microsystems.  All rights reserved.

 Chirp Microsystems CONFIDENTIAL

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL CHIRP MICROSYSTEMS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 You can contact the authors of this program by email at support@chirpmicro.com
 or by mail at 2560 Ninth Street, Suite 220, Berkeley, CA 94710.
 */

/* Zephyr Includes for i2C */
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <zephyr/sys/printk.h>
#include <hal/nrf_gpio.h>
//#include <nrfx/hal/nrf_gpio.h>

#include "chirp_smartsonic.h"		// header with board-specific defines
#include "soniclib.h"				// Chirp SonicLib API definitions
#include "chirp_bsp.h"
#include "time.h"
#include "chirp_board_config.h"
#include "Includes/I2C_Module.h"
#include "Includes/GPIO_Expander.h"

/* Used with Nordic */
//#define CHIRP_RST     31  // added by Samuel
//#define CHIRP_PROG_0  9 // added by Samuel
//#define CHIRP_INT     10 // added by Samuel

/** Reference voltage for ADC, in mV. */
#define VOLT_REF              (3277)
/** The ADC maximal digital value */
#define MAX_DIGITAL_12_BIT    (4095UL)
#define ADC_MODE_NO_AUTOTEST  (0x00000000)
#define ADC_ACR_REG_ADDR      (0x40038094)

#define ADC_MEASURE_ODR_HZ         (100)
#define ADC_NB_SAMPLES_PER_MEASURE (10) /* 10 samples per ODR cycle, it was for motion sensor */
#define ADC_MEASURE_WAIT_TIME_US   ((1000000 / ADC_MEASURE_ODR_HZ / ADC_NB_SAMPLES_PER_MEASURE) - 30)
#define SENSORS_CURRENT_NB_MEASURE (100)
#define ADC_CONV_TIME_US           (30)
#define ADC_MEAS_RAW_TO_UA_RATIO   (0.6171) /* 5.1R sense, x100 and 1.21 Op-amp gain. mA to uA */

static uint8_t chirp_i2c_addrs[] = CHIRP_I2C_ADDRS;
static uint8_t chirp_i2c_buses[] = CHIRP_I2C_BUSES;

/* Chirp sensor group pointer */
ch_group_t	*sensor_group_ptr;

/* Callback function pointers */
static ch_timer_callback_t  periodic_timer_callback_ptr = NULL;

//static uint16_t periodic_timer_interval_ms;

static uint16_t ultrasound_timer_period_in_tick = 0xFFFF;
static uint16_t ultrasound_prev_period_end_in_tick;

/* Counter used to decimate call to ultrasound timer callback from TC0 ISR in case decimation
   factor is != 1 */
static uint8_t decimation_counter = 0;

/* Used in case the timer resolution and range (16 bits HW counter) overflows */
static uint8_t decimation_factor;

extern void ext_MotionINT_handler(void);

/* Forward declaration */
static void program_next_period(void);
static uint32_t get_period_in_tick(uint32_t interval_us);
static void find_sensors(void);


static void find_sensors(void)
{
        uint8_t ret = 0;

        uint8_t sig_bytes[2];

	//nrf_gpio_cfg_output(CHIRP_RST); //reset=output  // changed by samuel
	//nrf_gpio_pin_set(CHIRP_RST); //reset=H   // changed by samuel
        gpio_expander_port_A_pin_set(CHIRP_RST_0);    // added by samuel Now controlled from GPIO Expander pins
        gpio_expander_port_A_pin_set(CHIRP_RST_1);    // added by samuel Now controlled from GPIO Expander pins
        gpio_expander_port_A_pin_set(CHIRP_RST_2);    // added by samuel Now controlled from GPIO Expander pins
        gpio_expander_port_A_pin_set(CHIRP_RST_3);    // added by samuel Now controlled from GPIO Expander pins

	/* Drive PROG low on all sensor ports */
	//nrf_gpio_cfg_output(CHIRP_PROG_0); //PROG_0=output   // changed by samuel
	//nrf_gpio_pin_clear(CHIRP_PROG_0); //PROG_0=L     // changed by samuel
        gpio_expander_port_A_pin_clear(CHIRP_PROG_0);    // added by samuel Now controlled from GPIO Expander pins
        gpio_expander_port_A_pin_clear(CHIRP_PROG_1);    // added by samuel Now controlled from GPIO Expander pins
        gpio_expander_port_A_pin_clear(CHIRP_PROG_2);    // added by samuel Now controlled from GPIO Expander pins
        gpio_expander_port_A_pin_clear(CHIRP_PROG_3);    // added by samuel Now controlled from GPIO Expander pins

	
        /******************* check sensor 0 *******************/
	//nrf_gpio_pin_set(CHIRP_PROG_0);   // changed by samuel
        gpio_expander_port_A_pin_set(CHIRP_PROG_0);    // added by samuel Now controlled from GPIO Expander pins
	sig_bytes[0] = 0;
	sig_bytes[1] = 0;
        uint8_t add = 0x00;
        printk("I2C add = %X \t", CH_I2C_ADDR_PROG);    
        ret = i2c1ReadRegister(CH_I2C_ADDR_PROG, &add, 1, &sig_bytes, 2); // added for USS
        if(ret != 0)
        {
          printk(" Error (%d) Reading USS\t", ret);
        }
        printk("Read = %X , %X \t", sig_bytes[0], sig_bytes[1]);
	printf("Chirp sensor 0 ");
	if ((sig_bytes[0] == CH_SIG_BYTE_0) && (sig_bytes[1] == CH_SIG_BYTE_1)) {
		printf("found\n");
		} else {
		printf("not found\n");
	}
	//nrf_gpio_pin_clear(CHIRP_PROG_0);    // changed by samuel
        gpio_expander_port_A_pin_clear(CHIRP_PROG_0);    // added by samuel Now controlled from GPIO Expander pins
        /******************************************************/
        

        /******************* check sensor 1 *******************/
	//nrf_gpio_pin_set(CHIRP_PROG_0);   // changed by samuel
        gpio_expander_port_A_pin_set(CHIRP_PROG_1);    // added by samuel Now controlled from GPIO Expander pins
	sig_bytes[0] = 0;
	sig_bytes[1] = 0;
        //uint8_t add = 0x00;
        printk("I2C1 add = %X \t", CH_I2C_ADDR_PROG);
        ret = i2c1ReadRegister(CH_I2C_ADDR_PROG, &add, 1, &sig_bytes, 2); // added for USS
        if(ret != 0)
        {
          printk(" Error (%d) Reading USS\t", ret);
        }
        printk("Read = %X , %X \t", sig_bytes[0], sig_bytes[1]);
	printf("Chirp sensor 1 ");
	if ((sig_bytes[0] == CH_SIG_BYTE_0) && (sig_bytes[1] == CH_SIG_BYTE_1)) {
		printf("found\n");
		} else {
		printf("not found\n");
	}
	//nrf_gpio_pin_clear(CHIRP_PROG_0);    // changed by samuel
        gpio_expander_port_A_pin_clear(CHIRP_PROG_1);    // added by samuel Now controlled from GPIO Expander pins
        /******************************************************/

        
        /******************* check sensor 2 *******************/
	//nrf_gpio_pin_set(CHIRP_PROG_0);   // changed by samuel
        gpio_expander_port_A_pin_set(CHIRP_PROG_2);    // added by samuel Now controlled from GPIO Expander pins
	sig_bytes[0] = 0;
	sig_bytes[1] = 0;
        //uint8_t add = 0x00;
        printk("I2C1 add = %X \t", CH_I2C_ADDR_PROG);
        ret = i2c1ReadRegister(CH_I2C_ADDR_PROG, &add, 1, &sig_bytes, 2); // added for USS
        if(ret != 0)
        {
          printk(" Error (%d) Reading USS\t", ret);
        }
        printk("Read = %X , %X \t", sig_bytes[0], sig_bytes[1]);
	printf("Chirp sensor 2 ");
	if ((sig_bytes[0] == CH_SIG_BYTE_0) && (sig_bytes[1] == CH_SIG_BYTE_1)) {
		printf("found\n");
		} else {
		printf("not found\n");
	}
	//nrf_gpio_pin_clear(CHIRP_PROG_0);    // changed by samuel
        gpio_expander_port_A_pin_clear(CHIRP_PROG_2);    // added by samuel Now controlled from GPIO Expander pins
        /******************************************************/
        

        /******************* check sensor 3 *******************/
	//nrf_gpio_pin_set(CHIRP_PROG_0);   // changed by samuel
        gpio_expander_port_A_pin_set(CHIRP_PROG_3);    // added by samuel Now controlled from GPIO Expander pins
	sig_bytes[0] = 0;
	sig_bytes[1] = 0;
        //uint8_t add = 0x00;
        printk("I2C1 add = %X \t", CH_I2C_ADDR_PROG);
        ret = i2c1ReadRegister(CH_I2C_ADDR_PROG, &add, 1, &sig_bytes, 2); // added for USS
        if(ret != 0)
        {
          printk(" Error (%d) Reading USS\t", ret);
        }
        printk("Read = %X , %X \t", sig_bytes[0], sig_bytes[1]);
	printf("Chirp sensor 3 ");
	if ((sig_bytes[0] == CH_SIG_BYTE_0) && (sig_bytes[1] == CH_SIG_BYTE_1)) {
		printf("found\n");
		} else {
		printf("not found\n");
	}
	//nrf_gpio_pin_clear(CHIRP_PROG_0);    // changed by samuel
        gpio_expander_port_A_pin_clear(CHIRP_PROG_3);    // added by samuel Now controlled from GPIO Expander pins
        /******************************************************/
}


/*!
 * \brief Initialize board hardware
 *
 * \note This function performs all necessary initialization on the board.
 */
void chbsp_board_init(ch_group_t *grp_ptr) {

	/* Make local copy of group pointer */
	sensor_group_ptr = grp_ptr;

	/* Initialize group descriptor */
	grp_ptr->num_ports = CHBSP_MAX_DEVICES;
	grp_ptr->num_i2c_buses = CHBSP_NUM_I2C_BUSES;
	grp_ptr->rtc_cal_pulse_ms = CHBSP_RTC_CAL_PULSE_MS;
        

	/* Probe I2C bus to find connected sensor(s) */
	find_sensors();   // Modifications Required For Nordic. By Samuel
}


/*!
 * \brief Assert the reset pin
 *
 * This function drives the sensor reset pin low.
 */
void chbsp_reset_assert(void) {

        //nrf_gpio_pin_clear(CHIRP_RST);  // added by Samuel
        gpio_expander_port_A_pin_clear(CHIRP_RST_0);    // added by samuel Now controlled from GPIO Expander pins
        gpio_expander_port_A_pin_clear(CHIRP_RST_1);    // added by samuel Now controlled from GPIO Expander pins
        gpio_expander_port_A_pin_clear(CHIRP_RST_2);    // added by samuel Now controlled from GPIO Expander pins
        gpio_expander_port_A_pin_clear(CHIRP_RST_3);    // added by samuel Now controlled from GPIO Expander pins
}

/*!
 * \brief Deassert the reset pin
 *
 * This function drives the sensor reset pin high.
 */
void chbsp_reset_release(void) {

        //nrf_gpio_pin_set(CHIRP_RST);  // added by Samuel
        gpio_expander_port_A_pin_set(CHIRP_RST_0);    // added by samuel Now controlled from GPIO Expander pins
        gpio_expander_port_A_pin_set(CHIRP_RST_1);    // added by samuel Now controlled from GPIO Expander pins
        gpio_expander_port_A_pin_set(CHIRP_RST_2);    // added by samuel Now controlled from GPIO Expander pins
        gpio_expander_port_A_pin_set(CHIRP_RST_3);    // added by samuel Now controlled from GPIO Expander pins
}

/*!
 * \brief Assert the PROG pin
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function drives the sensor PROG pin high on the specified port.
 */
void chbsp_program_enable(ch_dev_t *dev_ptr) {
	uint8_t dev_num = ch_get_dev_num(dev_ptr);
        
        printk("*********** Device Number = %d ********* \n", dev_num);

        //nrf_gpio_pin_set(CHIRP_PROG_0);  // added by Samuel
        if(dev_num == 0)
        {
          gpio_expander_port_A_pin_set(CHIRP_PROG_0);    // added by samuel Now controlled from GPIO Expander pins
        }
        else if(dev_num == 1)
        {
          gpio_expander_port_A_pin_set(CHIRP_PROG_1);    // added by samuel Now controlled from GPIO Expander pins
        }
        else if(dev_num == 2)
        {
          gpio_expander_port_A_pin_set(CHIRP_PROG_2);    // added by samuel Now controlled from GPIO Expander pins
        }
        else if(dev_num == 3)
        {
          gpio_expander_port_A_pin_set(CHIRP_PROG_3);    // added by samuel Now controlled from GPIO Expander pins
        }
}

/*!
 * \brief Deassert the PROG pin
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function drives the sensor PROG pin low on the specified port.
 */
void chbsp_program_disable(ch_dev_t *dev_ptr) {
	uint8_t dev_num = ch_get_dev_num(dev_ptr);
        printk("*********** Device Number = %d ********* \n", dev_num);
        
        //nrf_gpio_pin_clear(CHIRP_PROG_0);  // added by Samuel
        if(dev_num == 0)
        {
          gpio_expander_port_A_pin_clear(CHIRP_PROG_0);    // added by samuel Now controlled from GPIO Expander pins
        }
        else if(dev_num == 1)
        {
          gpio_expander_port_A_pin_clear(CHIRP_PROG_1);    // added by samuel Now controlled from GPIO Expander pins
        }
        else if(dev_num == 2)
        {
          gpio_expander_port_A_pin_clear(CHIRP_PROG_2);    // added by samuel Now controlled from GPIO Expander pins
        }
        else if(dev_num == 3)
        {
          gpio_expander_port_A_pin_clear(CHIRP_PROG_3);    // added by samuel Now controlled from GPIO Expander pins
        }
}

/*!
 * \brief Configure the Chirp sensor INT pin as an output for one sensor.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function configures the Chirp sensor INT pin as an output (from the perspective
 * of the host system).
 */
void chbsp_set_io_dir_out(ch_dev_t *dev_ptr) {
	uint8_t dev_num = ch_get_dev_num(dev_ptr);

        //nrf_gpio_cfg_output(CHIRP_INT);  // added by samuel for product balancer  // added by Samuel
}


/*!
 * \brief Configure the Chirp sensor INT pin as an input for one sensor.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function configures the Chirp sensor INT pin as an input (from the perspective of
 * the host system).
 */
void chbsp_set_io_dir_in(ch_dev_t *dev_ptr) {
	uint8_t dev_num = ch_get_dev_num(dev_ptr);

        //nrf_gpio_cfg_input(CHIRP_INT, GPIO_PIN_CNF_PULL_Pullup);
}


/*!
 * \brief Configure the Chirp sensor INT pins as outputs for a group of sensors
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 *
 * This function configures each Chirp sensor's INT pin as an output (from the perspective
 * of the host system).
 */
void chbsp_group_set_io_dir_out(ch_group_t *grp_ptr) {
        //nrf_gpio_cfg_output(CHIRP_INT);  // added by samuel for product balancer  // added by Samuel
}

/*!
 * \brief Configure the Chirp sensor INT pins as inputs for a group of sensors
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 *
 * \note This function assumes a bidirectional level shifter is interfacing the ICs.
 */
void chbsp_group_set_io_dir_in(ch_group_t *grp_ptr) {
        //nrf_gpio_cfg_input(CHIRP_INT, GPIO_PIN_CNF_PULL_Pullup);
}


/*!
 * \brief Initialize the I/O pins.
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 *
 * Configure reset and program pins as outputs. Assert reset and program. Configure
 * sensor INT pin as input.
 */
void chbsp_group_pin_init(ch_group_t *grp_ptr) {
	uint8_t dev_num;
	uint8_t port_num;

        //nrf_gpio_cfg_output(CHIRP_PROG_0);

        //nrf_gpio_pin_clear(CHIRP_PROG_0);
        gpio_expander_port_A_pin_clear(CHIRP_PROG_0);    // added by samuel Now controlled from GPIO Expander pins
        gpio_expander_port_A_pin_clear(CHIRP_PROG_1);    // added by samuel Now controlled from GPIO Expander pins
        gpio_expander_port_A_pin_clear(CHIRP_PROG_2);    // added by samuel Now controlled from GPIO Expander pins
        gpio_expander_port_A_pin_clear(CHIRP_PROG_3);    // added by samuel Now controlled from GPIO Expander pins

        //nrf_gpio_cfg_output(CHIRP_RST);
	chbsp_reset_assert();


	for (dev_num = 0; dev_num < grp_ptr->num_ports; dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		chbsp_program_enable(dev_ptr);
	}

	/* Initialize IO pins */
	chbsp_group_set_io_dir_in(grp_ptr);
}

/*!
 * \brief Set the INT pins low for a group of sensors.
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 *
 * This function drives the INT line low for each sensor in the group.
 */
void chbsp_group_io_clear(ch_group_t *grp_ptr) {
        //nrf_gpio_pin_clear(CHIRP_INT); // added by samuel
        gpio_expander_port_A_pin_clear(CHIRP_INT);    // added by samuel Now controlled from GPIO Expander pins
}

 /*!
 * \brief Set the INT pins high for a group of sensors.
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 *
 * This function drives the INT line high for each sensor in the group.
 */
void chbsp_group_io_set(ch_group_t *grp_ptr) {
        //nrf_gpio_pin_set(CHIRP_INT); // added by samuel
        gpio_expander_port_A_pin_set(CHIRP_INT);    // added by samuel Now controlled from GPIO Expander pins
}


/*!
 * \brief Disable interrupts for a group of sensors
 *
 * \param grp_ptr 	pointer to the ch_group_t config structure for a group of sensors
 *
 * For each sensor in the group, this function disables the host interrupt associated
 * with the Chirp sensor device's INT line.
 */
void chbsp_group_io_interrupt_enable(ch_group_t *grp_ptr) {
	uint8_t dev_num;
}

/*!
 * \brief Enable the interrupt for one sensor
 *
 * \param dev_ptr	pointer to the ch_dev_t config structure for a sensor
 *
 * This function enables the host interrupt associated with the Chirp sensor device's
 * INT line.
 */
void chbsp_io_interrupt_enable(ch_dev_t *dev_ptr) {
}

/*!
 * \brief Disable interrupts for a group of sensors
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 *
 * For each sensor in the group, this function disables the host interrupt associated
 * with the Chirp sensor device's INT line.
 */
void chbsp_group_io_interrupt_disable(ch_group_t *grp_ptr) {
}

/*!
 * \brief Disable the interrupt for one sensor
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function disables the host interrupt associated with the Chirp sensor device's
 * INT line.
 */
void chbsp_io_interrupt_disable(ch_dev_t *dev_ptr) {
}

/*!
 * \brief Set the INT pins low for a one sensor.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function drives the INT line low for one sensor.
 */
void chbsp_io_clear(ch_dev_t *dev_ptr) {
        //nrf_gpio_pin_clear(CHIRP_INT); // added by samuel
        gpio_expander_port_A_pin_clear(CHIRP_INT);    // added by samuel Now controlled from GPIO Expander pins
}

/*!
 * \brief Set the INT pins high for a one sensor.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function drives the INT line high for one sensor.
 */
void chbsp_io_set(ch_dev_t *dev_ptr) {
        //nrf_gpio_pin_set(CHIRP_INT); // added by samuel
        gpio_expander_port_A_pin_set(CHIRP_INT);    // added by samuel Now controlled from GPIO Expander pins
}

/*!
 * \brief Set callback routine for Chirp sensor I/O interrupt
 *
 * \param callback_func_ptr 	pointer to application function to be called when interrupt occurs
 *
 * This function sets up the specified callback routine to be called whenever the interrupt
 * associated with the sensor's INT line occurs.  The callback routine address in stored in
 * a pointer variable that will later be accessed from within the interrupt handler to call
 * the function.
 *
 * The callback function will be called at interrupt level from the interrupt
 * service routine.
 */
void chbsp_io_callback_set(ch_io_int_callback_t callback_func_ptr) {

	io_int_callback_ptr = callback_func_ptr;
}


/*!
 * \brief Delay for specified number of microseconds
 *
 * \param us  	number of microseconds to delay before returning
 *
 * This function waits for the specified number of microseconds before returning to
 * the caller.
 */
void chbsp_delay_us(uint32_t us) {

        k_usleep(us);
}

/*!
 * \brief Delay for specified number of milliseconds.
 *
 * \param ms 	number of milliseconds to delay before returning
 *
 * This function waits for the specified number of milliseconds before returning to
 * the caller.
 */
void chbsp_delay_ms(uint32_t ms) {

        k_msleep(ms);
}

/*!
 * \brief Initialize the host's I2C hardware.
 *
 * \return 0 if successful, 1 on error
 *
 * This function performs general I2C initialization on the host system.
 */
int chbsp_i2c_init(void) {

	return 0;
}

/*!
 * \brief Return I2C information for a sensor port on the board.
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a group of sensors
 * \param dev_num		device number within sensor group
 * \param info_ptr		pointer to structure to be filled with I2C config values
 *
 * \return 0 if successful, 1 if error
 *
 * This function returns I2C values in the ch_i2c_info_t structure specified by \a info_ptr.
 * The structure includes three fields.
 *  - The \a address field contains the I2C address for the sensor.
 *  - The \a bus_num field contains the I2C bus number (index).
 *  - The \a drv_flags field contains various bit flags through which the BSP can inform
 *  SonicLib driver functions to perform specific actions during I2C I/O operations.
 */
uint8_t chbsp_i2c_get_info(ch_group_t __attribute__((unused)) *grp_ptr, uint8_t io_index, ch_i2c_info_t *info_ptr) {
	uint8_t ret_val = 1;

	if (io_index <= CHBSP_MAX_DEVICES) {
		info_ptr->address = chirp_i2c_addrs[io_index];
		info_ptr->bus_num = chirp_i2c_buses[io_index];

		info_ptr->drv_flags = 0;	// no special I2C handling by SonicLib driver is needed

		ret_val = 0;
	}

	return ret_val;
}

/*!
 * \brief Write bytes to an I2C slave.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param data 			data to be transmitted
 * \param num_bytes 	length of data to be transmitted
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function writes one or more bytes of data to an I2C slave device.
 * The I2C interface must have already been initialized using \a chbsp_i2c_init().
 */
int chbsp_i2c_write(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes) {
	int error = 0;

        //printk("I2C write Addres = %X \n", dev_ptr->i2c_address);

	if (dev_ptr->i2c_bus_index == 0) {
                error = i2c1SendRegister(data, num_bytes, dev_ptr->i2c_address);  // added for USS

	} else if (dev_ptr->i2c_bus_index == 1) {   
                error = i2c1SendRegister(data, num_bytes, dev_ptr->i2c_address);  // added for USS
	}

        if(error != 0)
        {
          printk("Error (%d) Sending USS\n", error);
        }
	return error;
}

/*!
 * \brief Write bytes to an I2C slave using memory addressing.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param mem_addr		internal memory or register address within device
 * \param data 			data to be transmitted
 * \param num_bytes 	length of data to be transmitted
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function writes one or more bytes of data to an I2C slave device using an internal
 * memory or register address.  The remote device will write \a num_bytes bytes of
 * data starting at internal memory/register address \a mem_addr.
 * The I2C interface must have already been initialized using \a chbsp_i2c_init().
 */
int chbsp_i2c_mem_write(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes) {
	int error=0;
        
        //printk("I2C write Addres = %X \n", dev_ptr->i2c_address);

	if (dev_ptr->i2c_bus_index == 0) {
		error = i2c1SendBurst(dev_ptr->i2c_address, mem_addr, data, num_bytes);  // added for USS

		} else if (dev_ptr->i2c_bus_index == 1) {
		error = i2c1SendBurst(dev_ptr->i2c_address, mem_addr, data, num_bytes);  // added for USS
	}
        if(error != 0)
        {
          printk("Error (%d) Sending USS\n", error);
        }
	return error;
}

/*!
 * \brief Write bytes to an I2C slave, non-blocking.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param data 			pointer to the start of data to be transmitted
 * \param num_bytes		length of data to be transmitted
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function initiates a non-blocking write of the specified number of bytes to an
 * I2C slave device.
 *
 * The I2C interface must have already been initialized using \a chbsp_i2c_init().
 */
int chbsp_i2c_write_nb(ch_dev_t __attribute__((unused)) *dev_ptr, uint8_t __attribute__((unused)) *data, uint16_t __attribute__((unused)) num_bytes) {

	// XXX not implemented
	return 1;
}

/*!
 * \brief Write bytes to an I2C slave using memory addressing, non-blocking.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param mem_addr		internal memory or register address within device
 * \param data 			pointer to the start of data to be transmitted
 * \param num_bytes		length of data to be transmitted
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function initiates a non-blocking write of the specified number of bytes to an
 * I2C slave device, using an internal memory or register address.  The remote device will write
 * \a num_bytes bytes of data starting at internal memory/register address \a mem_addr.
 *
 * The I2C interface must have already been initialized using \a chbsp_i2c_init().
 */
int chbsp_i2c_mem_write_nb(ch_dev_t __attribute__((unused)) *dev_ptr, uint16_t __attribute__((unused)) mem_addr, uint8_t __attribute__((unused)) *data, uint16_t __attribute__((unused)) num_bytes) {

	// XXX not implemented
	return 1;
}

/*!
 * \brief Read bytes from an I2C slave.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param data 			pointer to receive data buffer
 * \param num_bytes 	number of bytes to read
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function reads the specified number of bytes from an I2C slave device.
 * The I2C interface must have already been initialized using \a chbsp_i2c_init().
 */
int chbsp_i2c_read(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes) {
	int error = 1;		// default is error return
	uint8_t i2c_addr = ch_get_i2c_address(dev_ptr);
	uint8_t bus_num  = ch_get_i2c_bus(dev_ptr);

        //printk("I2C read Addres = %X\t", i2c_addr);

	if (bus_num == 0) {
                error = i2c1Read(i2c_addr, data, num_bytes);  // added for USS

	} else if (bus_num == 1) {
                error = i2c1Read(i2c_addr, data, num_bytes);  // added for USS
	}
        if(error != 0)
        {
          printk("Error (%d) Reading USS\n", error);
        }
	return error;
}

/*!
 * \brief Read bytes from an I2C slave using memory addressing.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param mem_addr		internal memory or register address within device
 * \param data 			pointer to receive data buffer
 * \param num_bytes 	number of bytes to read
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function reads the specified number of bytes from an I2C slave device, using
 * an internal memory or register address.  The remote device will return \a num_bytes bytes
 * starting at internal memory/register address \a mem_addr.
 *
 * The I2C interface must have already been initialized using \a chbsp_i2c_init().
 */
int chbsp_i2c_mem_read(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes) {
	int error = 1;		// default is error return
	uint8_t i2c_addr = ch_get_i2c_address(dev_ptr);
	uint8_t bus_num  = ch_get_i2c_bus(dev_ptr);

        //printk("I2C read Addres = %X\t", i2c_addr);
        //printk("I2C mem = %X \n", mem_addr);
        //printk("I2C length = %d \n", num_bytes);

	if (bus_num == 0) {
                error = i2c1ReadRegister(i2c_addr, &mem_addr, 1, data, num_bytes);  // added for USS

		} else if (bus_num == 1) {
                error = i2c1ReadRegister(i2c_addr, &mem_addr, 1, data, num_bytes);  // added for USS
	}
        //printk("data = %d \n", *data);
        if(error != 0)
        {
          printk("Error (%d) Reading USS\n", error);
        }
	return error;
}

///*!
// * \brief Read bytes from an I2C slave, non-blocking.
// *
// * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
// * \param data 			pointer to receive data buffer
// * \param num_bytes 	number of bytes to read
// *
// * \return 0 if successful, 1 on error or NACK
// *
// * This function initiates a non-blocking read of the specified number of bytes from
// * an I2C slave.
// *
// * The I2C interface must have already been initialized using \a chbsp_i2c_init().
// */
int chbsp_i2c_read_nb(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes) {
}

/*!
 * \brief Read bytes from an I2C slave using memory addressing, non-blocking.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 * \param mem_addr		internal memory or register address within device
 * \param data 			pointer to receive data buffer
 * \param num_bytes 	number of bytes to read
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function initiates a non-blocking read of the specified number of bytes from an I2C slave.
 *
 * The I2C interface must have already been initialized using \a chbsp_i2c_init().
 */
int chbsp_i2c_mem_read_nb(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes) {
}

/*!
 * \brief Reset I2C bus associated with device.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a sensor
 *
 * This function performs a reset of the I2C interface for the specified device.
 */
void chbsp_i2c_reset(ch_dev_t * dev_ptr) {
}

/*!
 * \brief Initialize periodic timer.
 *
 * \param interval_ms		timer interval, in milliseconds
 * \param callback_func_ptr	address of routine to be called every time the timer expires
 *
 * \return 0 if successful, 1 if error
 *
 * This function initializes a periodic timer on the board.  The timer is programmed
 * to generate an interrupt after every \a interval_ms milliseconds.
 *
 * The \a callback_func_ptr parameter specifies a callback routine that will be called when the
 * timer expires (and interrupt occurs).  The \a chbsp_periodic_timer_handler function
 * will call this function.
 */
uint8_t chbsp_periodic_timer_init(uint16_t interval_ms, ch_timer_callback_t callback_func_ptr) {
}

void chbsp_periodic_timer_change_period(uint32_t new_interval_us)
{
    uint16_t prev_expiration = ultrasound_prev_period_end_in_tick - ultrasound_timer_period_in_tick;

    ultrasound_timer_period_in_tick = get_period_in_tick(new_interval_us);

    ultrasound_prev_period_end_in_tick = prev_expiration;

    program_next_period();
}

uint32_t get_period_in_tick(uint32_t interval_us) {
	uint64_t timer_period_in_tick = (uint64_t) ULTRASOUND_TIMER_FREQUENCY * interval_us / 1000000;

	/* If the ODR is too slow to be handled then program a faster interrupt and decimate it */
	if (timer_period_in_tick > UINT16_MAX)
		decimation_factor = timer_period_in_tick / UINT16_MAX + 1;
	else
		decimation_factor = 1;

	/* Calculate the final tick in case a decimation is needed */
	return (uint32_t) (timer_period_in_tick / decimation_factor);
}

void program_next_period(void)
{
	uint32_t time = ultrasound_prev_period_end_in_tick + ultrasound_timer_period_in_tick;
	ultrasound_prev_period_end_in_tick = time;
}

/*!
 * \brief Enable periodic timer interrupt.
 *
 * This function enables the interrupt associated with the periodic timer initialized by
 * \a chbsp_periodic_timer_init().
 */
void chbsp_periodic_timer_irq_enable(void) {
}

/*!
 * \brief Disable periodic timer interrupt.
 *
 * This function enables the interrupt associated with the periodic timer initialized by
 * \a chbsp_periodic_timer_init().
 */
void chbsp_periodic_timer_irq_disable(void) {

}


/*!
 * \brief Start periodic timer.
 *
 * \return 0 if successful, 1 if error
 *
 * This function starts the periodic timer initialized by \a chbsp_periodic_timer_init().
 */
uint8_t chbsp_periodic_timer_start(void) {

	decimation_counter = 0;
	/* The timer start done at the very end is resetting the counter */
	ultrasound_prev_period_end_in_tick = 0;
	program_next_period();

	/* Start the HW counter (this resets the counter */

	return 0;
}


/*!
 * \brief Stop periodic timer.
 *
 * \return 0 if successful, 1 if error
 *
 * This function stops the periodic timer initialized by \a chbsp_periodic_timer_init().
 */
uint8_t chbsp_periodic_timer_stop(void) {

	return 0;
}


/*!
 * \brief Periodic timer handler.
 *
 * \return 0 if successful, 1 if error
 *
 * This function handles the expiration of the periodic timer, re-arms it and any associated
 * interrupts for the next interval, and calls the callback routine that was registered using
 * \a chbsp_periodic_timer_init().
 */
void chbsp_periodic_timer_handler(void) {
	ch_timer_callback_t func_ptr = periodic_timer_callback_ptr;

	decimation_counter++;
	program_next_period();
	if (decimation_counter >= decimation_factor) {
		decimation_counter = 0;
		if (func_ptr != NULL) {
			(*func_ptr)();			// call application timer callback routine
		}
	}
}


/*!
 * \brief Put the processor into low-power sleep state.
 *
 * This function puts the host processor (MCU) into a low-power sleep mode, to conserve energy.
 * The sleep state should be selected such that interrupts associated with the I2C, external
 * GPIO pins, and the periodic timer (if used) are able to wake up the device.
 */
void chbsp_proc_sleep(void) {

}

/*!
 * \brief Turn on an LED on the board.
 *
 * This function turns on an LED on the board.
 *
 * The \a dev_num parameter contains the device number of a specific sensor.  This routine
 * will turn on the LED on the Chirp sensor daughterboard that is next to the specified
 * sensor.
 */
void chbsp_led_on(uint8_t led_num) {

}

/*!
 * \brief Turn off an LED on the board.
 *
 * This function turns off an LED on the board.
 *
 * The \a dev_num parameter contains the device number of a specific sensor.  This routine
 * will turn off the LED on the Chirp sensor daughterboard that is next to the specified
 * sensor.
 */
void chbsp_led_off(uint8_t led_num) {

}

/*!
 * \brief Toggles an LED on the board.
 *
 * This function toggles an LED on the board.
 *
 * The \a dev_num parameter contains the device number of a specific sensor.  This routine
 * will toggles the LED on the Chirp sensor daughterboard that is next to the specified
 * sensor.
 */
void chbsp_led_toggle(uint8_t led_num) {

}

/*!
 * \brief Output a text string via serial interface
 *
 * \param str	pointer to a string of characters to be output
 *
 * This function prints debug information to the console.
 */
void chbsp_print_str(char *str) {
	printf(str);
}

/*!
 * \brief Return the current time in ms
 *
 * This function returns the system current time in ms.
 */
uint32_t chbsp_timestamp_ms(void) {
       return k_uptime_get();
}

/************* End of file chbsp_chirp_smartsonic.c  --   Copyright 2016-2019 Chirp Microsystems **********/
