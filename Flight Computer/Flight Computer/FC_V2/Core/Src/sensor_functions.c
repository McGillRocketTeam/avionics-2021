/*
 * sensor_functions.c
 * modified from ST libraries for the Flight Computer project
 *
 */

#define SENSOR_BUS hi2c1

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>
#include "lsm6dsr_reg.h"
#include "lps22hh_reg.h"
#include "stm32f3xx_hal.h"


/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms

/* Private variables ---------------------------------------------------------*/

// For LSM6DSR
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static uint8_t whoamI_lsm6dsr, rst_lsm6dsr;

// For LPS22HL
static uint32_t data_raw_pressure;
static int16_t data_raw_temperature;
static uint8_t whoamI_lps22hh, rst_lps22hh;


/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t lsm6dsr_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len);
static int32_t lsm6dsr_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static int32_t lps22hh_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len);
static int32_t lps22hh_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint32_t ms);

/* LSM6DSR Functions ---------------------------------------------------------*/

stmdev_ctx_t lsm6dsr_init(void){

	stmdev_ctx_t dev_ctx_lsm6dsr;

	/* Initialize mems driver interface */
	dev_ctx_lsm6dsr.write_reg = lsm6dsr_write;
	dev_ctx_lsm6dsr.read_reg = lsm6dsr_read;
	dev_ctx_lsm6dsr.handle = &SENSOR_BUS;

	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);

	/* Check device ID */
	lsm6dsr_device_id_get(&dev_ctx_lsm6dsr, &whoamI_lsm6dsr);

	if (whoamI_lsm6dsr != LSM6DSR_ID){
		HAL_GPIO_WritePin(LED_Status_GPIO_Port, LED_Status_Pin, GPIO_PIN_SET);
		__BKPT();
		while (1);
	}

	/* Restore default configuration */
	lsm6dsr_reset_set(&dev_ctx_lsm6dsr, PROPERTY_ENABLE);

	do {
	lsm6dsr_reset_get(&dev_ctx_lsm6dsr, &rst_lsm6dsr);
	} while (rst_lsm6dsr);

	/* Disable I3C interface */
	lsm6dsr_i3c_disable_set(&dev_ctx_lsm6dsr, LSM6DSR_I3C_DISABLE);

	/* Enable Block Data Update */
	lsm6dsr_block_data_update_set(&dev_ctx_lsm6dsr, PROPERTY_ENABLE);

	/* Set Output Data Rate */
	lsm6dsr_xl_data_rate_set(&dev_ctx_lsm6dsr, LSM6DSR_XL_ODR_12Hz5);
	lsm6dsr_gy_data_rate_set(&dev_ctx_lsm6dsr, LSM6DSR_GY_ODR_12Hz5);

	/* Set full scale */
	lsm6dsr_xl_full_scale_set(&dev_ctx_lsm6dsr, LSM6DSR_2g);
	lsm6dsr_gy_full_scale_set(&dev_ctx_lsm6dsr, LSM6DSR_2000dps);

	/* Configure filtering chain(No aux interface)
	* Accelerometer - LPF1 + LPF2 path
	*/
	lsm6dsr_xl_hp_path_on_out_set(&dev_ctx_lsm6dsr, LSM6DSR_LP_ODR_DIV_100);
	lsm6dsr_xl_filter_lp2_set(&dev_ctx_lsm6dsr, PROPERTY_ENABLE);

	return dev_ctx_lsm6dsr;
}

void get_acceleration(stmdev_ctx_t dev_ctx_lsm6dsr, float *acceleration_mg){

	uint8_t reg;

	/* Read output only if new xl value is available */
	lsm6dsr_xl_flag_data_ready_get(&dev_ctx_lsm6dsr, &reg);

	if (reg) {
	  /* Read acceleration field data */
	  memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
	  lsm6dsr_acceleration_raw_get(&dev_ctx_lsm6dsr, data_raw_acceleration);
	  acceleration_mg[0] =
		lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[0]);
	  acceleration_mg[1] =
		lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[1]);
	  acceleration_mg[2] =
		lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[2]);
	}

}

void get_angvelocity(stmdev_ctx_t dev_ctx_lsm6dsr, float *angular_rate_mdps){
	uint8_t reg;

	/* Read output only if new gyro value is available*/
	lsm6dsr_gy_flag_data_ready_get(&dev_ctx_lsm6dsr, &reg);

	if (reg) {
	  /* Read angular rate field data */
	  memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
	  lsm6dsr_angular_rate_raw_get(&dev_ctx_lsm6dsr, data_raw_angular_rate);
	  angular_rate_mdps[0] =
		lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
	  angular_rate_mdps[1] =
		lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
	  angular_rate_mdps[2] =
		lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);
	}
}

/* LPS22HH Functions ---------------------------------------------------------*/
stmdev_ctx_t lps22hh_init(void){
	stmdev_ctx_t dev_ctx_lps22hh;

	/* Initialize mems driver interface */
	dev_ctx_lps22hh.write_reg = lps22hh_write;
	dev_ctx_lps22hh.read_reg = lps22hh_read;
	dev_ctx_lps22hh.handle = &SENSOR_BUS;


	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);

	/* Check device ID */
	whoamI_lps22hh = 0;
	lps22hh_device_id_get(&dev_ctx_lps22hh, &whoamI_lps22hh);

	if ( whoamI_lps22hh != LPS22HH_ID ){
		HAL_GPIO_WritePin(LED_Status_GPIO_Port, LED_Status_Pin, GPIO_PIN_SET);
		__BKPT();
		while (1); /*manage here device not found */
	}


	/* Restore default configuration */
	lps22hh_reset_set(&dev_ctx_lps22hh, PROPERTY_ENABLE);

	do {
		lps22hh_reset_get(&dev_ctx_lps22hh, &rst_lps22hh);
	} while (rst_lps22hh);

	/* Enable Block Data Update */
	lps22hh_block_data_update_set(&dev_ctx_lps22hh, PROPERTY_ENABLE);

	/* Set Output Data Rate */
	lps22hh_data_rate_set(&dev_ctx_lps22hh, LPS22HH_10_Hz_LOW_NOISE);

	return dev_ctx_lps22hh;
}

void get_pressure(stmdev_ctx_t dev_ctx_lps22hh, float *pressure){
	/* Read output only if new value is available */
	lps22hh_reg_t reg;
	lps22hh_read_reg(&dev_ctx_lps22hh, LPS22HH_STATUS, (uint8_t *)&reg, 1);

	if (reg.status.p_da) {
	  memset(&data_raw_pressure, 0x00, sizeof(uint32_t));
	  lps22hh_pressure_raw_get(&dev_ctx_lps22hh, &data_raw_pressure);
	  *pressure = lps22hh_from_lsb_to_hpa( data_raw_pressure);
	}
}

void get_temperature(stmdev_ctx_t dev_ctx_lps22hh, float *temperature){
	/* Read output only if new value is available */
	lps22hh_reg_t reg;
	lps22hh_read_reg(&dev_ctx_lps22hh, LPS22HH_STATUS, (uint8_t *)&reg, 1);

	if (reg.status.t_da) {
	  memset(&data_raw_temperature, 0x00, sizeof(int16_t));
	  lps22hh_temperature_raw_get(&dev_ctx_lps22hh, &data_raw_temperature);
	  *temperature = lps22hh_from_lsb_to_celsius(data_raw_temperature);
	}
}


/* Platform Dependent Sensor Read/Write Functions ----------------------------*/

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t lsm6dsr_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len)
{

  HAL_I2C_Mem_Write(handle, LSM6DSR_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t lsm6dsr_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{

  HAL_I2C_Mem_Read(handle, LSM6DSR_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

  return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t lps22hh_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len)
{

  HAL_I2C_Mem_Write(handle, LPS22HH_I2C_ADD_H, reg,
                    I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t lps22hh_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LPS22HH_I2C_ADD_H, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

  return 0;
}


/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}
