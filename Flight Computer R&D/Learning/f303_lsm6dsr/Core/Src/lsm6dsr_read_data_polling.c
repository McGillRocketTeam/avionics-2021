/*
 ******************************************************************************
 * @file    read_data_polling.c
 * @author  Sensors Software Solution Team
 * @brief   This file show how to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */


/* STMicroelectronics evaluation boards definition
 *
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */


/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

#define SENSOR_BUS hi2c1

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>
#include "lsm6dsr_reg.h"
#include "stm32f3xx_hal.h"


/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float temperature_degC;
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void lsm6dsr_read_data_polling(void)
{
  stmdev_ctx_t dev_ctx;
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;
  /* Init test platform */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  lsm6dsr_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSR_ID)
    while (1);

  /* Restore default configuration */
  lsm6dsr_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dsr_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dsr_i3c_disable_set(&dev_ctx, LSM6DSR_I3C_DISABLE);
  /* Enable Block Data Update */
  lsm6dsr_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lsm6dsr_xl_data_rate_set(&dev_ctx, LSM6DSR_XL_ODR_12Hz5);
  lsm6dsr_gy_data_rate_set(&dev_ctx, LSM6DSR_GY_ODR_12Hz5);
  /* Set full scale */
  lsm6dsr_xl_full_scale_set(&dev_ctx, LSM6DSR_2g);
  lsm6dsr_gy_full_scale_set(&dev_ctx, LSM6DSR_2000dps);
  /* Configure filtering chain(No aux interface)
   * Accelerometer - LPF1 + LPF2 path
   */
  lsm6dsr_xl_hp_path_on_out_set(&dev_ctx, LSM6DSR_LP_ODR_DIV_100);
  lsm6dsr_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);

  /* Read samples in polling mode */
  while (1) {
    uint8_t reg;
    /* Read output only if new xl value is available */
    lsm6dsr_xl_flag_data_ready_get(&dev_ctx, &reg);

    if (reg) {
      /* Read acceleration field data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      lsm6dsr_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      acceleration_mg[0] =
        lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] =
        lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] =
        lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[2]);
      sprintf((char *)tx_buffer,
              "Acceleration [mg]:%hu\t%hu\t%hu\r\n",
			  (uint16_t)acceleration_mg[0], (uint16_t)acceleration_mg[1], (uint16_t)acceleration_mg[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    lsm6dsr_gy_flag_data_ready_get(&dev_ctx, &reg);

    if (reg) {
      /* Read angular rate field data */
      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
      lsm6dsr_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
      angular_rate_mdps[0] =
        lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
      angular_rate_mdps[1] =
        lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
      angular_rate_mdps[2] =
        lsm6dsr_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);
      sprintf((char *)tx_buffer,
              "Angular rate [mdps]:%hu\t%hu\t%hu\r\n",
              (uint16_t)angular_rate_mdps[0], (uint16_t)angular_rate_mdps[1], (uint16_t)angular_rate_mdps[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    lsm6dsr_temp_flag_data_ready_get(&dev_ctx, &reg);

    if (reg) {
      /* Read temperature data */
      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
      lsm6dsr_temperature_raw_get(&dev_ctx, &data_raw_temperature);
      temperature_degC = lsm6dsr_from_lsb_to_celsius(
                           data_raw_temperature);
      sprintf((char *)tx_buffer,
              "Temperature [degC]:%hu\r\n", (uint16_t)temperature_degC);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
  }
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
static int32_t platform_write(void *handle, uint8_t reg,
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
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{

  HAL_I2C_Mem_Read(handle, LSM6DSR_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

  return 0;
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{

  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);

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

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{

}
