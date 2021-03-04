/*
 ******************************************************************************
 * @file    read_data_polling.c
 * @author  MEMS Software Solution Team
 * @brief   This file shows how to get data from sensor .
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#define SENSOR_BUS hi2c1


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>
#include "lps22hh_reg.h"
#include "stm32f3xx_hal.h"


/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME        5 //ms

/* Private variables ---------------------------------------------------------*/
static uint32_t data_raw_pressure;
static int16_t data_raw_temperature;
//static float pressure_hPa;
//static float temperature_degC;
static uint8_t whoamI, rst;


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

stmdev_ctx_t lps22hh_init(void){
	stmdev_ctx_t dev_ctx;

	/* Initialize mems driver interface */
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &SENSOR_BUS;


	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);

	/* Check device ID */
	whoamI = 0;
	lps22hh_device_id_get(&dev_ctx, &whoamI);

	if ( whoamI != LPS22HH_ID )
	while (1); /*manage here device not found */

	/* Restore default configuration */
	lps22hh_reset_set(&dev_ctx, PROPERTY_ENABLE);

	do {
	lps22hh_reset_get(&dev_ctx, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lps22hh_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

	/* Set Output Data Rate */
	lps22hh_data_rate_set(&dev_ctx, LPS22HH_10_Hz_LOW_NOISE);

	return dev_ctx;
}

void get_pressure(stmdev_ctx_t dev_ctx, float *pressure){
	/* Read output only if new value is available */
	lps22hh_reg_t reg;
	lps22hh_read_reg(&dev_ctx, LPS22HH_STATUS, (uint8_t *)&reg, 1);

	if (reg.status.p_da) {
	  memset(&data_raw_pressure, 0x00, sizeof(uint32_t));
	  lps22hh_pressure_raw_get(&dev_ctx, &data_raw_pressure);
//	  pressure_hPa = lps22hh_from_lsb_to_hpa( data_raw_pressure);
//	  sprintf((char *)tx_buffer, "pressure [hPa]:%hu\r\n", (uint16_t)pressure_hPa);
//	  tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );

	  *pressure = lps22hh_from_lsb_to_hpa( data_raw_pressure);
	}
}

void get_temperature(stmdev_ctx_t dev_ctx, float *temperature){
	/* Read output only if new value is available */
	lps22hh_reg_t reg;
	lps22hh_read_reg(&dev_ctx, LPS22HH_STATUS, (uint8_t *)&reg, 1);

	if (reg.status.t_da) {
	  memset(&data_raw_temperature, 0x00, sizeof(int16_t));
	  lps22hh_temperature_raw_get(&dev_ctx, &data_raw_temperature);
//	  temperature_degC = lps22hh_from_lsb_to_celsius(
//						   data_raw_temperature );
//	  sprintf((char *)tx_buffer, "temperature [degC]:%hu\r\n",
//			  (uint16_t)temperature_degC );
//	  tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );

	  *temperature = lps22hh_from_lsb_to_celsius(data_raw_temperature);
	}
}


//void lps22hh_read_data_polling(void)
//{
//  stmdev_ctx_t dev_ctx;
//  lps22hh_reg_t reg;
//  /* Initialize mems driver interface */
//  dev_ctx.write_reg = platform_write;
//  dev_ctx.read_reg = platform_read;
//  dev_ctx.handle = &SENSOR_BUS;
//  /* Initialize platform specific hardware */
//  platform_init();
//  /* Wait sensor boot time */
//  platform_delay(BOOT_TIME);
//  /* Check device ID */
//  whoamI = 0;
//  lps22hh_device_id_get(&dev_ctx, &whoamI);
//
//  if ( whoamI != LPS22HH_ID )
//    while (1); /*manage here device not found */
//
//  /* Restore default configuration */
//  lps22hh_reset_set(&dev_ctx, PROPERTY_ENABLE);
//
//  do {
//    lps22hh_reset_get(&dev_ctx, &rst);
//  } while (rst);
//
//  /* Enable Block Data Update */
//  lps22hh_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
//  /* Set Output Data Rate */
//  lps22hh_data_rate_set(&dev_ctx, LPS22HH_10_Hz_LOW_NOISE);
//
//  /* Read samples in polling mode (no int) */
//  while (1) {
//    /* Read output only if new value is available */
//    lps22hh_read_reg(&dev_ctx, LPS22HH_STATUS, (uint8_t *)&reg, 1);
//
//    if (reg.status.p_da) {
//      memset(&data_raw_pressure, 0x00, sizeof(uint32_t));
//      lps22hh_pressure_raw_get(&dev_ctx, &data_raw_pressure);
//      pressure_hPa = lps22hh_from_lsb_to_hpa( data_raw_pressure);
//      sprintf((char *)tx_buffer, "pressure [hPa]:%hu\r\n", (uint16_t)pressure_hPa);
//      tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );
//    }
//
//    if (reg.status.t_da) {
//      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
//      lps22hh_temperature_raw_get(&dev_ctx, &data_raw_temperature);
//      temperature_degC = lps22hh_from_lsb_to_celsius(
//                           data_raw_temperature );
//      sprintf((char *)tx_buffer, "temperature [degC]:%hu\r\n",
//    		  (uint16_t)temperature_degC );
//      tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );
//    }
//  }
//}

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
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LPS22HH_I2C_ADD_H, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

  return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
//static void tx_com(uint8_t *tx_buffer, uint16_t len)
//{
//  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
//}

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

