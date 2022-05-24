/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   : ae_i2c.h
  * @brief  : Header for i2c_gpio.c file.
  This file contains the common defines of the application.
  * @version  : 0.0.1
  * @creaor   : magi8051
  * @update	  : 21.08.12
  ******************************************************************************
  * @attention  : Design for BlueBerry Board
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __I2C_GPIO_H
#define __I2C_GPIO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

  static void idelay(uint32_t tmout);
  void init_i2c(uint32_t type, uint32_t ch);
  uint32_t init_hs_i2c(uint8_t dat, uint32_t tmout);

  uint32_t i2c_write_task(uint8_t id, uint16_t addr, uint16_t type, uint8_t *dat, uint16_t size, uint16_t tmout);
  uint32_t i2c_read_task(uint8_t id, uint16_t addr, uint16_t type, uint8_t *dat, uint16_t size, uint16_t tmout);

  uint32_t new_i2c_write_task(uint8_t id, uint8_t *dat, uint16_t size, uint16_t tmout);
  uint32_t new_i2c_read_task(uint8_t id, uint8_t *dat, uint16_t size, uint16_t tmout);
  uint8_t i2c_pin_state(void);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_GPIO_H */
