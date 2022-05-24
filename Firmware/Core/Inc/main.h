/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "stdio.h"
#include "arm_math.h"
  /* USER CODE END Includes */

  /* Exported types ------------------------------------------------------------*/
  /* USER CODE BEGIN ET */
  typedef unsigned char u8;
  typedef signed char s8;
  typedef unsigned short u16;
  typedef signed short s16;
  typedef unsigned int u32;
  typedef signed int s32;

  /*Boot Define*/
#define REDNOAH_FW_INFO 0x22052300
#define REDNOAH_RESET SCB->AIRCR = 0x05FA0000 | 0x04
#define REDNOAH_FLASH_20 ((u32)0x08180000)
/* Packet Define */
#define RX_SIZE (1024 * 40 + 32) // 40Kb

/*UART Define*/
#define BHD6 6
#define UART1_BUAD 921600 // 1Mbps
#define LED_ON 0
#define LED_OF 1
#define STOP 0xFF
#define PLAY 0xCC
#define RUN 0x11
#define TIMER_1 1
#define TIMER_7 7
#define PI_ 3.14159265

/*I2C Define*/
#define I2C_400KHZ 20
#define I2C_1MHZ 7
#define I2C_2MHZ 2
#define I2C_3MHZ 0
#define I2C_8BIT 1
#define I2C_16BIT 2
#define I2C_ACK 0

/*hardware-dependent declaration for WBB*/
#define BOOT_ID0 HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_8)
#define BOOT_ID1 HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_13)

#define LED5(in) HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, (GPIO_PinState)in)
#define LED4(in) HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, (GPIO_PinState)in)
#define LED3(in) HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, (GPIO_PinState)in)
#define LED2(in) HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, (GPIO_PinState)in)
#define LED1(in) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, (GPIO_PinState)in)

#define P0(in) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, (GPIO_PinState)in)
#define P1(in) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, (GPIO_PinState)in)
#define P2(in) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, (GPIO_PinState)in)
#define P3(in) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, (GPIO_PinState)in)

#define LED1_T HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_7)
#define LED2_T HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_9)
#define LED3_T HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_10)
#define LED4_T HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_11)
#define LED5_T HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_12)

#define LED6_T HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_11)
#define LED7_T HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_12)
#define LED8_T HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13)

#define P0_T HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6)
#define P1_T HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7)
#define P2_T HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8)
#define P3_T HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9)
#define KEY2 HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)

#define ADJ_LDO_EN(in) HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, (GPIO_PinState)in)
#define LDO_EN(in) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, (GPIO_PinState)in)

  /* USER CODE END ET */

  /* Exported constants --------------------------------------------------------*/
  /* USER CODE BEGIN EC */

  /* USER CODE END EC */

  /* Exported macro ------------------------------------------------------------*/
  /* USER CODE BEGIN EM */

  /* USER CODE END EM */

  /* Exported functions prototypes ---------------------------------------------*/
  void Error_Handler(void);

  /* USER CODE BEGIN EFP */

  /*bit register stuct*/
  struct bit_reg
  {
    u32
        rx_done : 1,
        rx_fail : 1,
        tx_done : 1,
        tx_fail : 1,
        i2c_act : 1,
        timer1_act : 1,
        timer7_act : 1,
        scope_act : 1,
        rtp_act : 1;
  };

  /*stream buffer stuct*/
  struct rtp_packet
  {
    u8 buf[2][RX_SIZE];
    u8 dev;
    u8 echo;
    u8 swap;
    u16 play;
    u16 cnt[2];
    u16 size[2];
  };

  /*uart buffer stuct*/
  struct uart_packet
  {
    u8 buf[RX_SIZE];
    u8 hw;
    u8 get;
    u8 echo;
    u32 dev_num;
    u32 get_size;
  };

  /* USER CODE END EFP */

  /* Private defines -----------------------------------------------------------*/
  /* USER CODE BEGIN Private defines */
  u16 _8u16(u8 *in);
  u32 _8u32(u8 *in);
  void _16u8(u16 *_u16, u8 *_u8);

  int play_rtp_task(void);
  int init_rtp_task(void);
  u8 i2c_8bit_r(u8 addr);
  u8 i2c_8bit_w(u8 addr, u8 data);
  void uart_transfer_task(u32 tx_size);
  void echo_i2c_msg(u8 msg);
  static void i2c_task(void);
  static void sys_led_task(void);
  static void init_redhoah_system(void);
  static void sys_timer_set(u32 timer, u32 mode, u32 usec);
  static void board_set_task(void);
  static void device_info_request(void);
  static void main_func_state_machine(void);

  /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
