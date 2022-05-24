/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ae_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef _DEBUG_JKS
#define jks_debug(...) printf(__VA_ARGS__)
#else
#define jks_debug(...) NULL
#endif
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/*Base Struct for BBB*/
volatile struct bit_reg s_bit; // bit reigster
struct uart_packet s_upk;      // uart buffer
struct rtp_packet s_rtp;       // rtp buffer

/*board info*/
u32 g_board_info;
u8 g_ic_info;

/*boot pv*/
u32 g_flash_jmp;

/*i2c pv*/
u8 g_i2c_info[4] = {I2C_1MHZ, I2C_2MHZ, I2C_3MHZ, I2C_400KHZ};
u8 g_i2c_clk;
u8 g_i2c_id;

/*timer pv*/
u16 g_tmr_psc;
u16 g_tmr_period;

/*boot loader pv*/
#define BBB_FLASH_SECTOR_22 ((u32)0x081C0000)
#define BBB_FLASH_SECTOR BBB_FLASH_SECTOR_22
typedef void (*pFunction)(void);
pFunction jmp_to_application;
u32 jmp_address;

enum board_type
{
  REDNOAH2 = 0,
  REDNOAH3
};

enum fsm
{
  SYS_FSM = 0,
  I2C_FSM,
  RTP_FSM
};

enum dev
{
  DW7800 = 0,
  DW7912,
  DW7914
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/* Macro Data Conversion Function */
u16 _8u16(u8 *in) { return (u16)(((u16)in[0] << 8) | in[1]); }

u32 _8u32(u8 *in) { return (u32)((((u32)in[0] << 24) | ((u32)in[1] << 16) | ((u32)in[2] << 8) | ((u32)in[3] << 0))); }

s16 _twos(u16 in)
{ // two's complement
  s16 ret;
  if (in > 32767)
    ret = (s16)(in - 65536);
  else
    ret = in;
  return ret;
}

float _fp(u32 in, u32 num)
{
  return (float)in / (float)num;
}

void _16u8(u16 *_u16, u8 *_u8)
{
  *(_u8 + 0) = *(_u16 + 0) >> 8;
  *(_u8 + 1) = *(_u16 + 0) >> 0;
}

/***************************************************************************
Function	: uart_transfer_task
Date      : 2021.08.12
Editor		: JKS
Version		: 1.0
Descript 	:
***************************************************************************/
void uart_transfer_task(u32 tx_size)
{
  /*Default Header. Do not change!*/
  s_upk.buf[0] = 0xAE;
  s_upk.buf[1] = s_upk.hw;
  s_upk.buf[2] = tx_size >> 8;
  s_upk.buf[3] = tx_size >> 0;
  s_upk.buf[4] = 0;
  s_upk.buf[5] = s_upk.echo;

  for (int i = 0; i < tx_size; i++)
  {
    s_upk.buf[4] += s_upk.buf[6 + i];
  }
  HAL_UART_Transmit_DMA(&huart1, (u8 *)s_upk.buf, tx_size + BHD6);
  // HAL_UART_Transmit_IT(&huart1,(u8*)s_upk.buf, txsize + BHD6);
}

static void device_info_request(void)
{
  u8 clk, pin;

  pin = i2c_pin_state();

  /*step: sw version*/
  s_upk.buf[0x06] = 0x00; // board
  s_upk.buf[0x07] = 0x00; // info
  s_upk.buf[0x08] = (u8)(REDNOAH_FW_INFO >> 24);
  s_upk.buf[0x09] = (u8)(REDNOAH_FW_INFO >> 16);
  s_upk.buf[0x0A] = (u8)(REDNOAH_FW_INFO >> 8);
  s_upk.buf[0x0B] = (u8)(REDNOAH_FW_INFO >> 0);

  if (pin != 3)
  {
    s_upk.buf[12] = 0xff;
    s_upk.buf[13] = pin;
    s_upk.buf[14] = 0;
    s_upk.buf[15] = 0;
    s_upk.dev_num = 1;
  }
  g_i2c_clk = clk;
  uart_transfer_task(7);
}

/***************************************************************************
Function	: board_set_task
Date      : 2021.08.12
Editor		: JKS
Version		: 1.0
Descript 	: chip info, reset, who am i
***************************************************************************/
static void board_set_task(void)
{
  switch (s_upk.buf[7])
  {
  case 0x00: /*board info */
    device_info_request();
    break;
  case 0x01: /*device info*/
    g_ic_info = s_upk.buf[8];
    g_i2c_id = s_upk.buf[9];
    if (s_upk.buf[5])
      uart_transfer_task(1); // echo
    break;
  case 0x02: /*board reset */
    REDNOAH_RESET;
    break;
  case 0x03: /*ldo control */
    break;
  case 0x04: /* find auto i2c id */
    s_upk.buf[8] = 0;
    for (u8 id = 1; id < 119; id++)
    {
      if (i2c_write_task(id << 1, 0, 1, &id, 1, I2C_400KHZ) == I2C_ACK)
      {
        s_upk.buf[9 + s_upk.buf[8]] = id << 1;
        s_upk.buf[8]++;
      }
    }
    uart_transfer_task(3 + s_upk.buf[8]); // do not change!
    break;
  case 0xFA: /*firmware update*/
    if (s_upk.buf[8] == 0x01)
    {
      // bootloader_update_task();
      // jump_vector_table();
    }
    break;
  }
}

/***************************************************************************
Function  : sys_timer_set
Date      : 2021.08.12
Editor    : JKS
Version		: 1.0
Descript 	: timer control tick base is usec
***************************************************************************/
static void sys_timer_set(u32 timer, u32 mode, u32 usec)
{
  switch (timer)
  {
  case TIMER_1:
    if (mode == PLAY)
    {
      /*APB2_180Mhz*/
      g_tmr_psc = 180 - 1;
      s_bit.timer1_act = 1;
      g_tmr_period = usec - 1;
      MX_TIM1_Init();
      HAL_TIM_Base_Start_IT(&htim1);
    }
    else
    {
      s_bit.timer1_act = 0;
      HAL_TIM_Base_Stop_IT(&htim1);
    }
    break;

  case TIMER_7:
    if (mode == PLAY)
    {
      g_tmr_psc = 90 - 1;
      s_bit.timer7_act = 1;
      g_tmr_period = usec - 1;
      MX_TIM7_Init();
      HAL_TIM_Base_Start_IT(&htim7);
    }
    else
    {
      s_bit.timer7_act = 0;
      HAL_TIM_Base_Stop_IT(&htim7);
    }
    break;
  }
}

/***************************************************************************
Function	: init_rednoah_platform_board
Date      : 2021.08.12
Editor		: JKS
Version		: 1.0
Descript 	: init function
***************************************************************************/
static void init_redhoah_system(void)
{

  /*Auto board check*/
  // u8 num;
  // num = (BOOT_ID2 << 2) | (BOOT_ID1 << 1) | (BOOT_ID0 << 0);

  /*gpio i2c init start*/
  g_i2c_clk = g_i2c_info[1]; /* set 1mhz */
  init_i2c(0, 1);            /* rednoah default, i2c ch 1 */

  /*uart get start*/
  HAL_UART_Receive_IT(&huart1, &s_upk.get, 1);
  // HAL_UART_Receive_DMA(&huart1, &s_upk.get, 1);

  /*LDO power on*/
  LDO_EN(1); /* ADJ LDO Enable */

  /* WBB LDO Pin for test */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, 1);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);

  device_info_request();

  /*boot messsage to PC*/
  sys_timer_set(TIMER_1, PLAY, 10000); /* systick 10ms */
}

/***************************************************************************
Function	: tick_led_task
Date		  : 2021.08.12
Editor		: JKS
Version		: 1.0
Descript 	: led blink tick 10ms
***************************************************************************/
static void sys_led_task(void)
{
  static u32 tick;

  tick++;

  if (tick > 25)
  {
    tick = 0;
    HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_9); /*LED2*/
  }
}

/***************************************************************************
Function	: echo_i2c_msg
Date      : 2021.08.12
Editor		: JKS
Version		: 1.0
Descript 	: i2c task message
***************************************************************************/
void echo_i2c_msg(u8 msg)
{
  s_upk.buf[7] = msg;
  uart_transfer_task(2);
  if (msg == 0xff)
  {
  }
  else
  {
  }
}

/***************************************************************************
Function	: i2c task
Date      : 2021.10.01
Editor		: JKS
Version		: 1.2
Descript 	: i2c r/w event
***************************************************************************/
static void i2c_task(void)
{
  static u8 scope;
  u8 type, mode, cnt;
  u16 dsize, asize, clk;

  if (s_bit.scope_act)
  {
    scope = 1;
    s_bit.scope_act = 0;
  }

  clk = g_i2c_clk;
  s_bit.i2c_act = 1;
  type = s_upk.buf[7] & 0x10;
  asize = s_upk.buf[7] & 0x0f;
  g_i2c_clk = g_i2c_info[s_upk.buf[8]];
  dsize = _8u16(s_upk.buf + 9);
  g_i2c_id = s_upk.buf[11];

  if (type == 0x10) /* i2c read mode */
  {
    if (new_i2c_write_task(g_i2c_id, (u8 *)s_upk.buf + 12, asize, g_i2c_clk) != I2C_ACK)
    {
      echo_i2c_msg(0xff);
    }
    else
    {
      new_i2c_read_task(g_i2c_id, (u8 *)s_upk.buf + (12 + asize), dsize, g_i2c_clk);
      uart_transfer_task(BHD6 + asize + dsize);
    }
  }
  else /* i2c write mode */
  {
    if (new_i2c_write_task(g_i2c_id, (u8 *)s_upk.buf + 12, dsize + asize, g_i2c_clk) != I2C_ACK)
    {
      echo_i2c_msg(0xff);
    }
    else
    {
      echo_i2c_msg(0x00);
    }
  }

  g_i2c_clk = clk;
  s_bit.i2c_act = 0;
  s_bit.scope_act = scope;
  scope = 0;
}

u8 i2c_8bit_w(u8 addr, u8 data)
{
  return i2c_write_task(g_i2c_id, addr, I2C_8BIT, &data, 1, g_i2c_clk);
}

u8 i2c_8bit_r(u8 addr)
{
  u8 data;
  i2c_read_task(g_i2c_id, addr, I2C_8BIT, &data, 1, g_i2c_clk);
  return data;
}

int init_rtp_task(void)
{
  static u8 p;

  /* basic setup */
  if (s_upk.buf[8] == PLAY)
  {
    /* first time register clear*/
    if (s_bit.rtp_act == 0)
    {
      p = 0;
      s_rtp.play = 0;
      s_rtp.cnt[0] = 0;
      s_rtp.cnt[1] = 0;
    }

    s_rtp.play++;
    s_rtp.cnt[p] = 0;
    s_bit.rtp_act = 1;
    s_rtp.dev = s_upk.buf[7];
    s_rtp.size[p] = _8u16(s_upk.buf + 9);

    for (int i = 0; i < s_rtp.size[p]; i++)
    {
      s_rtp.buf[p][i] = s_upk.buf[11 + i];
    }
    p ^= 1;

    /* device setup */
    if (s_rtp.dev == DW7800)
    {
      g_i2c_id = 0xB2;
      g_i2c_clk = I2C_1MHZ;
    }
    else if (s_rtp.dev == DW7912)
    {
    }
    else if (s_rtp.dev == DW7914)
    {
    }
  }
  else /* stop */
  {
    if (s_rtp.dev == DW7800)
    {
      /* dw7800 sw reset */
      i2c_8bit_w(0x05, 0x01);
    }
    else if (s_rtp.dev == DW7912)
    {
    }
    else if (s_rtp.dev == DW7914)
    {
    }
    s_bit.rtp_act = 0;
    sys_timer_set(TIMER_7, STOP, 5000); /* Stop 5ms timer */

    /* for hand shake */
    s_upk.buf[8] = 0xff;
    uart_transfer_task(5);
  }

  /* is first time run?*/
  if (s_rtp.play > 0)
  {
    sys_timer_set(TIMER_7, PLAY, 5000); /* Play 5ms timer */
  }

  return 0;
}

int play_rtp_task(void)
{
  u8 fifo;
  u16 play;
  static u8 echo, p;

  /* stop play */
  if (!s_bit.rtp_act)
  {
    p = 0;
    echo = 0;
    return 0;
  }

  if (s_rtp.dev == DW7800)
  {
    /* step : check fifo */
    fifo = i2c_8bit_r(0x03);

    if (fifo < 127)
    {
      /* what if fifo is empty?*/
      if (s_rtp.size[p] > s_rtp.cnt[p]) /*  */
      {
        play = 127 - fifo;

        if (play > (s_rtp.size[p] - s_rtp.cnt[p]))
        {
          play = s_rtp.size[p] - s_rtp.cnt[p];
        }

        i2c_write_task(g_i2c_id, 0x04, I2C_8BIT, ((u8 *)s_rtp.buf[p] + s_rtp.cnt[p]), play, g_i2c_clk);
        s_rtp.cnt[p] += play;

        /* hand shake */
        if (echo == 0)
        {
          echo = 1;
          _16u8(&s_rtp.cnt[p], s_upk.buf + 9);
          uart_transfer_task(5);
        }
      }
      else
      {
        s_rtp.play--;

        if (s_rtp.play == 0)
        {
          /* play end */
          p = 0;
          echo = 0;
          s_bit.rtp_act = 0;
          sys_timer_set(TIMER_7, STOP, 0);

          /* for hand shake */
          s_upk.buf[8] = 0xff;
          uart_transfer_task(5);
        }
        else
        {
          /* play continue */
          p ^= 1;
          echo = 0;
          s_rtp.cnt[p] = 0;
        }
      }
    }

    else if (s_rtp.dev == DW7912)
    {
    }

    else if (s_rtp.dev == DW7914)
    {
    }
  }

  return 0;
}

/***************************************************************************
Function	: main_func_state_machine
Date      : 2021.08.12
Editor		: JKS
Version		: 1.0
Descript 	: max packet 4kbyte
***************************************************************************/
static void main_func_state_machine(void)
{
  if (s_bit.rx_done == 1)
  {
    switch (s_upk.buf[6])
    {
    case SYS_FSM:
      board_set_task();
      break;
    case I2C_FSM:
      i2c_task();
      break;
    case RTP_FSM:
      init_rtp_task();
      break;
    }
    s_bit.rx_done = 0;
  }
}

/* USER CODE END 0 */
u8 g_uart_rx_done = 0;
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_SPI3_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  init_redhoah_system();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    main_func_state_machine();
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
   */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief DAC Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
   */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
   */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
   */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */
}

/**
 * @brief SPI5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  htim1.Init.Prescaler = g_tmr_psc;
  htim1.Init.Period = g_tmr_period;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */
  htim7.Init.Prescaler = g_tmr_psc;
  htim7.Init.Period = g_tmr_period;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END TIM7_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  huart1.Init.BaudRate = UART1_BUAD;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB13 PB14
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD4 PD5 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG9 PG10 PG11 PG12
                           PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/* USER CODE BEGIN 4 */
/***************************************************************************
Function	: HAL_GPIO_EXTI_Callback
Date		: 2021.08.12
Editor		: JKS
Version		: 1.0
Descript 	: sw1, sw2 key
***************************************************************************/
void HAL_GPIO_EXTI_Callback(u16 GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_2)
  {
    /*user code here*/
    LED5_T;
    uart_transfer_task(2);
  }
  if (GPIO_Pin == GPIO_PIN_3)
  {
    /*user code here*/
    LED4_T;
  }
}

/***************************************************************************
Function	: HAL_UART_RxCpltCallback
Date			: 2021.08.12
Editor		: JKS
Version		: 1.0
Descript 	: degign for speed
***************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  u8 cks;
  static u32 num, step, get_size;

  s_upk.buf[num++] = s_upk.get;

  switch (step)
  {
  case 0:
    if (s_upk.buf[0] != 0xAE)
      num = 0;
    else
      step = 1;
    break;
  case 1:
    if (num >= BHD6)
    {
      s_upk.get_size = _8u16(s_upk.buf + 2);
      get_size = s_upk.get_size + BHD6;
      step = 2;
    }
    break;
  case 2:
    if (num >= get_size)
    {
      cks = 0;
      for (int i = 0; i < s_upk.get_size; i++)
      {
        cks += s_upk.buf[6 + i];
      }

      /*checksum fail*/
      if (cks != s_upk.buf[4])
      {
        s_upk.buf[6] = 0x00;
        s_upk.buf[7] = 0xEC;
        s_upk.buf[8] = 0xE1;
        uart_transfer_task(3);
      }
      else
      {
        s_bit.rx_done = 1;
      }
      num = 0;
      step = 0;
    }
    break;
  }

  HAL_UART_Receive_IT(&huart1, &s_upk.get, 1);
#if 0
  HAL_UART_Receive_DMA(&huart1, &s_upk.get, 1);
#endif
}

/***************************************************************************
Function	: HAL_UART_TxCpltCallback
Date			: 2021.08.12
Editor		: JKS
Version		: 1.0
Descript 	: Reserve
***************************************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete*/
  // P0_T;
  s_bit.tx_done = 1;
}

/***************************************************************************
Function	: HAL_TIM_PeriodElapsedCallback
Date			: 2021.08.12
Editor		: JKS
Version		: 1.0
Descript 	: degign for speed
***************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  if (htim->Instance == TIM1)
  {
    sys_led_task();
  }
  else if (htim->Instance == TIM7)
  {
    play_rtp_task();
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
