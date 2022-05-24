/**
  ******************************************************************************
  * @file		: ae_i2c.c
  * @version	: 210812A
  * @creaor		: magi8051
  * @update		: 21.08.12
  ******************************************************************************
  * @attention	: Design for BlueBerry Board
  * 2020.03.11	: i2c time margin change!
	2019.11.13 i2c pin change
	- SCL1_PIN  GPIO_PIN_8
	- SDA1_PIN  GPIO_PIN_7
	2020.07.30 BBB Rev Define code add - JKS
	2021.08.12 I2C CKL Fixed - JKS, 3Mhz, 2Mhz, 1Mhz, 400Khz
	2021.09.15 I2C Write & Read Code Optimize
	2021.11.29 Setup & Hold time margin code
 ******************************************************************************
* Do Not Change!! *
*/

#include "stm32f4xx_hal.h"
#include "ae_i2c.h"

/*user free define code*/
GPIO_TypeDef *I2C_BASS;
uint16_t SCL_PIN;
uint16_t SDA_PIN;
/*2019.11.13 i2c pin change*/
//#define SCL1_PIN 	GPIO_PIN_8
//#define SDA1_PIN 	GPIO_PIN_7

/*can not redefine!!*/
#define SDA(in) I2C_BASS->BSRR = (in) ? SDA_PIN : SDA_PIN << 16
#define SCL(in) I2C_BASS->BSRR = (in) ? SCL_PIN : SCL_PIN << 16

/*ACK:"0", NoACK:"1"*/
#define I2C_NACK (I2C_BASS->IDR & SDA_PIN) ? 1 : 0
#define I2C_DAT (I2C_BASS->IDR & SDA_PIN) ? 1 : 0

/*Select i2c for the channel you want for board.*/
void init_i2c(uint32_t type, uint32_t ch)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_BASS = GPIOB;

	if (ch == 1)
	{
		if (type == 0x00)
		{ // rednoah3
			SCL_PIN = GPIO_PIN_8;
			SDA_PIN = GPIO_PIN_7;
		}
	}
	else
	{
		I2C_BASS = GPIOD;
		SCL_PIN = GPIO_PIN_8;
		SDA_PIN = GPIO_PIN_9;
	}

	/*Configure I2Cx Port */
	GPIO_InitStruct.Pin = SCL_PIN | SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(I2C_BASS, &GPIO_InitStruct);
	HAL_GPIO_WritePin(I2C_BASS, SDA_PIN | SCL_PIN, GPIO_PIN_SET);
}

/* design for i2c delay (5cycle) */
static void asm_delay(void)
{
	__asm volatile("NOP");
	// __asm volatile("NOP");
	// __asm volatile("NOP");
	// __asm volatile("NOP");
	// __asm volatile("NOP");
}

static void idelay(volatile uint32_t tmout)
{
	if (tmout < 1)
	{
		__asm volatile("NOP");
		__asm volatile("NOP");
		__asm volatile("NOP");
		__asm volatile("NOP");
		__asm volatile("NOP");
	}

	else
	{
		for (volatile int i = 0; i < tmout; i++)
		{
		}
	}
}

uint32_t init_hs_i2c(uint8_t dat, uint32_t tmout)
{
	uint8_t tmp;
	uint8_t ack;

	// Start
	SDA(0);
	idelay(tmout);
	tmp = 0;
	ack = 0;
	SCL(0);
	tmp = dat;

	for (int i = 0; i < 8; i++)
	{
		SDA((tmp & 0x80) ? 1 : 0);
		idelay(tmout);
		SCL(1);
		idelay(tmout);
		SCL(0);
		tmp <<= 1;
	}

	// No ACK Event
	SDA(1); // Set High-z
	idelay(tmout);
	SCL(1);
	idelay(tmout);
	SCL(0);
	idelay(tmout);

	// no stop function
	// SDA(0);     // Set Low    not used 2017.01.25
	idelay(tmout);
	SCL(1);
	idelay(tmout);
	SDA(1);

	return ack;
}

uint32_t i2c_write_task(uint8_t id, uint16_t addr, uint16_t type, uint8_t *dat, uint16_t size, uint16_t tmout)
{
	uint8_t tmp, ack;

#ifdef I2C_HS_MODE
	init_hs_i2c(0x08, 40);
#endif

	/*START*/
	SDA(0);
	idelay(tmout); // 100ns
	ack = 0;
	tmp = id;
	SCL(0);
	asm_delay();

	/*Start ID*/
	for (int i = 0; i < 8; i++)
	{
		SDA((tmp & 0x80) ? 1 : 0);
		tmp <<= 1;
		idelay(tmout);
		SCL(1);
		idelay(tmout);
		SCL(0);
		// asm_delay();
	}

	/*ACK*/
	SDA(1);
	idelay(tmout);
	SCL(1);
	ack = I2C_NACK;
	idelay(tmout);
	SCL(0);
	asm_delay();

	if (type == 2)
	{
		// 16bit address
		tmp = addr >> 8;
		for (int i = 0; i < 8; i++)
		{
			SDA((tmp & 0x80) ? 1 : 0);
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			tmp <<= 1;
			SCL(0);
			asm_delay();
		}
		/*ACK*/
		SDA(1);
		idelay(tmout);
		SCL(1);
		// ack = I2C_NACK;
		idelay(tmout);
		tmp = addr;
		SCL(0);
		asm_delay();

		for (int i = 0; i < 8; i++)
		{
			SDA((tmp & 0x80) ? 1 : 0);
			tmp <<= 1;
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			SCL(0);
			asm_delay();
		}
		/*ACK*/
		SDA(1);
		idelay(tmout);
		SCL(1);
		// ack = I2C_NACK;
		idelay(tmout);
		SCL(0);
		idelay(tmout);
	}
	else
	{
		// 8bit address
		tmp = addr;
		for (int i = 0; i < 8; i++)
		{
			SDA((tmp & 0x80) ? 1 : 0);
			tmp <<= 1;
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			SCL(0);
			asm_delay();
		}
		/*ACK*/
		SDA(1);
		idelay(tmout);
		SCL(1);
		// ack = I2C_NACK;
		idelay(tmout);
		SCL(0);
		asm_delay();
	}

	while (size--)
	{
		tmp = (*dat++);
		for (int i = 0; i < 8; i++)
		{
			SDA((tmp & 0x80) ? 1 : 0);
			tmp <<= 1;
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			SCL(0);
			asm_delay();
		}
		/*ACK*/
		SDA(1);
		idelay(tmout);
		SCL(1);
		// ack = I2C_NACK;
		idelay(tmout);
		SCL(0);
		idelay(tmout);
	}

	/*STOP*/
	SDA(0);
	idelay(tmout);
	SCL(1);
	idelay(tmout);
	SDA(1);

	return ack;
}

uint32_t i2c_read_task(uint8_t id, uint16_t addr, uint16_t type, uint8_t *dat, uint16_t size, uint16_t tmout)
{
	uint8_t ack, tmp;

	ack = i2c_write_task(id, addr, type, dat, 0, tmout);

	/*START*/
	SDA(0);
	idelay(tmout);
	tmp = id + 1;
	SCL(0);
	asm_delay();

	/*R-ID*/
	for (int i = 0; i < 8; i++)
	{
		SDA((tmp & 0x80) ? 1 : 0);
		idelay(tmout);
		SCL(1);
		idelay(tmout);
		tmp <<= 1;
		SCL(0);
		asm_delay();
	}
	/*ACK*/
	SDA(1);
	idelay(tmout);
	SCL(1);
	idelay(tmout);
	tmp = 0;
	SCL(0);

	/*R-DATA*/
	while (size--)
	{ /*read sda pin*/
		for (int i = 0; i < 8; i++)
		{
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			tmp |= I2C_DAT;
			if (i < 7)
				tmp <<= 1;
			SCL(0);
			// asm_delay();
		}
		(*dat++) = tmp;
		tmp = 0;

		if (size)
		{
			SDA(0); /*Force NoACK*/
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			SCL(0);
			asm_delay();
			SDA(1);
		}
		else
		{ /*Noack*/
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			SCL(0);
			asm_delay();
		}
	}

	/*STOP*/
	SDA(0);
	idelay(tmout);
	SCL(1);
	idelay(tmout);
	SDA(1);

	return ack;
}

uint32_t new_i2c_write_task(uint8_t id, uint8_t *dat, uint16_t size, uint16_t tmout)
{
	uint8_t tmp, ack;

	/*START*/
	SDA(0);
	idelay(tmout); // 100ns ?
	ack = 0;
	tmp = id;
	SCL(0);
	asm_delay();

	/*Start ID*/
	for (int i = 0; i < 8; i++)
	{
		SDA((tmp & 0x80) ? 1 : 0);
		tmp <<= 1;
		idelay(tmout);
		SCL(1);
		idelay(tmout);
		SCL(0);
		asm_delay();
	}

	/*ACK*/
	SDA(1);
	idelay(tmout);
	SCL(1);
	ack = I2C_NACK;
	idelay(tmout);
	SCL(0);
	asm_delay();

	while (size--)
	{
		tmp = (*dat++);
		for (int i = 0; i < 8; i++)
		{
			SDA((tmp & 0x80) ? 1 : 0);
			tmp <<= 1;
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			SCL(0);
			asm_delay();
		}
		/*ACK*/
		SDA(1);
		idelay(tmout);
		SCL(1);
		// ack = I2C_NACK;
		idelay(tmout);
		SCL(0);
		asm_delay();
	}

	/*STOP*/
	SDA(0);
	idelay(tmout);
	SCL(1);
	idelay(tmout);
	SDA(1);

	return ack;
}

uint32_t new_i2c_read_task(uint8_t id, uint8_t *dat, uint16_t size, uint16_t tmout)
{
	uint8_t tmp;

	/*START*/
	SDA(0);
	idelay(tmout);
	tmp = id + 1;
	SCL(0);
	asm_delay();

	/*R-ID*/
	for (int i = 0; i < 8; i++)
	{
		SDA((tmp & 0x80) ? 1 : 0);
		idelay(tmout);
		SCL(1);
		idelay(tmout);
		tmp <<= 1;
		SCL(0);
		asm_delay();
	}
	/*ACK*/
	SDA(1);
	idelay(tmout);
	SCL(1);
	idelay(tmout);
	tmp = 0;
	SCL(0);
	idelay(tmout);

	/*R-DATA*/
	while (size--)
	{ /*read sda pin*/
		for (int i = 0; i < 8; i++)
		{
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			tmp |= I2C_DAT;
			if (i < 7)
				tmp <<= 1;
			SCL(0);
			asm_delay();
		}
		(*dat++) = tmp;
		tmp = 0;

		if (size)
		{
			SDA(0); /*Force NoACK*/
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			SCL(0);
			asm_delay();
			SDA(1);
		}
		else
		{ /*Noack*/
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			SCL(0);
			asm_delay();
		}
	}
	/*STOP*/
	SDA(0);
	idelay(tmout);
	SCL(1);
	idelay(tmout);
	SDA(1);

	return 0;
}

uint8_t i2c_pin_state(void)
{
	uint8_t sda, scl;
	sda = (I2C_BASS->IDR & SDA_PIN) ? 1 : 0;
	scl = (I2C_BASS->IDR & SCL_PIN) ? 1 : 0;
	return (scl << 1) | sda;
}