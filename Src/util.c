/**
  * This file is part of the hoverboard-sideboard-hack project.
  *
  * Copyright (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Includes
#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "i2c.h"
#include "defines.h"
#include "config.h"
#include "util.h"

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;

/* =========================== General Functions =========================== */

void consoleLog(char *message)
{
  #ifdef SERIAL_DEBUG
	log_i("%s", message);
  #endif
}

void get_tick_count_ms(unsigned long *count)
{
	*count = HAL_GetTick();
}

/* retarget the C library printf function to the USART */
#ifdef SERIAL_DEBUG	
	#ifdef __GNUC__
		#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
	#else
		#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
	#endif
	PUTCHAR_PROTOTYPE {
		HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1000);  
		return ch;
	}
	
	#ifdef __GNUC__
		int _write(int file, char *data, int len) {
			int i;
			for (i = 0; i < len; i++) { __io_putchar( *data++ );}
			return len; 
		}
	#endif
#endif

void intro_demo_led(uint32_t tDelay)
{
	int i;

	for (i = 0; i < 6; i++) {
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);		
		HAL_Delay(tDelay);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);	
		HAL_Delay(tDelay);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);	
		HAL_Delay(tDelay);
	}
	
	for (i = 0; i < 2; i++) {
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
		HAL_Delay(tDelay);
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
	}		
		
}

/* =========================== I2C WRITE Functions =========================== */

/*
 * write bytes to chip register
 */
int8_t i2c_writeBytes(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{	
	// !! Using the I2C Interrupt will fail writing the DMP.. could be that DMP memory writing requires more time !! So use the I2C without interrupt.
	// HAL_I2C_Mem_Write_IT(&hi2c1, slaveAddr << 1, regAddr, 1, data, length);
	// while(HAL_I2C_STATE_READY != HAL_I2C_GetState(&hi2c1));                     // Wait until all data bytes are sent/received
	// return 0;

	return HAL_I2C_Mem_Write(&hi2c1, slaveAddr << 1, regAddr, 1, data, length, 100); 		// Address is shifted one position to the left. LSB is reserved for the Read/Write bit.

}

/*
 * write 1 byte to chip register
 */
int8_t i2c_writeByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t data)
{
	return i2c_writeBytes(slaveAddr, regAddr, 1, &data);
}

/*
 * write one bit to chip register
 */
int8_t i2c_writeBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    i2c_readByte(slaveAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return i2c_writeByte(slaveAddr, regAddr, b);
}

/* =========================== I2C READ Functions =========================== */

/*
 * read bytes from chip register
 */
int8_t i2c_readBytes(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t *data) 
{	  
	// !! Using the I2C Interrupt will fail writing the DMP.. could be that DMP memory writing requires more time !! So use the I2C without interrupt.
	// HAL_I2C_Mem_Read(&hi2c1, slaveAddr << 1, regAddr, 1, data, length);
	// while(HAL_I2C_STATE_READY != HAL_I2C_GetState(&hi2c1));                   // Wait until all data bytes are sent/received
	// return 0;

	return HAL_I2C_Mem_Read(&hi2c1, slaveAddr << 1, regAddr, 1, data, length, 100); 		// Address is shifted one position to the left. LSB is reserved for the Read/Write bit.
  
}

/*
 * read 1 byte from chip register
 */
int8_t i2c_readByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t *data)
{
	return i2c_readBytes(slaveAddr, regAddr, 1, data);
}

/*
 * read 1 bit from chip register
 */
int8_t i2c_readBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
    uint8_t b;
    int8_t status = i2c_readByte(slaveAddr, regAddr, &b);
    *data = b & (1 << bitNum);
    return status;
}








