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
#include "mpu6050.h"

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;

// USART variables
#ifdef SERIAL_CONTROL
SerialSideboard Sideboard;
#endif

#if defined(SERIAL_DEBUG) || defined(SERIAL_FEEDBACK)
static uint8_t rx_buffer[SERIAL_BUFFER_SIZE]; 	// USART Rx DMA circular buffer
static uint32_t rx_buffer_len = ARRAY_LEN(rx_buffer);
#endif

#ifdef SERIAL_FEEDBACK
SerialFeedback Feedback;
SerialFeedback FeedbackRaw;
uint16_t timeoutCntSerial = 0; 					// Timeout counter for Rx Serial command
uint8_t timeoutFlagSerial = 0; 					// Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)
static uint32_t Feedback_len  = sizeof(Feedback);
#endif

// MPU variables
ErrorStatus mpuStatus; 							// holds the MPU-6050 status: SUCCESS or ERROR

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

	for (i = 0; i < 3; i++) {
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


/* =========================== Input Initialization Function =========================== */
void input_init(void) {
	#ifdef SERIAL_CONTROL
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&Sideboard, sizeof(Sideboard));
	#endif
	#if defined(SERIAL_DEBUG) || defined(SERIAL_FEEDBACK)
		HAL_UART_Receive_DMA (&huart2, (uint8_t *)rx_buffer, sizeof(rx_buffer));
	#endif

	intro_demo_led(100);												// Short LEDs intro demo with 100 ms delay. This also gives some time for the MPU-6050 to power-up.	

	#ifdef MPU_SENSOR_ENABLE
		if(mpu_config()) { 												// IMU MPU-6050 config
			mpuStatus = ERROR;
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);  // Turn on RED LED
		}
		else {
			mpuStatus = SUCCESS;
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);  // Turn on GREEN LED
		}
		mpu_handle_input('h'); 						  					// Print the User Help commands to serial
	#else
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);  	// Turn on GREEN LED
	#endif
}

/* =========================== USART READ Functions =========================== */

/*
 * Check for new data received on USART with DMA: refactored function from https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
 * - this function is called for every USART IDLE line detection, in the USART interrupt handler
 */
void usart_rx_check(void)
{
	#ifdef SERIAL_DEBUG
	static uint32_t old_pos;
	uint32_t pos;

	pos = rx_buffer_len - __HAL_DMA_GET_COUNTER(huart2.hdmarx); 				// Calculate current position in buffer, Rx: DMA1_Channel6->CNDTR, Tx: DMA1_Channel7
    if (pos != old_pos) {                       								// Check change in received data		
        if (pos > old_pos) {      												// "Linear" buffer mode: check if current position is over previous one
        	usart_process_debug(&rx_buffer[old_pos], pos - old_pos);   			// Process data
        } else {																// "Overflow" buffer mode
 			usart_process_debug(&rx_buffer[old_pos], rx_buffer_len - old_pos); 	// First Process data from the end of buffer            
            if (pos > 0) {														// Check and continue with beginning of buffer				
				usart_process_debug(&rx_buffer[0], pos);						// Process remaining data 			
			}
        }
    }
    old_pos = pos;                              								// Updated old position

	if (old_pos == rx_buffer_len) { 											// Check and manually update if we reached end of buffer
        old_pos = 0;
    }
	#endif // SERIAL_DEBUG

	#ifdef SERIAL_FEEDBACK
	static uint32_t old_pos;
	uint32_t pos;
	uint8_t *ptr;	
    pos = rx_buffer_len - __HAL_DMA_GET_COUNTER(huart2.hdmarx); 				// Calculate current position in buffer, Rx: DMA1_Channel6->CNDTR, Tx: DMA1_Channel7	
    if (pos != old_pos) {                       								// Check change in received data
		ptr = (uint8_t *)&FeedbackRaw;											// Initialize the pointer with FeedbackRaw address
        if (pos > old_pos && (pos - old_pos) == Feedback_len) {      			// "Linear" buffer mode: check if current position is over previous one AND data length equals expected length
			memcpy(ptr, &rx_buffer[old_pos], Feedback_len); 					// Copy data. This is possible only if FeedbackRaw is contiguous! (meaning all the structure members have the same size)
			usart_process_data(&FeedbackRaw, &Feedback);						// Process data
        } else if ((rx_buffer_len - old_pos + pos) == Feedback_len) {			// "Overflow" buffer mode: check if data length equals expected length
			memcpy(ptr, &rx_buffer[old_pos], rx_buffer_len - old_pos); 			// First copy data from the end of buffer
            if (pos > 0) {														// Check and continue with beginning of buffer
				ptr += rx_buffer_len - old_pos;									// Move to correct position in FeedbackRaw		
				memcpy(ptr, &rx_buffer[0], pos); 								// Copy remaining data
            }
			usart_process_data(&FeedbackRaw, &Feedback); 						// Process data
        }		
    }	
    old_pos = pos;                              								// Update old position
	if (old_pos == rx_buffer_len) { 											// Check and manually update if we reached end of buffer
        old_pos = 0;
    }	
	#endif // SERIAL_FEEDBACK
}

/*
 * Process Rx debug user command input
 */
#ifdef SERIAL_DEBUG
void usart_process_debug(uint8_t *userCommand, uint32_t len)
{
	for (; len > 0; len--, userCommand++) {
		if (*userCommand != '\n' && *userCommand != '\r') { 	// Do not accept 'new line' and 'carriage return' commands
			log_i("Command = %c\n", *userCommand);						
			mpu_handle_input(*userCommand);
		}
    }
}
#endif // SERIAL_DEBUG

/*
 * Process Rx data
 * - if the Feedback_in data is valid (correct START_FRAME and checksum) copy the Feedback_in to Feedback_out
 */
#ifdef SERIAL_FEEDBACK
void usart_process_data(SerialFeedback *Feedback_in, SerialFeedback *Feedback_out)
{	
	uint16_t checksum;
	if (Feedback_in->start == SERIAL_START_FRAME) {
		checksum = (uint16_t)(Feedback_in->start ^ Feedback_in->cmd1 ^ Feedback_in->cmd2 ^ Feedback_in->speedR_meas ^ Feedback_in->speedL_meas
							^ Feedback_in->batVoltage ^ Feedback_in->boardTemp ^ Feedback_in->cmdLed);
		if (Feedback_in->checksum == checksum) {					
			*Feedback_out = *Feedback_in;
			timeoutCntSerial  = 0;		// Reset timeout counter
			timeoutFlagSerial = 0; 		// Clear timeout flag
		}
	}
}
#endif // SERIAL_FEEDBACK

/*
 * UART User Error Callback
 * - According to the STM documentation, when a DMA transfer error occurs during a DMA read or a write access,
 *   the faulty channel is automatically disabled through a hardware clear of its EN bit
 * - For hoverboard applications, the UART communication can be unrealiable, disablind the DMA transfer
 * - therefore the DMA needs to be re-started
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uartHandle) {
	#if defined(SERIAL_DEBUG) || defined(SERIAL_FEEDBACK)
	if(uartHandle->Instance == USART2) {
		HAL_UART_Receive_DMA (uartHandle, (uint8_t *)rx_buffer, sizeof(rx_buffer));
	}
	#endif
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








